#include "ysController.hpp"

ysController::ysController(double wheel_radius, double wheel_base)
: wheel_radius_(wheel_radius), wheel_base_(wheel_base)
{
    // 프로젝트 전체에서 목표 상태는 robot.Ref 하나로 관리한다.
    // 생성 시점에는 원점 기준 reference로 시작한다.
    ResetReferenceToOrigin();
}

void ysController::UpdateReference(double dt, double joy_dx, double joy_dyaw)
{
    // 조이스틱의 아주 작은 흔들림은 무시해서
    // reference가 계속 떠다니는 현상을 막는다.
    const double joy_v_target = Deadband(joy_dx, 0.02);
    const double joy_w_target = Deadband(joy_dyaw, 0.02);

    // 조이스틱 목표 속도를 바로 쓰지 않고
    // 내부 reference 속도 상태가 최대 가속도 제한을 지키며 따라가게 만든다.
    // 이렇게 하면 속도가 계단처럼 튀지 않고 사다리꼴 프로파일처럼 증가/감소한다.
    robot.Ref.Vel.Base.x = ApplyRateLimit(
        robot.Ref.Vel.Base.x, joy_v_target, max_ref_accel_v_, dt);
    robot.Ref.Vel.Base.yaw = ApplyRateLimit(
        robot.Ref.Vel.Base.yaw, joy_w_target, max_ref_accel_w_, dt);
    robot.Ref.Vel.Base.y = 0.0;

    // 가감속 제한이 적용된 reference 속도를 적분해서
    // 최종 목표 자세(reference pose)를 생성한다.
    robot.Ref.Pos.Base.x += robot.Ref.Vel.Base.x * std::cos(robot.Ref.Pos.Base.yaw) * dt;
    robot.Ref.Pos.Base.y += robot.Ref.Vel.Base.x * std::sin(robot.Ref.Pos.Base.yaw) * dt;
    robot.Ref.Pos.Base.yaw += robot.Ref.Vel.Base.yaw * dt;
    robot.Ref.Pos.Base.yaw = NormalizeAngle(robot.Ref.Pos.Base.yaw);
}

void ysController::ResetReferenceToCurrentPose()
{
    // 현재 추정 자세를 reference로 복사하면
    // 제어기는 현재 위치를 유지하려고 동작하게 된다.
    robot.Ref.Pos.Base.x = robot.Est.Pos.Base.x;
    robot.Ref.Pos.Base.y = robot.Est.Pos.Base.y;
    robot.Ref.Pos.Base.yaw = robot.Est.Pos.Base.yaw;
    robot.Ref.Pos.Wheel.left = robot.Est.Pos.Wheel.left;
    robot.Ref.Pos.Wheel.right = robot.Est.Pos.Wheel.right;
}

void ysController::ResetReferenceToOrigin()
{
    // 목표 자세를 월드 원점으로 초기화한다.
    robot.Ref.Pos.Base.x = 0.0;
    robot.Ref.Pos.Base.y = 0.0;
    robot.Ref.Pos.Base.yaw = 0.0;

    // reference 속도도 같이 0으로 만들어서
    // Ref 구조체 내부 상태가 서로 모순되지 않게 맞춘다.
    robot.Ref.Vel.Base.x = 0.0;
    robot.Ref.Vel.Base.y = 0.0;
    robot.Ref.Vel.Base.yaw = 0.0;
    robot.Ref.Vel.Wheel.left = 0.0;
    robot.Ref.Vel.Wheel.right = 0.0;
    robot.Ref.Tau.Wheel.left = 0.0;
    robot.Ref.Tau.Wheel.right = 0.0;
    robot.Ref.Pos.Wheel.left = 0.0;
    robot.Ref.Pos.Wheel.right = 0.0;
}

void ysController::EmergencyStop()
{
    // 비상 정지는 odometry를 건드리지 않는다.
    // 단지 reference를 현재 자세에 고정하고
    // 미분항 메모리를 지워서 다시 시작할 때 튀지 않게 만든다.
    ResetReferenceToCurrentPose();
    robot.Ref.Vel.Base.x = 0.0;
    robot.Ref.Vel.Base.y = 0.0;
    robot.Ref.Vel.Base.yaw = 0.0;
    robot.Ref.Vel.Wheel.left = 0.0;
    robot.Ref.Vel.Wheel.right = 0.0;
    robot.Ref.Tau.Wheel.left = 0.0;
    robot.Ref.Tau.Wheel.right = 0.0;
}

WheelEffortCommand ysController::ComputeWheelEffort(double dt)
{
    WheelEffortCommand command;
    const double x = robot.Est.Pos.Base.x;
    const double y = robot.Est.Pos.Base.y;
    const double yaw = robot.Est.Pos.Base.yaw;

    // 월드 좌표계 기준 위치 오차
    const double dx = robot.Ref.Pos.Base.x - x;
    const double dy = robot.Ref.Pos.Base.y - y;

    // 월드 좌표계 오차를 로봇 바디 좌표계 오차로 변환한다.
    // ex: 전후 오차, ey: 좌우 오차
    const double ex = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double ey = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    const double eyaw = NormalizeAngle(robot.Ref.Pos.Base.yaw - yaw);

    // 바깥쪽 PD 제어기의 D 성분은 "목표 속도 - 현재 속도" 형태의
    // 몸체 속도 오차를 사용한다.
    const double evx = robot.Ref.Vel.Base.x - robot.Est.Vel.Base.x;
    const double evy = robot.Ref.Vel.Base.y - robot.Est.Vel.Base.y;
    const double evyaw = robot.Ref.Vel.Base.yaw - robot.Est.Vel.Base.yaw;

    // 바깥쪽 제어기:
    // 위치/자세 오차와 몸체 속도 오차를 이용해
    // 전진 제어입력(u_v)과 회전 제어입력(u_w)을 만든다.
    // 여기서 reference 속도는 위치 레퍼런스를 만드는 데 사용되었고,
    // 현재는 속도 오차 항(evx, evyaw 등)으로만 추종에 반영된다.
    double u_v = kp_x_ * ex + kd_x_ * evx;
    double u_w = kp_y_ * ey + kd_y_ * evy + kp_yaw_ * eyaw + kd_yaw_ * evyaw;

    u_v = ClampValue(u_v, -max_v_, max_v_);
    u_w = ClampValue(u_w, -max_w_, max_w_);

    // 차동구동 inverse kinematics:
    // 몸체 전진/회전 입력을 바퀴 각속도 기준으로 환산한다.
    const double wl_des = (u_v - 0.5 * wheel_base_ * u_w) / wheel_radius_;
    const double wr_des = (u_v + 0.5 * wheel_base_ * u_w) / wheel_radius_;

    robot.Ref.Vel.Wheel.left = ClampValue(wl_des, -max_wheel_vel_, max_wheel_vel_);
    robot.Ref.Vel.Wheel.right = ClampValue(wr_des, -max_wheel_vel_, max_wheel_vel_);

    // 바퀴 레퍼런스 위치도 같이 적분해서 저장한다.
    // UI에서 Ref.Pos.Wheel과 Est.Pos.Wheel을 비교할 수 있게 한다.
    robot.Ref.Pos.Wheel.left += robot.Ref.Vel.Wheel.left * dt;
    robot.Ref.Pos.Wheel.right += robot.Ref.Vel.Wheel.right * dt;

    // 안쪽 바퀴 속도 루프를 두지 않는 현재 구조에서는
    // 역기구학으로 얻은 바퀴 각속도 부호를 그대로 effort 부호에 반영한다.
    command.left = ClampValue(wl_des, -max_effort_, max_effort_);
    command.right = ClampValue(wr_des, -max_effort_, max_effort_);

    robot.Ref.Tau.Wheel.left = command.left;
    robot.Ref.Tau.Wheel.right = command.right;

    return command;
}

double ysController::ClampValue(double value, double min_value, double max_value) const
{
    // 명령이 제한 범위를 넘지 않도록 포화시킨다.
    return std::max(min_value, std::min(value, max_value));
}

double ysController::Deadband(double value, double band) const
{
    // 0 근처의 작은 조이스틱 노이즈를 제거한다.
    if (std::fabs(value) < band)
    {
        return 0.0;
    }

    return value;
}

double ysController::NormalizeAngle(double angle) const
{
    // 각도를 [-pi, pi] 범위로 정규화해서
    // heading error가 불연속적으로 커지지 않게 한다.
    while (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }

    while (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }

    return angle;
}

double ysController::ApplyRateLimit(double current_value, double target_value, double max_rate, double dt) const
{
    // 한 주기 동안 바뀔 수 있는 최대 변화량을 제한해서
    // 목표 속도가 가속도 제한을 가진 사다리꼴 프로파일을 따르도록 만든다.
    const double max_delta = max_rate * dt;
    const double delta = target_value - current_value;

    if (delta > max_delta)
    {
        return current_value + max_delta;
    }

    if (delta < -max_delta)
    {
        return current_value - max_delta;
    }

    return target_value;
}
