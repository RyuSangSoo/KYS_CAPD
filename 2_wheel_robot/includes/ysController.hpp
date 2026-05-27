#ifndef YSCONTROLLER_HPP
#define YSCONTROLLER_HPP

#include "header.hpp"
#include "ysState.hpp"

struct WheelEffortCommand
{
    // 좌/우 바퀴 effort controller로 보낼 최종 명령
    double left = 0.0;
    double right = 0.0;
};

class ysController
{
public:
    // wheel_radius / wheel_base는 차동구동 inverse kinematics에 사용한다.
    ysController(double wheel_radius, double wheel_base);

    // 조이스틱 입력을 적분해서 robot.Ref.Pos.Base에 목표 궤적을 만든다.
    // 스틱을 계속 밀고 있으면 reference가 계속 이동한다.
    void UpdateReference(double dt, double joy_dx, double joy_dyaw);

    // 현재 추정 자세를 reference로 복사한다.
    // 현재 위치를 그대로 유지하고 싶을 때 사용한다.
    void ResetReferenceToCurrentPose();

    // reference를 원점으로 되돌리고 PD 오차 메모리도 같이 초기화한다.
    void ResetReferenceToOrigin();

    // 현재 자세를 기준으로 비상 정지 상태를 만든다.
    // reference를 현재 자세로 맞추고 derivative history를 지운다.
    void EmergencyStop();

    // reference 추종 오차를 좌/우 바퀴 effort command로 바꾼다.
    // 내부 순서는 다음과 같다.
    // 1) pose tracking controller
    // 2) 전진/회전 제어입력을 좌/우 바퀴 effort로 분배
    WheelEffortCommand ComputeWheelEffort(double dt);

private:
    // 컨트롤러 내부에서만 쓰는 유틸 함수
    double ClampValue(double value, double min_value, double max_value) const;
    double Deadband(double value, double band) const;
    double NormalizeAngle(double angle) const;
    double ApplyRateLimit(double current_value, double target_value, double max_rate, double dt) const;

    // 로봇 형상 파라미터
    double wheel_radius_;
    double wheel_base_;

    // 위치/자세 추종 게인
    double kp_x_ = 50.0;
    double kp_y_ = 50.0;
    double kp_yaw_ = 50.0;
    double kd_x_ = 5.0;
    double kd_y_ = 5.0;
    double kd_yaw_ = 5.0;

    // 속도 및 effort 제한값
    double max_v_ = 0.5;
    double max_w_ = 2.0;
    double max_ref_accel_v_ = 0.8;
    double max_ref_accel_w_ = 3.0;
    double max_wheel_vel_ = 20.0;
    double max_effort_ = 20.0;
};

#endif
