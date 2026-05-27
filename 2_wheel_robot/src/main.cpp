#include "ysRos.hpp"
#include "ysController.hpp"
#include "ysOdom.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // ROS 노드:
    // 조이스틱 입력 구독, 바퀴 명령 퍼블리시, 상태 토픽 퍼블리시 담당
    auto robot_node = std::make_shared<ysRos>();

    double dt = 0.001;
    rclcpp::Rate loop_rate(1.0 / dt);

    // 좌/우 바퀴 effort controller에 보낼 명령 메시지
    std_msgs::msg::Float64MultiArray WheelCmd;
    WheelCmd.data.resize(2);
    WheelCmd.data[0] = 0.0;
    WheelCmd.data[1] = 0.0;

    // ============================
    // 로봇 파라미터
    // 실제 로봇에 맞게 수정해야 함
    // ============================
    const double wheel_radius = 0.1;   // [m]
    const double wheel_base = 0.40;     // [m]

    // Odom:
    // 측정된 바퀴 속도로부터 robot.Est를 갱신한다.
    ysOdom odom(wheel_radius, wheel_base);

    // Controller:
    // robot.Ref를 만들고 Ref/Est 오차로 바퀴 effort를 계산한다.
    ysController controller(wheel_radius, wheel_base);

    auto prev_time = std::chrono::steady_clock::now();

    while (rclcpp::ok())
    {
        rclcpp::spin_some(robot_node);

        auto now_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now_time - prev_time;
        dt = elapsed.count();
        prev_time = now_time;

        if (dt <= 0.0 || dt > 0.1)
        {
            dt = 0.001;
        }

        // 현재 측정된 바퀴 속도로 odometry를 적분해서 추정 자세를 갱신한다.
        odom.Update(dt);

        // 조이스틱 입력으로 목표 궤적(reference)을 한 스텝 업데이트한다.
        controller.UpdateReference(dt, robot_node->JOYdx, robot_node->JOYdyaw);

        // A 버튼:
        // 현재 로봇 자세를 목표 자세로 설정해서 그 자리를 유지하게 한다.
        if (robot_node->JOYBtnA == 1)
        {
            controller.ResetReferenceToCurrentPose();
        }

        // X 버튼:
        // 추정 자세와 reference를 둘 다 원점으로 초기화한다.
        if (robot_node->JOYBtnX == 1)
        {
            odom.Reset();
            controller.ResetReferenceToOrigin();
        }

        WheelEffortCommand effort;

        // B 버튼:
        // 현재 자세를 기준으로 비상 정지 상태를 만든다.
        // effort는 기본값 0이 유지된다.
        if (robot_node->JOYBtnB == 1)
        {
            controller.EmergencyStop();
        }
        else
        {
            // 일반 추종 모드:
            // reference와 estimate의 차이로부터 바퀴 effort를 계산한다.
            effort = controller.ComputeWheelEffort(dt);
        }

        WheelCmd.data[0] = effort.left;
        WheelCmd.data[1] = effort.right;

        // ros2_control 바퀴 컨트롤러로 effort command를 보낸다.
        robot_node->wheel_pub->publish(WheelCmd);

        // UI에서 그래프를 그릴 수 있도록 Ref/Est 상태를 토픽으로 내보낸다.
        robot_node->PublishStates();

        // odom -> base_link TF를 퍼블리시한다.
        robot_node->PublishOdomTF();

        loop_rate.sleep();
    }

    WheelCmd.data[0] = 0.0;
    WheelCmd.data[1] = 0.0;
    robot_node->wheel_pub->publish(WheelCmd);

    rclcpp::shutdown();

    return 0;
}
