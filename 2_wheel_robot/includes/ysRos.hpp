#ifndef YSROS_HPP
#define YSROS_HPP

#include "header.hpp"
#include "ysState.hpp"

class ysRos : public rclcpp::Node
{
private:
    
public:
    ysRos(/* args */);
    ~ysRos();

    // 바퀴 effort command 퍼블리셔
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub;

    // UI/그래프 확인용 상태 퍼블리셔
    // base_state: [x, y, yaw, vx, vy, vyaw]
    // wheel_state: [left_pos, right_pos, left_vel, right_vel, left_tau, right_tau]
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ref_base_state_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr est_base_state_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ref_wheel_state_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr est_wheel_state_pub;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointdata;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr JoySub;

    void JointStateCallBack(const sensor_msgs::msg::JointState::SharedPtr JointMsg);
    void JoyCallBack(const sensor_msgs::msg::Joy::SharedPtr JoyMsg);
    void PublishStates();

    // 조이스틱 입력 스케일
    double max_dx   = 0.4;  // [m/s]
    double max_dyaw = 1.5;  // [rad/s]

    double JOYdx    = 0.0;
    double JOYdyaw  = 0.0;
    double JOYpitch = 0.0;
    double JOYroll  = 0.0;

    double JOYBtnA  = 0.0;
    double JOYBtnB  = 0.0;
    double JOYBtnX  = 0.0;
    double JOYBtnY  = 0.0;
    double JoyOut   = 0.0;
    
};

#endif  // YSROS_HPP
