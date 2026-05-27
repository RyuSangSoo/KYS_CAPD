#include "ysRos.hpp"

Robot robot;

ysRos::ysRos() : Node("WheelRobot") {
    wheel_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_effort_controller/commands", 1);
    ref_base_state_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/state/ref/base_state", 1);
    est_base_state_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/state/est/base_state", 1);
    ref_wheel_state_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/state/ref/wheel_state", 1);
    est_wheel_state_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/state/est/wheel_state", 1);

    jointdata = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1, std::bind(&ysRos::JointStateCallBack, this, std::placeholders::_1));

    JoySub      = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, std::bind(&ysRos::JoyCallBack, this, std::placeholders::_1));
}

ysRos::~ysRos() {
    std::cout << "FINISH" << std::endl;
}

void ysRos::JointStateCallBack(const sensor_msgs::msg::JointState::SharedPtr JointMsg) {

    // joint_states의 이름 순서가 항상 같다고 가정하지 않고
    // 이름 기반으로 좌/우 바퀴 값을 찾아서 상태에 저장한다.
    std::unordered_map<std::string, double> pos_map;
    std::unordered_map<std::string, double> vel_map;

    for (size_t i = 0; i < JointMsg->name.size(); ++i) {
        pos_map[JointMsg->name[i]] = JointMsg->position[i];
        vel_map[JointMsg->name[i]] = JointMsg->velocity[i];
    }

    robot.Est.Pos.Wheel.left  = pos_map["left_wheel_joint"];
    robot.Est.Pos.Wheel.right = pos_map["right_wheel_joint"];

    robot.Est.Vel.Wheel.left  = vel_map["left_wheel_joint"];
    robot.Est.Vel.Wheel.right = vel_map["right_wheel_joint"];
}

void ysRos::JoyCallBack(const sensor_msgs::msg::Joy::SharedPtr JoyMsg) {
    // 조이스틱 축 입력을 바로 쓰지 않고 속도 명령 단위로 변환한다.
    JOYdx       = JoyMsg->axes[1] * max_dx;
    JOYdyaw     = JoyMsg->axes[3] * max_dyaw;
    JOYpitch    = JoyMsg->axes[4] * max_dx;
    JOYroll     = JoyMsg->axes[0] * max_dx;

    JOYBtnA     = JoyMsg->buttons[0];
    JOYBtnB     = JoyMsg->buttons[1];
    JOYBtnX     = JoyMsg->buttons[2];
    JOYBtnY     = JoyMsg->buttons[3];
}

void ysRos::PublishStates() {
    std_msgs::msg::Float64MultiArray ref_base_msg;
    std_msgs::msg::Float64MultiArray est_base_msg;
    std_msgs::msg::Float64MultiArray ref_wheel_msg;
    std_msgs::msg::Float64MultiArray est_wheel_msg;

    // base_state:
    // [x, y, yaw, vx, vy, vyaw]
    ref_base_msg.data = {
        robot.Ref.Pos.Base.x,
        robot.Ref.Pos.Base.y,
        robot.Ref.Pos.Base.yaw,
        robot.Ref.Vel.Base.x,
        robot.Ref.Vel.Base.y,
        robot.Ref.Vel.Base.yaw
    };

    est_base_msg.data = {
        robot.Est.Pos.Base.x,
        robot.Est.Pos.Base.y,
        robot.Est.Pos.Base.yaw,
        robot.Est.Vel.Base.x,
        robot.Est.Vel.Base.y,
        robot.Est.Vel.Base.yaw
    };

    // wheel_state:
    // [left_pos, right_pos, left_vel, right_vel, left_tau, right_tau]
    ref_wheel_msg.data = {
        robot.Ref.Pos.Wheel.left,
        robot.Ref.Pos.Wheel.right,
        robot.Ref.Vel.Wheel.left,
        robot.Ref.Vel.Wheel.right,
        robot.Ref.Tau.Wheel.left,
        robot.Ref.Tau.Wheel.right
    };

    est_wheel_msg.data = {
        robot.Est.Pos.Wheel.left,
        robot.Est.Pos.Wheel.right,
        robot.Est.Vel.Wheel.left,
        robot.Est.Vel.Wheel.right,
        robot.Est.Tau.Wheel.left,
        robot.Est.Tau.Wheel.right
    };

    ref_base_state_pub->publish(ref_base_msg);
    est_base_state_pub->publish(est_base_msg);
    ref_wheel_state_pub->publish(ref_wheel_msg);
    est_wheel_state_pub->publish(est_wheel_msg);
}
