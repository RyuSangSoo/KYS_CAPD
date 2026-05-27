// ads_state_ui_node.cpp
// ROS 토픽 파라미터 선언, subscriber/publisher 생성, 콜백 처리처럼 데이터 입출력을 담당하는 파일입니다.

#include "ads_state_ui_internal.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace ui_ads_ver {

AdsStateUiNode::AdsStateUiNode()
    : rclcpp::Node("ads_state_ui") {
    sim_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");
    ref_topic_ = declare_parameter<std::string>("joint_state_ref_topic", "/joint_states_ref");
    motor_sim_topic_ = declare_parameter<std::string>("motor_state_topic", "/motor_states");
    motor_ref_topic_ = declare_parameter<std::string>("motor_state_ref_topic", "/motor_states_ref");
    mju_topic_ = declare_parameter<std::string>("mju_state_topic", "/ads/mju_state");
    test_config_state_topic_ = declare_parameter<std::string>("test_config_state_topic", "/ads/test_config_state");
    test_config_cmd_topic_ = declare_parameter<std::string>("test_config_cmd_topic", "/ads/test_config_cmd");
    ref_base_topic_ = declare_parameter<std::string>("ref_base_state_topic", "/state/ref/base_state");
    est_base_topic_ = declare_parameter<std::string>("est_base_state_topic", "/state/est/base_state");
    ref_wheel_topic_ = declare_parameter<std::string>("ref_wheel_state_topic", "/state/ref/wheel_state");
    est_wheel_topic_ = declare_parameter<std::string>("est_wheel_state_topic", "/state/est/wheel_state");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");

    sim_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        sim_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { OnSimJointState(*msg); });
    ref_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        ref_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { OnRefJointState(*msg); });
    motor_sim_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        motor_sim_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { OnMotorSimJointState(*msg); });
    motor_ref_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        motor_ref_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { OnMotorRefJointState(*msg); });
    mju_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        mju_topic_, rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { OnMjuState(*msg); });
    test_config_state_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        test_config_state_topic_, rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { OnTestConfigState(*msg); });
    ref_base_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        ref_base_topic_, rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { OnRefBaseState(*msg); });
    est_base_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        est_base_topic_, rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { OnEstBaseState(*msg); });
    ref_wheel_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        ref_wheel_topic_, rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { OnRefWheelState(*msg); });
    est_wheel_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        est_wheel_topic_, rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { OnEstWheelState(*msg); });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { OnScan(*msg); });
    test_config_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        test_config_cmd_topic_, rclcpp::QoS(10).reliable());

    RCLCPP_INFO(get_logger(),
                "Listening: joint_sim=%s joint_ref=%s motor_sim=%s motor_ref=%s mju=%s test_state=%s test_cmd=%s ref_base=%s est_base=%s ref_wheel=%s est_wheel=%s scan=%s",
                sim_topic_.c_str(), ref_topic_.c_str(), motor_sim_topic_.c_str(),
                motor_ref_topic_.c_str(), mju_topic_.c_str(),
                test_config_state_topic_.c_str(), test_config_cmd_topic_.c_str(),
                ref_base_topic_.c_str(), est_base_topic_.c_str(),
                ref_wheel_topic_.c_str(), est_wheel_topic_.c_str(), scan_topic_.c_str());
}

UiSnapshot AdsStateUiNode::Snapshot() const {
    UiSnapshot snapshot;
    snapshot.sim = sim_data_;
    snapshot.ref = ref_data_;
    snapshot.motor_sim = motor_sim_data_;
    snapshot.motor_ref = motor_ref_data_;
    snapshot.test_config = test_config_;
    snapshot.ref_base = ref_base_data_;
    snapshot.est_base = est_base_data_;
    snapshot.ref_wheel = ref_wheel_data_;
    snapshot.est_wheel = est_wheel_data_;
    snapshot.scan = scan_data_;
    snapshot.mju_state = mju_state_;
    snapshot.sim_count = sim_count_;
    snapshot.ref_count = ref_count_;
    snapshot.motor_sim_count = motor_sim_count_;
    snapshot.motor_ref_count = motor_ref_count_;
    snapshot.mju_count = mju_count_;
    snapshot.test_config_count = test_config_count_;
    snapshot.ref_base_count = ref_base_count_;
    snapshot.est_base_count = est_base_count_;
    snapshot.ref_wheel_count = ref_wheel_count_;
    snapshot.est_wheel_count = est_wheel_count_;
    snapshot.scan_count = scan_count_;
    snapshot.joint_order_warning = joint_order_warning_;
    snapshot.sim_topic = sim_topic_;
    snapshot.ref_topic = ref_topic_;
    snapshot.motor_sim_topic = motor_sim_topic_;
    snapshot.motor_ref_topic = motor_ref_topic_;
    snapshot.mju_topic = mju_topic_;
    snapshot.test_config_state_topic = test_config_state_topic_;
    snapshot.test_config_cmd_topic = test_config_cmd_topic_;
    snapshot.ref_base_topic = ref_base_topic_;
    snapshot.est_base_topic = est_base_topic_;
    snapshot.ref_wheel_topic = ref_wheel_topic_;
    snapshot.est_wheel_topic = est_wheel_topic_;
    snapshot.scan_topic = scan_topic_;
    return snapshot;
}

void AdsStateUiNode::PublishTestConfigCommand(const int joint_index,
                                              const double amplitude_rad,
                                              const double frequency_hz) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
        static_cast<double>(std::clamp(joint_index, 0, static_cast<int>(kJointCount))),
        std::max(0.0, amplitude_rad),
        std::max(0.0, frequency_hz),
    };
    test_config_cmd_pub_->publish(msg);
}

void AdsStateUiNode::FillOrderedJointData(
    const sensor_msgs::msg::JointState& msg,
    JointStateData& data,
    bool& joint_order_warning,
    const std::array<std::string, kJointCount>& expected_names) {
    std::array<int, kJointCount> order {-1, -1, -1, -1};
    bool exact_match = msg.name.size() >= kJointCount;
    for (size_t i = 0; i < kJointCount; ++i) {
        auto it = std::find(msg.name.begin(), msg.name.end(), expected_names[i]);
        if (it == msg.name.end()) {
            exact_match = false;
            continue;
        }
        order[i] = static_cast<int>(std::distance(msg.name.begin(), it));
    }

    if (!exact_match) {
        joint_order_warning = true;
    }

    for (size_t i = 0; i < kJointCount; ++i) {
        if (order[i] < 0) {
            data.position_rad[i] = 0.0;
            data.velocity_rad_s[i] = 0.0;
            data.effort[i] = 0.0;
            continue;
        }

        const size_t index = static_cast<size_t>(order[i]);
        data.position_rad[i] = index < msg.position.size() ? msg.position[index] : 0.0;
        data.velocity_rad_s[i] = index < msg.velocity.size() ? msg.velocity[index] : 0.0;
        data.effort[i] = index < msg.effort.size() ? msg.effort[index] : 0.0;
    }
    data.received = true;
}

void AdsStateUiNode::OnSimJointState(const sensor_msgs::msg::JointState& msg) {
    FillOrderedJointData(msg, sim_data_, joint_order_warning_, internal::kJointNames);
    ++sim_count_;
}

void AdsStateUiNode::OnRefJointState(const sensor_msgs::msg::JointState& msg) {
    FillOrderedJointData(msg, ref_data_, joint_order_warning_, internal::kJointNames);
    ++ref_count_;
}

void AdsStateUiNode::OnMotorSimJointState(const sensor_msgs::msg::JointState& msg) {
    FillOrderedJointData(msg, motor_sim_data_, joint_order_warning_, internal::kMotorNames);
    ++motor_sim_count_;
}

void AdsStateUiNode::OnMotorRefJointState(const sensor_msgs::msg::JointState& msg) {
    FillOrderedJointData(msg, motor_ref_data_, joint_order_warning_, internal::kMotorNames);
    ++motor_ref_count_;
}

void AdsStateUiNode::OnMjuState(const std_msgs::msg::Float64MultiArray& msg) {
    mju_state_ = msg.data;
    ++mju_count_;
}

void AdsStateUiNode::OnTestConfigState(const std_msgs::msg::Float64MultiArray& msg) {
    if (msg.data.size() < internal::kTestConfigFieldCount) {
        return;
    }

    if (!std::isfinite(msg.data[0]) ||
        !std::isfinite(msg.data[1]) ||
        !std::isfinite(msg.data[2]) ||
        !std::isfinite(msg.data[3]) ||
        !std::isfinite(msg.data[4])) {
        return;
    }

    test_config_.joint_index = std::clamp(static_cast<int>(std::lround(msg.data[0])), 0, static_cast<int>(kJointCount));
    test_config_.amplitude_rad = std::max(0.0, msg.data[1]);
    test_config_.frequency_hz = std::max(0.0, msg.data[2]);
    test_config_.current_mode = static_cast<int>(std::lround(msg.data[3]));
    test_config_.change_counter = static_cast<unsigned long long>(std::llround(std::max(0.0, msg.data[4])));
    test_config_.received = true;
    ++test_config_count_;
}

bool AdsStateUiNode::FillBaseStateData(const std_msgs::msg::Float64MultiArray& msg, BaseStateData& data) {
    if (msg.data.size() < 6) {
        return false;
    }

    for (size_t i = 0; i < 6; ++i) {
        if (!std::isfinite(msg.data[i])) {
            return false;
        }
    }

    data.x = msg.data[0];
    data.y = msg.data[1];
    data.yaw = msg.data[2];
    data.vx = msg.data[3];
    data.vy = msg.data[4];
    data.vyaw = msg.data[5];
    data.received = true;
    return true;
}

bool AdsStateUiNode::FillWheelStateData(const std_msgs::msg::Float64MultiArray& msg, WheelStateData& data) {
    if (msg.data.size() < 6) {
        return false;
    }

    for (size_t i = 0; i < 6; ++i) {
        if (!std::isfinite(msg.data[i])) {
            return false;
        }
    }

    data.left_pos = msg.data[0];
    data.right_pos = msg.data[1];
    data.left_vel = msg.data[2];
    data.right_vel = msg.data[3];
    data.left_tau = msg.data[4];
    data.right_tau = msg.data[5];
    data.received = true;
    return true;
}

void AdsStateUiNode::OnRefBaseState(const std_msgs::msg::Float64MultiArray& msg) {
    if (FillBaseStateData(msg, ref_base_data_)) {
        ++ref_base_count_;
    }
}

void AdsStateUiNode::OnEstBaseState(const std_msgs::msg::Float64MultiArray& msg) {
    if (FillBaseStateData(msg, est_base_data_)) {
        ++est_base_count_;
    }
}

void AdsStateUiNode::OnRefWheelState(const std_msgs::msg::Float64MultiArray& msg) {
    if (FillWheelStateData(msg, ref_wheel_data_)) {
        ++ref_wheel_count_;
    }
}

void AdsStateUiNode::OnEstWheelState(const std_msgs::msg::Float64MultiArray& msg) {
    if (FillWheelStateData(msg, est_wheel_data_)) {
        ++est_wheel_count_;
    }
}

void AdsStateUiNode::OnScan(const sensor_msgs::msg::LaserScan& msg) {
    scan_data_.angle_min = msg.angle_min;
    scan_data_.angle_increment = msg.angle_increment;
    scan_data_.range_min = msg.range_min;
    scan_data_.range_max = msg.range_max;
    scan_data_.ranges = msg.ranges;
    scan_data_.received = true;
    ++scan_count_;
}

}  // namespace ui_ads_ver
