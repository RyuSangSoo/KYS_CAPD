#ifndef UI_ADS_VER_ADS_STATE_UI_HPP
#define UI_ADS_VER_ADS_STATE_UI_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace ui_ads_ver {

constexpr size_t kJointCount = 4;

struct JointStateData {
    bool received = false;
    std::array<double, kJointCount> position_rad {};
    std::array<double, kJointCount> velocity_rad_s {};
    std::array<double, kJointCount> effort {};
};

struct TestConfigData {
    bool received = false;
    int joint_index = 0;  // 0..3: motor1..motor4, 4: all
    double amplitude_rad = 0.25;
    double frequency_hz = 0.2;
    int current_mode = 0;
    unsigned long long change_counter = 0;
};

struct BaseStateData {
    bool received = false;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double vyaw = 0.0;
};

struct ScanData {
    bool received = false;
    float angle_min = 0.0f;
    float angle_increment = 0.0f;
    float range_min = 0.0f;
    float range_max = 0.0f;
    std::vector<float> ranges;
};

struct WheelStateData {
    bool received = false;
    double left_pos = 0.0;
    double right_pos = 0.0;
    double left_vel = 0.0;
    double right_vel = 0.0;
    double left_tau = 0.0;
    double right_tau = 0.0;
};

struct UiSnapshot {
    JointStateData sim;
    JointStateData ref;
    JointStateData motor_sim;
    JointStateData motor_ref;
    TestConfigData test_config;
    BaseStateData ref_base;
    BaseStateData est_base;
    WheelStateData ref_wheel;
    WheelStateData est_wheel;
    ScanData scan;
    std::vector<double> mju_state;
    size_t sim_count = 0;
    size_t ref_count = 0;
    size_t motor_sim_count = 0;
    size_t motor_ref_count = 0;
    size_t mju_count = 0;
    size_t test_config_count = 0;
    size_t ref_base_count = 0;
    size_t est_base_count = 0;
    size_t ref_wheel_count = 0;
    size_t est_wheel_count = 0;
    size_t scan_count = 0;
    bool joint_order_warning = false;
    std::string sim_topic = "/joint_states";
    std::string ref_topic = "/joint_states_ref";
    std::string motor_sim_topic = "/motor_states";
    std::string motor_ref_topic = "/motor_states_ref";
    std::string mju_topic = "/ads/mju_state";
    std::string test_config_state_topic = "/ads/test_config_state";
    std::string test_config_cmd_topic = "/ads/test_config_cmd";
    std::string ref_base_topic = "/state/ref/base_state";
    std::string est_base_topic = "/state/est/base_state";
    std::string ref_wheel_topic = "/state/ref/wheel_state";
    std::string est_wheel_topic = "/state/est/wheel_state";
    std::string scan_topic = "/scan";
};

class AdsStateUiNode : public rclcpp::Node {
public:
    AdsStateUiNode();

    UiSnapshot Snapshot() const;
    void PublishTestConfigCommand(int joint_index, double amplitude_rad, double frequency_hz);

private:
    static void FillOrderedJointData(const sensor_msgs::msg::JointState& msg,
                                     JointStateData& data,
                                     bool& joint_order_warning,
                                     const std::array<std::string, kJointCount>& expected_names);

    void OnSimJointState(const sensor_msgs::msg::JointState& msg);
    void OnRefJointState(const sensor_msgs::msg::JointState& msg);
    void OnMotorSimJointState(const sensor_msgs::msg::JointState& msg);
    void OnMotorRefJointState(const sensor_msgs::msg::JointState& msg);
    void OnMjuState(const std_msgs::msg::Float64MultiArray& msg);
    void OnTestConfigState(const std_msgs::msg::Float64MultiArray& msg);
    void OnRefBaseState(const std_msgs::msg::Float64MultiArray& msg);
    void OnEstBaseState(const std_msgs::msg::Float64MultiArray& msg);
    void OnRefWheelState(const std_msgs::msg::Float64MultiArray& msg);
    void OnEstWheelState(const std_msgs::msg::Float64MultiArray& msg);
    void OnScan(const sensor_msgs::msg::LaserScan& msg);
    static bool FillBaseStateData(const std_msgs::msg::Float64MultiArray& msg, BaseStateData& data);
    static bool FillWheelStateData(const std_msgs::msg::Float64MultiArray& msg, WheelStateData& data);

    std::string sim_topic_;
    std::string ref_topic_;
    std::string motor_sim_topic_;
    std::string motor_ref_topic_;
    std::string mju_topic_;
    std::string test_config_state_topic_;
    std::string test_config_cmd_topic_;
    std::string ref_base_topic_;
    std::string est_base_topic_;
    std::string ref_wheel_topic_;
    std::string est_wheel_topic_;
    std::string scan_topic_;
    JointStateData sim_data_;
    JointStateData ref_data_;
    JointStateData motor_sim_data_;
    JointStateData motor_ref_data_;
    TestConfigData test_config_;
    BaseStateData ref_base_data_;
    BaseStateData est_base_data_;
    WheelStateData ref_wheel_data_;
    WheelStateData est_wheel_data_;
    ScanData scan_data_;
    std::vector<double> mju_state_;
    size_t sim_count_ = 0;
    size_t ref_count_ = 0;
    size_t motor_sim_count_ = 0;
    size_t motor_ref_count_ = 0;
    size_t mju_count_ = 0;
    size_t test_config_count_ = 0;
    size_t ref_base_count_ = 0;
    size_t est_base_count_ = 0;
    size_t ref_wheel_count_ = 0;
    size_t est_wheel_count_ = 0;
    size_t scan_count_ = 0;
    bool joint_order_warning_ = false;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sim_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ref_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_sim_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mju_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr test_config_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_base_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr est_base_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_wheel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr est_wheel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr test_config_cmd_pub_;
};

int RunAdsStateUi(int argc, char** argv);

}  // namespace ui_ads_ver

#endif
