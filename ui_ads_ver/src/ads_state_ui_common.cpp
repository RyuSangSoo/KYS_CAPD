// ads_state_ui_common.cpp
// 각 화면/노드 cpp에서 공통으로 사용하는 상수, 단위 변환, 버퍼 유틸리티를 모아둔 파일입니다.

#include "ads_state_ui_internal.hpp"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace ui_ads_ver {
namespace internal {

const std::array<std::string, kJointCount> kJointNames = {"joint1", "joint2", "joint3", "joint4"};
const std::array<std::string, kJointCount> kMotorNames = {"motor1", "motor2", "motor3", "motor4"};
const std::array<const char*, kJointCount + 1> kTestJointLabels = {"motor1", "motor2", "motor3", "motor4", "all"};

PlotBuffer::PlotBuffer() : time_s(kPlotCapacity, 0.0f) {
    for (size_t i = 0; i < kJointCount; ++i) {
        ref_rad[i].assign(kPlotCapacity, 0.0f);
        sim_rad[i].assign(kPlotCapacity, 0.0f);
    }
}

void PlotBuffer::Push(const float time_value,
                      const std::array<double, kJointCount>& ref_value_rad,
                      const std::array<double, kJointCount>& sim_value_rad) {
    time_s[head] = time_value;
    for (size_t i = 0; i < kJointCount; ++i) {
        ref_rad[i][head] = static_cast<float>(ref_value_rad[i]);
        sim_rad[i][head] = static_cast<float>(sim_value_rad[i]);
    }
    head = (head + 1) % time_s.size();
    if (head == 0) {
        full = true;
    }
}

size_t PlotBuffer::Size() const {
    return full ? time_s.size() : head;
}

BaseTrajectoryBuffer::BaseTrajectoryBuffer()
    : ref_points(kBaseTrajectoryCapacity, ImVec2(0.0f, 0.0f)),
      est_points(kBaseTrajectoryCapacity, ImVec2(0.0f, 0.0f)) {}

void BaseTrajectoryBuffer::Push(const BaseStateData& ref_state, const BaseStateData& est_state) {
    ref_points[head] = ImVec2(static_cast<float>(ref_state.x), static_cast<float>(ref_state.y));
    est_points[head] = ImVec2(static_cast<float>(est_state.x), static_cast<float>(est_state.y));
    head = (head + 1) % ref_points.size();
    if (head == 0) {
        full = true;
    }
}

size_t BaseTrajectoryBuffer::Size() const {
    return full ? ref_points.size() : head;
}

template <size_t ChannelCount>
MultiSeriesBuffer<ChannelCount>::MultiSeriesBuffer()
    : time_s(kPlotCapacity, 0.0f) {
    for (size_t i = 0; i < ChannelCount; ++i) {
        ref_values[i].assign(kPlotCapacity, 0.0f);
        est_values[i].assign(kPlotCapacity, 0.0f);
    }
}

template <size_t ChannelCount>
void MultiSeriesBuffer<ChannelCount>::Push(const float time_value,
                                           const std::array<double, ChannelCount>& ref_sample,
                                           const std::array<double, ChannelCount>& est_sample) {
    time_s[head] = time_value;
    for (size_t i = 0; i < ChannelCount; ++i) {
        ref_values[i][head] = static_cast<float>(ref_sample[i]);
        est_values[i][head] = static_cast<float>(est_sample[i]);
    }
    head = (head + 1) % time_s.size();
    if (head == 0) {
        full = true;
    }
}

template <size_t ChannelCount>
size_t MultiSeriesBuffer<ChannelCount>::Size() const {
    return full ? time_s.size() : head;
}

double ToDegrees(const double radians) {
    return radians * 180.0 / M_PI;
}

double DisplayAngleValue(const double radians, const bool display_in_deg) {
    return display_in_deg ? ToDegrees(radians) : radians;
}

double DisplayAngularVelocityValue(const double radians_per_second, const bool display_in_deg) {
    return display_in_deg ? ToDegrees(radians_per_second) : radians_per_second;
}

const char* AngleUnitLabel(const bool display_in_deg) {
    return display_in_deg ? "deg" : "rad";
}

const char* AngularVelocityUnitLabel(const bool display_in_deg) {
    return display_in_deg ? "deg/s" : "rad/s";
}

std::array<double, kJointCount> ConvertAngleArrayForDisplay(
    const std::array<double, kJointCount>& values_rad,
    const bool display_in_deg) {
    std::array<double, kJointCount> converted {};
    for (size_t i = 0; i < kJointCount; ++i) {
        converted[i] = DisplayAngleValue(values_rad[i], display_in_deg);
    }
    return converted;
}

std::array<double, kJointCount> ConvertAngularVelocityArrayForDisplay(
    const std::array<double, kJointCount>& values_rad_s,
    const bool display_in_deg) {
    std::array<double, kJointCount> converted {};
    for (size_t i = 0; i < kJointCount; ++i) {
        converted[i] = DisplayAngularVelocityValue(values_rad_s[i], display_in_deg);
    }
    return converted;
}

double FrequencyToPeriodSeconds(const double frequency_hz) {
    return frequency_hz > 1e-9 ? (1.0 / frequency_hz) : 0.0;
}

const char* ModeName(const int mode) {
    switch (mode) {
        case 0: return "초기화";
        case 1: return "준비";
        case 2: return "테스트";
        case 3: return "스탠드";
        case 4: return "워킹";
        case 5: return "수영 준비";
        case 6: return "수영";
        default: return "알 수 없음";
    }
}

ImU32 JointColor(const size_t index) {
    static const ImU32 colors[] = {
        IM_COL32(240, 80, 80, 255),
        IM_COL32(80, 160, 240, 255),
        IM_COL32(250, 180, 60, 255),
        IM_COL32(120, 220, 220, 255),
    };
    return colors[index % (sizeof(colors) / sizeof(colors[0]))];
}

std::string FindFirstExistingPath(const std::vector<std::string>& candidates) {
    for (const auto& path : candidates) {
        if (std::filesystem::exists(path)) {
            return path;
        }
    }
    return {};
}

void CopyToCharBuffer(const std::string& value, std::array<char, 512>& buffer) {
    std::snprintf(buffer.data(), buffer.size(), "%s", value.c_str());
}

std::string DefaultMujocoModelPath() {
    const std::filesystem::path source_file(__FILE__);
    const std::filesystem::path workspace_src = source_file.parent_path().parent_path().parent_path();
    return FindFirstExistingPath({
        (workspace_src / "ui_ads_ver" / "models" / "scene.xml").string(),
        (workspace_src / "ADS" / "model" / "mujoco" / "openrobot_monitor.xml").string(),
        "/home/drcl-rss/.mujoco/mujoco-3.5.0/model/tendon_arm/arm26.xml",
    });
}

MujocoViewerState CreateMujocoViewerState() {
    MujocoViewerState state;
    CopyToCharBuffer(DefaultMujocoModelPath(), state.model_path);
    return state;
}

GLFWwindow* CreateMainWindow() {
    if (!glfwInit()) {
        return nullptr;
    }

    GLFWwindow* window = glfwCreateWindow(1400, 860, kWindowTitle, nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        return nullptr;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(0);
    return window;
}

template struct MultiSeriesBuffer<2>;
template struct MultiSeriesBuffer<3>;

}  // namespace internal
}  // namespace ui_ads_ver
