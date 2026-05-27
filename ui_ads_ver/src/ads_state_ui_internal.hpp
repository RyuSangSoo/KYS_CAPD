// ads_state_ui_internal.hpp
// ads_state_ui 화면을 역할별 cpp로 분리하기 위한 내부 선언 모음입니다.
// ROS 노드 외부에 노출할 필요가 없는 UI/MuJoCo/유틸리티 함수를 여기서 관리합니다.

#ifndef UI_ADS_VER_ADS_STATE_UI_INTERNAL_HPP
#define UI_ADS_VER_ADS_STATE_UI_INTERNAL_HPP

#include "ui_ads_ver/ads_state_ui.hpp"

#include "imgui.h"

#include <array>
#include <string>
#include <vector>

#if UI_ADS_VER_HAVE_MUJOCO
#include <mujoco/mujoco.h>
#endif

struct GLFWwindow;

namespace ui_ads_ver {
namespace internal {

constexpr size_t kPlotCapacity = 180;
constexpr size_t kBaseTrajectoryCapacity = 300;
constexpr float kBasePlotHalfRangeMeters = 5.0f;
constexpr size_t kMjuPhaseStride = 85;
constexpr size_t kSimTipPosOffset = 64;
constexpr size_t kRefTipPosOffset = kSimTipPosOffset + kMjuPhaseStride;
constexpr size_t kTestConfigFieldCount = 5;
constexpr double kUiRateHz = 30.0;
constexpr char kWindowTitle[] = "OpenRobot Motor Monitor";

extern const std::array<std::string, kJointCount> kJointNames;
extern const std::array<std::string, kJointCount> kMotorNames;
extern const std::array<const char*, kJointCount + 1> kTestJointLabels;

struct UiFonts {
    ImFont* body = nullptr;
    ImFont* heading = nullptr;
    ImFont* mono = nullptr;
};

struct PlotBuffer {
    std::vector<float> time_s;
    std::array<std::vector<float>, kJointCount> ref_rad;
    std::array<std::vector<float>, kJointCount> sim_rad;
    size_t head = 0;
    bool full = false;

    PlotBuffer();
    void Push(float time_value,
              const std::array<double, kJointCount>& ref_value_rad,
              const std::array<double, kJointCount>& sim_value_rad);
    size_t Size() const;
};

struct BaseTrajectoryBuffer {
    std::vector<ImVec2> ref_points;
    std::vector<ImVec2> est_points;
    size_t head = 0;
    bool full = false;

    BaseTrajectoryBuffer();
    void Push(const BaseStateData& ref_state, const BaseStateData& est_state);
    size_t Size() const;
};

template <size_t ChannelCount>
struct MultiSeriesBuffer {
    std::vector<float> time_s;
    std::array<std::vector<float>, ChannelCount> ref_values;
    std::array<std::vector<float>, ChannelCount> est_values;
    size_t head = 0;
    bool full = false;

    MultiSeriesBuffer();
    void Push(float time_value,
              const std::array<double, ChannelCount>& ref_sample,
              const std::array<double, ChannelCount>& est_sample);
    size_t Size() const;
};

struct MujocoCanvasRequest {
    bool visible = false;
    ImVec2 min {};
    ImVec2 max {};
};

struct MujocoViewerState {
#if UI_ADS_VER_HAVE_MUJOCO
    mjModel* model = nullptr;
    mjData* data = nullptr;
    mjvCamera camera {};
    mjvOption option {};
    mjvScene scene {};
    mjrContext context {};
    std::array<int, kJointCount> qpos_address {-1, -1, -1, -1};
#endif
    std::array<char, 512> model_path {};
    std::string loaded_path;
    std::string status = "MuJoCo viewer is idle";
    std::string error;
    bool loaded = false;
    bool follow_joint_sim = false;
    bool show_geom_frame = false;
};

double ToDegrees(double radians);
double DisplayAngleValue(double radians, bool display_in_deg);
double DisplayAngularVelocityValue(double radians_per_second, bool display_in_deg);
const char* AngleUnitLabel(bool display_in_deg);
const char* AngularVelocityUnitLabel(bool display_in_deg);
std::array<double, kJointCount> ConvertAngleArrayForDisplay(
    const std::array<double, kJointCount>& values_rad,
    bool display_in_deg);
std::array<double, kJointCount> ConvertAngularVelocityArrayForDisplay(
    const std::array<double, kJointCount>& values_rad_s,
    bool display_in_deg);
double FrequencyToPeriodSeconds(double frequency_hz);
const char* ModeName(int mode);
ImU32 JointColor(size_t index);
std::string FindFirstExistingPath(const std::vector<std::string>& candidates);
void CopyToCharBuffer(const std::string& value, std::array<char, 512>& buffer);
std::string DefaultMujocoModelPath();
MujocoViewerState CreateMujocoViewerState();

void ApplyImGuiStyle();
UiFonts LoadFonts();
void DrawSingleJointPlot(const char* label,
                         const char* canvas_id,
                         const PlotBuffer& buffer,
                         size_t joint_index,
                         ImU32 joint_color,
                         bool display_in_deg,
                         ImVec2 size);
void DrawLatestJointBlock(
    const char* title,
    const std::array<double, kJointCount>& ref_values,
    const std::array<double, kJointCount>& sim_values,
    const char* unit,
    const std::array<std::string, kJointCount>& item_names = kJointNames);
void DrawTipPositionBlock(const std::vector<double>& mju_state);
void DrawBaseStateBlock(const UiSnapshot& snapshot);
void DrawWheelStateBlock(const UiSnapshot& snapshot);
void DrawBaseTrajectoryPlot(const char* canvas_id,
                            const BaseTrajectoryBuffer& buffer,
                            const BaseStateData& ref_state,
                            const BaseStateData& est_state,
                            const ScanData& scan,
                            float& half_range_m);
template <size_t ChannelCount>
void DrawMultiChannelTimeSeriesPlot(const char* title,
                                    const char* canvas_id,
                                    const MultiSeriesBuffer<ChannelCount>& buffer,
                                    const std::array<const char*, ChannelCount>& channel_labels,
                                    float height);
void LoadUiTestConfigFromSnapshot(const UiSnapshot& snapshot,
                                  int& ui_joint_index,
                                  float& ui_amplitude_rad,
                                  float& ui_period_s);

void FreeMujocoModel(MujocoViewerState& state);
void ResetMujocoCamera(MujocoViewerState& state);
bool LoadMujocoModel(MujocoViewerState& state, const std::string& model_path);
void HandleMujocoCameraInput(MujocoViewerState& state,
                             bool hovered,
                             const ImVec2& canvas_size);
void RenderMujocoCanvas(MujocoViewerState& state,
                        const UiSnapshot& snapshot,
                        const MujocoCanvasRequest& canvas_request,
                        int framebuffer_height,
                        const ImVec2& framebuffer_scale);

GLFWwindow* CreateMainWindow();

}  // namespace internal
}  // namespace ui_ads_ver

#endif
