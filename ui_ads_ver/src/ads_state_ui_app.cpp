// ads_state_ui_app.cpp
// GLFW 창 생성, 전체 레이아웃, 탭 구성, 렌더 루프처럼 실제 UI 실행 흐름을 담당하는 파일입니다.

#include "ads_state_ui_internal.hpp"

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl2.h"
#include "imgui.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

namespace ui_ads_ver {
namespace {

void RenderLeftInfoPanel(const UiSnapshot& snapshot) {
    internal::DrawBaseStateBlock(snapshot);
    internal::DrawWheelStateBlock(snapshot);
}

void RenderBaseTrajectoryTab(const internal::BaseTrajectoryBuffer& base_trajectory_buffer,
                             const UiSnapshot& snapshot,
                             float& base_plot_half_range_m) {
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.08f, 0.11f, 0.16f, 1.00f));
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8.0f);
    ImGui::BeginChild("base_trajectory_group", ImVec2(0.0f, 0.0f), true);
    ImGui::SeparatorText("Base 2D Trajectory");
    internal::DrawBaseTrajectoryPlot("base_trajectory_canvas",
                                     base_trajectory_buffer,
                                     snapshot.ref_base,
                                     snapshot.est_base,
                                     snapshot.scan,
                                     base_plot_half_range_m);
    ImGui::EndChild();
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
}

void RenderTimeGraphsTab(const internal::MultiSeriesBuffer<3>& base_pos_buffer,
                         const internal::MultiSeriesBuffer<3>& base_vel_buffer,
                         const internal::MultiSeriesBuffer<2>& wheel_pos_buffer,
                         const internal::MultiSeriesBuffer<2>& wheel_vel_buffer) {
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.08f, 0.11f, 0.16f, 1.00f));
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8.0f);
    ImGui::BeginChild("time_graph_group", ImVec2(0.0f, 0.0f), true);
    const float block_h = std::max(130.0f, (ImGui::GetContentRegionAvail().y - 40.0f) / 4.0f);
    internal::DrawMultiChannelTimeSeriesPlot<3>("Base Position", "base_pos_ts", base_pos_buffer, {"x", "y", "yaw"}, block_h);
    internal::DrawMultiChannelTimeSeriesPlot<3>("Base Velocity", "base_vel_ts", base_vel_buffer, {"vx", "vy", "wyaw"}, block_h);
    internal::DrawMultiChannelTimeSeriesPlot<2>("Wheel Position", "wheel_pos_ts", wheel_pos_buffer, {"left", "right"}, block_h);
    internal::DrawMultiChannelTimeSeriesPlot<2>("Wheel Velocity", "wheel_vel_ts", wheel_vel_buffer, {"left", "right"}, block_h);
    ImGui::EndChild();
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
}

}  // namespace

int RunAdsStateUi(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<AdsStateUiNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    GLFWwindow* window = internal::CreateMainWindow();
    if (window == nullptr) {
        throw std::runtime_error("glfw window initialization failed");
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    internal::ApplyImGuiStyle();
    const internal::UiFonts fonts = internal::LoadFonts();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL2_Init();

    internal::BaseTrajectoryBuffer base_trajectory_buffer;
    internal::MultiSeriesBuffer<3> base_pos_buffer;
    internal::MultiSeriesBuffer<3> base_vel_buffer;
    internal::MultiSeriesBuffer<2> wheel_pos_buffer;
    internal::MultiSeriesBuffer<2> wheel_vel_buffer;
    size_t last_plotted_ref_base_count = 0;
    size_t last_plotted_est_base_count = 0;
    size_t last_plotted_ref_wheel_count = 0;
    size_t last_plotted_est_wheel_count = 0;
    float base_plot_half_range_m = internal::kBasePlotHalfRangeMeters;
    const auto start_time = std::chrono::steady_clock::now();
    rclcpp::WallRate ui_rate(internal::kUiRateHz);

    while (!glfwWindowShouldClose(window) && rclcpp::ok()) {
        glfwPollEvents();
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }
        executor.spin_some();

        const UiSnapshot snapshot = node->Snapshot();
        const bool has_new_base_sample = snapshot.ref_base_count != last_plotted_ref_base_count ||
                                         snapshot.est_base_count != last_plotted_est_base_count;
        if (has_new_base_sample && snapshot.ref_base.received && snapshot.est_base.received) {
            const float elapsed_s = static_cast<float>(
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count());
            base_trajectory_buffer.Push(snapshot.ref_base, snapshot.est_base);
            base_pos_buffer.Push(elapsed_s,
                                 {snapshot.ref_base.x, snapshot.ref_base.y, snapshot.ref_base.yaw},
                                 {snapshot.est_base.x, snapshot.est_base.y, snapshot.est_base.yaw});
            base_vel_buffer.Push(elapsed_s,
                                 {snapshot.ref_base.vx, snapshot.ref_base.vy, snapshot.ref_base.vyaw},
                                 {snapshot.est_base.vx, snapshot.est_base.vy, snapshot.est_base.vyaw});
            last_plotted_ref_base_count = snapshot.ref_base_count;
            last_plotted_est_base_count = snapshot.est_base_count;
        }
        const bool has_new_wheel_sample = snapshot.ref_wheel_count != last_plotted_ref_wheel_count ||
                                          snapshot.est_wheel_count != last_plotted_est_wheel_count;
        if (has_new_wheel_sample && snapshot.ref_wheel.received && snapshot.est_wheel.received) {
            const float elapsed_s = static_cast<float>(
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count());
            wheel_pos_buffer.Push(elapsed_s,
                                  {snapshot.ref_wheel.left_pos, snapshot.ref_wheel.right_pos},
                                  {snapshot.est_wheel.left_pos, snapshot.est_wheel.right_pos});
            wheel_vel_buffer.Push(elapsed_s,
                                  {snapshot.ref_wheel.left_vel, snapshot.ref_wheel.right_vel},
                                  {snapshot.est_wheel.left_vel, snapshot.est_wheel.right_vel});
            last_plotted_ref_wheel_count = snapshot.ref_wheel_count;
            last_plotted_est_wheel_count = snapshot.est_wheel_count;
        }

        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize, ImGuiCond_Always);
        const ImGuiWindowFlags window_flags =
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoScrollbar |
            ImGuiWindowFlags_NoScrollWithMouse;

        ImGui::Begin("ADS Joint State Monitor", nullptr, window_flags);
        ImGui::PushFont(fonts.heading);
        ImGui::TextUnformatted(internal::kWindowTitle);
        ImGui::PopFont();
        ImGui::Text("ROS 2 topic subscriber view");
        ImGui::Separator();

        const ImGuiTableFlags table_flags =
            ImGuiTableFlags_Resizable |
            ImGuiTableFlags_BordersInnerV |
            ImGuiTableFlags_SizingStretchProp;

        if (ImGui::BeginTable("main_split_table", 2, table_flags)) {
            ImGui::TableSetupColumn("LeftInfo", ImGuiTableColumnFlags_WidthStretch, 0.38f);
            ImGui::TableSetupColumn("RightPlots", ImGuiTableColumnFlags_WidthStretch, 0.62f);
            ImGui::TableNextRow();

            ImGui::TableSetColumnIndex(0);
            ImGui::BeginChild("left_info_panel", ImVec2(0.0f, 0.0f), false, ImGuiWindowFlags_AlwaysVerticalScrollbar);
            RenderLeftInfoPanel(snapshot);
            ImGui::EndChild();

            ImGui::TableSetColumnIndex(1);
            ImGui::BeginChild("right_plot_panel", ImVec2(0.0f, 0.0f), false, ImGuiWindowFlags_NoScrollbar);
            if (ImGui::BeginTabBar("right_panel_tabs")) {
                if (ImGui::BeginTabItem("Base 2D")) {
                    RenderBaseTrajectoryTab(base_trajectory_buffer, snapshot, base_plot_half_range_m);
                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem("Time Graphs")) {
                    RenderTimeGraphsTab(base_pos_buffer, base_vel_buffer, wheel_pos_buffer, wheel_vel_buffer);
                    ImGui::EndTabItem();
                }

                ImGui::EndTabBar();
            }
            ImGui::EndChild();

            ImGui::EndTable();
        }
        ImGui::End();

        ImGui::Render();
        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window, &width, &height);
        glViewport(0, 0, width, height);
        glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
        ui_rate.sleep();
    }

    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    executor.remove_node(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}

}  // namespace ui_ads_ver
