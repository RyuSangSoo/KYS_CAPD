// ads_state_ui_theme.cpp
// ImGui 스타일, 폰트, 상태 블록, 그래프 위젯처럼 화면 표현에 집중된 코드를 모아둔 파일입니다.

#include "ads_state_ui_internal.hpp"

#include "imgui.h"

#include <algorithm>
#include <cstdio>
#include <string>

namespace ui_ads_ver {
namespace internal {

void ApplyImGuiStyle() {
    ImGuiStyle& style = ImGui::GetStyle();
    ImGui::StyleColorsDark(&style);

    style.WindowRounding = 10.0f;
    style.ChildRounding = 10.0f;
    style.FrameRounding = 7.0f;
    style.GrabRounding = 7.0f;
    style.ScrollbarRounding = 8.0f;
    style.TabRounding = 7.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;
    style.WindowPadding = ImVec2(18.0f, 18.0f);
    style.FramePadding = ImVec2(10.0f, 7.0f);
    style.CellPadding = ImVec2(8.0f, 8.0f);
    style.ItemSpacing = ImVec2(12.0f, 10.0f);
    style.ItemInnerSpacing = ImVec2(8.0f, 6.0f);
    style.IndentSpacing = 18.0f;

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg] = ImVec4(0.06f, 0.08f, 0.11f, 1.00f);
    colors[ImGuiCol_ChildBg] = ImVec4(0.09f, 0.11f, 0.15f, 1.00f);
    colors[ImGuiCol_PopupBg] = ImVec4(0.10f, 0.12f, 0.16f, 0.98f);
    colors[ImGuiCol_Border] = ImVec4(0.24f, 0.29f, 0.36f, 0.70f);
    colors[ImGuiCol_FrameBg] = ImVec4(0.10f, 0.13f, 0.18f, 1.00f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.16f, 0.21f, 0.28f, 1.00f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.18f, 0.23f, 0.31f, 1.00f);
    colors[ImGuiCol_TitleBg] = ImVec4(0.08f, 0.10f, 0.13f, 1.00f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.08f, 0.10f, 0.13f, 1.00f);
    colors[ImGuiCol_Header] = ImVec4(0.15f, 0.20f, 0.28f, 1.00f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.22f, 0.29f, 0.39f, 1.00f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.18f, 0.25f, 0.34f, 1.00f);
    colors[ImGuiCol_Separator] = ImVec4(0.28f, 0.35f, 0.45f, 0.65f);
    colors[ImGuiCol_Text] = ImVec4(0.92f, 0.95f, 0.98f, 1.00f);
    colors[ImGuiCol_TextDisabled] = ImVec4(0.56f, 0.62f, 0.70f, 1.00f);
    colors[ImGuiCol_TableBorderStrong] = ImVec4(0.22f, 0.28f, 0.36f, 1.00f);
    colors[ImGuiCol_TableBorderLight] = ImVec4(0.16f, 0.20f, 0.27f, 1.00f);
    colors[ImGuiCol_TableRowBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    colors[ImGuiCol_TableRowBgAlt] = ImVec4(0.08f, 0.10f, 0.14f, 0.45f);
}

UiFonts LoadFonts() {
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->Clear();

    const std::string regular_font = FindFirstExistingPath({
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    });
    const std::string medium_font = FindFirstExistingPath({
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    });
    const std::string mono_font = FindFirstExistingPath({
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/home/drcl-rss/.local/share/fonts/DejaVu Sans Mono for Powerline.ttf",
    });

    ImFontConfig config;
    config.OversampleH = 3;
    config.OversampleV = 2;
    config.PixelSnapH = false;
    const ImWchar* korean_ranges = io.Fonts->GetGlyphRangesKorean();

    UiFonts fonts;
    if (!regular_font.empty()) {
        fonts.body = io.Fonts->AddFontFromFileTTF(regular_font.c_str(), 20.0f, &config, korean_ranges);
    }
    if (!medium_font.empty()) {
        fonts.heading = io.Fonts->AddFontFromFileTTF(medium_font.c_str(), 26.0f, &config, korean_ranges);
    }
    if (!mono_font.empty()) {
        fonts.mono = io.Fonts->AddFontFromFileTTF(mono_font.c_str(), 18.0f, &config, korean_ranges);
    }

    if (fonts.body == nullptr) {
        fonts.body = io.Fonts->AddFontDefault();
    }
    if (fonts.heading == nullptr) {
        fonts.heading = fonts.body;
    }
    if (fonts.mono == nullptr) {
        fonts.mono = fonts.body;
    }

    io.FontDefault = fonts.body;
    return fonts;
}

void DrawSingleJointPlot(const char* label,
                         const char* canvas_id,
                         const PlotBuffer& buffer,
                         const size_t joint_index,
                         const ImU32 joint_color,
                         const bool display_in_deg,
                         const ImVec2 size) {
    const size_t n = buffer.Size();
    ImGui::TextUnformatted(label);
    ImGui::InvisibleButton(canvas_id, size);
    const ImVec2 p0 = ImGui::GetItemRectMin();
    const ImVec2 p1 = ImGui::GetItemRectMax();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    const float left_margin = 60.0f;
    const float top_margin = 30.0f;
    const float right_margin = 10.0f;
    const float bottom_margin = 14.0f;
    const ImVec2 plot_p0(p0.x + left_margin, p0.y + top_margin);
    const ImVec2 plot_p1(p1.x - right_margin, p1.y - bottom_margin);

    draw_list->AddRectFilled(p0, p1, IM_COL32(22, 22, 26, 255), 4.0f);
    draw_list->AddRect(p0, p1, IM_COL32(80, 80, 90, 255), 4.0f);
    if (n < 2) {
        draw_list->AddText(ImVec2(p0.x + 8.0f, p0.y + 8.0f), IM_COL32(160, 160, 170, 255),
                           "waiting for topic data");
        return;
    }

    if (plot_p1.x <= plot_p0.x || plot_p1.y <= plot_p0.y) {
        return;
    }

    const auto IndexAt = [&](const size_t i) -> size_t {
        if (!buffer.full) {
            return i;
        }
        return (buffer.head + i) % buffer.time_s.size();
    };

    float min_time = buffer.time_s[IndexAt(0)];
    float max_time = buffer.time_s[IndexAt(n - 1)];
    if (max_time <= min_time) {
        max_time = min_time + 1e-3f;
    }

    float min_y = static_cast<float>(DisplayAngleValue(buffer.ref_rad[joint_index][IndexAt(0)], display_in_deg));
    float max_y = min_y;
    for (size_t i = 0; i < n; ++i) {
        const size_t index = IndexAt(i);
        min_y = std::min(min_y, static_cast<float>(DisplayAngleValue(buffer.ref_rad[joint_index][index], display_in_deg)));
        max_y = std::max(max_y, static_cast<float>(DisplayAngleValue(buffer.ref_rad[joint_index][index], display_in_deg)));
        min_y = std::min(min_y, static_cast<float>(DisplayAngleValue(buffer.sim_rad[joint_index][index], display_in_deg)));
        max_y = std::max(max_y, static_cast<float>(DisplayAngleValue(buffer.sim_rad[joint_index][index], display_in_deg)));
    }
    if (max_y <= min_y) {
        min_y -= 1.0f;
        max_y += 1.0f;
    }
    const float padding = 0.1f * (max_y - min_y);
    min_y -= padding;
    max_y += padding;

    const auto MapX = [&](const float ts) {
        return plot_p0.x + (ts - min_time) / (max_time - min_time) * (plot_p1.x - plot_p0.x);
    };
    const auto MapY = [&](const float value) {
        return plot_p1.y - (value - min_y) / (max_y - min_y) * (plot_p1.y - plot_p0.y);
    };

    draw_list->AddRectFilled(plot_p0, plot_p1, IM_COL32(16, 18, 24, 255), 4.0f);
    draw_list->AddRect(plot_p0, plot_p1, IM_COL32(70, 76, 90, 255), 4.0f);

    constexpr int kTickCount = 5;
    for (int tick = 0; tick < kTickCount; ++tick) {
        const float t = static_cast<float>(tick) / static_cast<float>(kTickCount - 1);
        const float y = plot_p0.y + (plot_p1.y - plot_p0.y) * t;
        draw_list->AddLine(ImVec2(plot_p0.x, y), ImVec2(plot_p1.x, y), IM_COL32(50, 50, 58, 255), 1.0f);

        const double tick_value = max_y - (max_y - min_y) * static_cast<double>(t);
        char tick_label[32];
        std::snprintf(tick_label, sizeof(tick_label), "%.1f", tick_value);
        draw_list->AddText(ImVec2(p0.x + 6.0f, y - 8.0f), IM_COL32(170, 178, 190, 255), tick_label);
    }

    for (size_t i = 1; i < n; ++i) {
        const size_t prev = IndexAt(i - 1);
        const size_t curr = IndexAt(i);
        draw_list->AddLine(
            ImVec2(MapX(buffer.time_s[prev]), MapY(static_cast<float>(DisplayAngleValue(buffer.ref_rad[joint_index][prev], display_in_deg)))),
            ImVec2(MapX(buffer.time_s[curr]), MapY(static_cast<float>(DisplayAngleValue(buffer.ref_rad[joint_index][curr], display_in_deg)))),
            IM_COL32(80, 200, 120, 255), 2.0f);
        draw_list->AddLine(
            ImVec2(MapX(buffer.time_s[prev]), MapY(static_cast<float>(DisplayAngleValue(buffer.sim_rad[joint_index][prev], display_in_deg)))),
            ImVec2(MapX(buffer.time_s[curr]), MapY(static_cast<float>(DisplayAngleValue(buffer.sim_rad[joint_index][curr], display_in_deg)))),
            joint_color, 2.0f);
    }

    float legend_x = plot_p0.x;
    draw_list->AddText(ImVec2(legend_x, p0.y + 8.0f), IM_COL32(80, 200, 120, 255), "ref");
    legend_x += 42.0f;
    draw_list->AddText(ImVec2(legend_x, p0.y + 8.0f), joint_color, "actual");
}

void DrawLatestJointBlock(const char* title,
                          const std::array<double, kJointCount>& ref_values,
                          const std::array<double, kJointCount>& sim_values,
                          const char* unit,
                          const std::array<std::string, kJointCount>& item_names) {
    const std::string title_text = title;
    const bool show_error = title_text.find("Position") != std::string::npos;
    ImGui::SeparatorText(title);
    for (size_t i = 0; i < kJointCount; ++i) {
        ImGui::TextColored(ImColor(JointColor(i)), "%s", item_names[i].c_str());
        ImGui::SameLine(90.0f);
        ImGui::Text("ref=%7.2f %s", ref_values[i], unit);
        ImGui::SameLine(250.0f);
        ImGui::Text("act=%7.2f %s", sim_values[i], unit);
        if (show_error) {
            ImGui::SameLine(410.0f);
            ImGui::Text("err=%7.2f %s", ref_values[i] - sim_values[i], unit);
        }
    }
}

void DrawTipPositionBlock(const std::vector<double>& mju_state) {
    if (mju_state.size() < (kRefTipPosOffset + 3)) {
        return;
    }
    ImGui::SeparatorText("Tip Position");
    ImGui::Text("sim: x=% .3f  y=% .3f  z=% .3f",
                mju_state[kSimTipPosOffset + 0],
                mju_state[kSimTipPosOffset + 1],
                mju_state[kSimTipPosOffset + 2]);
    ImGui::Text("ref: x=% .3f  y=% .3f  z=% .3f",
                mju_state[kRefTipPosOffset + 0],
                mju_state[kRefTipPosOffset + 1],
                mju_state[kRefTipPosOffset + 2]);
}

void DrawBaseStateBlock(const UiSnapshot& snapshot) {
    ImGui::SeparatorText("2 Wheel Base State");
    ImGui::Text("received ref/est/scan=%zu/%zu/%zu", snapshot.ref_base_count, snapshot.est_base_count, snapshot.scan_count);
    if (!snapshot.ref_base.received && !snapshot.est_base.received) {
        ImGui::TextDisabled("base_state 수신 대기 중");
        return;
    }

    const ImGuiTableFlags table_flags =
        ImGuiTableFlags_Borders |
        ImGuiTableFlags_RowBg |
        ImGuiTableFlags_SizingStretchProp;
    if (ImGui::BeginTable("base_state_table", 4, table_flags)) {
        ImGui::TableSetupColumn("Field", ImGuiTableColumnFlags_WidthStretch, 0.28f);
        ImGui::TableSetupColumn("Ref", ImGuiTableColumnFlags_WidthStretch, 0.24f);
        ImGui::TableSetupColumn("Est", ImGuiTableColumnFlags_WidthStretch, 0.24f);
        ImGui::TableSetupColumn("Error", ImGuiTableColumnFlags_WidthStretch, 0.24f);
        ImGui::TableHeadersRow();

        const auto DrawRow = [&](const char* label, const double ref_value, const double est_value) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(label);
            ImGui::TableSetColumnIndex(1);
            if (snapshot.ref_base.received) {
                ImGui::Text("%.3f", ref_value);
            } else {
                ImGui::TextDisabled("-");
            }
            ImGui::TableSetColumnIndex(2);
            if (snapshot.est_base.received) {
                ImGui::Text("%.3f", est_value);
            } else {
                ImGui::TextDisabled("-");
            }
            ImGui::TableSetColumnIndex(3);
            if (snapshot.ref_base.received && snapshot.est_base.received) {
                ImGui::Text("%.3f", ref_value - est_value);
            } else {
                ImGui::TextDisabled("-");
            }
        };

        DrawRow("x [m]", snapshot.ref_base.x, snapshot.est_base.x);
        DrawRow("y [m]", snapshot.ref_base.y, snapshot.est_base.y);
        DrawRow("yaw [rad]", snapshot.ref_base.yaw, snapshot.est_base.yaw);
        DrawRow("vx [m/s]", snapshot.ref_base.vx, snapshot.est_base.vx);
        DrawRow("vy [m/s]", snapshot.ref_base.vy, snapshot.est_base.vy);
        DrawRow("wyaw [rad/s]", snapshot.ref_base.vyaw, snapshot.est_base.vyaw);

        ImGui::EndTable();
    }
}

void DrawWheelStateBlock(const UiSnapshot& snapshot) {
    ImGui::SeparatorText("2 Wheel Wheel State");
    if (!snapshot.ref_wheel.received && !snapshot.est_wheel.received) {
        ImGui::TextDisabled("wheel_state 수신 대기 중");
        return;
    }

    const ImGuiTableFlags table_flags =
        ImGuiTableFlags_Borders |
        ImGuiTableFlags_RowBg |
        ImGuiTableFlags_SizingStretchProp;
    if (ImGui::BeginTable("wheel_state_table", 4, table_flags)) {
        ImGui::TableSetupColumn("Field", ImGuiTableColumnFlags_WidthStretch, 0.28f);
        ImGui::TableSetupColumn("Ref", ImGuiTableColumnFlags_WidthStretch, 0.24f);
        ImGui::TableSetupColumn("Est", ImGuiTableColumnFlags_WidthStretch, 0.24f);
        ImGui::TableSetupColumn("Error", ImGuiTableColumnFlags_WidthStretch, 0.24f);
        ImGui::TableHeadersRow();

        const auto DrawRow = [&](const char* label, const double ref_value, const double est_value) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(label);
            ImGui::TableSetColumnIndex(1);
            if (snapshot.ref_wheel.received) {
                ImGui::Text("%.3f", ref_value);
            } else {
                ImGui::TextDisabled("-");
            }
            ImGui::TableSetColumnIndex(2);
            if (snapshot.est_wheel.received) {
                ImGui::Text("%.3f", est_value);
            } else {
                ImGui::TextDisabled("-");
            }
            ImGui::TableSetColumnIndex(3);
            if (snapshot.ref_wheel.received && snapshot.est_wheel.received) {
                ImGui::Text("%.3f", ref_value - est_value);
            } else {
                ImGui::TextDisabled("-");
            }
        };

        DrawRow("left pos [rad]", snapshot.ref_wheel.left_pos, snapshot.est_wheel.left_pos);
        DrawRow("right pos [rad]", snapshot.ref_wheel.right_pos, snapshot.est_wheel.right_pos);
        DrawRow("left vel [rad/s]", snapshot.ref_wheel.left_vel, snapshot.est_wheel.left_vel);
        DrawRow("right vel [rad/s]", snapshot.ref_wheel.right_vel, snapshot.est_wheel.right_vel);

        ImGui::EndTable();
    }
}

template <size_t ChannelCount>
void DrawMultiChannelTimeSeriesPlot(const char* title,
                                    const char* canvas_id,
                                    const MultiSeriesBuffer<ChannelCount>& buffer,
                                    const std::array<const char*, ChannelCount>& channel_labels,
                                    const float height) {
    ImGui::SeparatorText(title);
    ImGui::InvisibleButton(canvas_id, ImVec2(-1.0f, height));
    const ImVec2 p0 = ImGui::GetItemRectMin();
    const ImVec2 p1 = ImGui::GetItemRectMax();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    const float left_margin = 52.0f;
    const float top_margin = 28.0f;
    const float right_margin = 12.0f;
    const float bottom_margin = 18.0f;
    const ImVec2 plot_p0(p0.x + left_margin, p0.y + top_margin);
    const ImVec2 plot_p1(p1.x - right_margin, p1.y - bottom_margin);

    draw_list->AddRectFilled(p0, p1, IM_COL32(16, 18, 24, 255), 8.0f);
    draw_list->AddRect(p0, p1, IM_COL32(84, 98, 120, 255), 8.0f);
    if (plot_p1.x <= plot_p0.x || plot_p1.y <= plot_p0.y) {
        return;
    }

    const size_t n = buffer.Size();
    if (n < 2) {
        draw_list->AddText(ImVec2(p0.x + 12.0f, p0.y + 12.0f), IM_COL32(180, 186, 198, 255),
                           "waiting for time-series data");
        return;
    }

    const auto IndexAt = [&](const size_t i) -> size_t {
        if (!buffer.full) {
            return i;
        }
        return (buffer.head + i) % buffer.time_s.size();
    };

    float min_time = buffer.time_s[IndexAt(0)];
    float max_time = buffer.time_s[IndexAt(n - 1)];
    if (max_time <= min_time) {
        max_time = min_time + 1e-3f;
    }

    float min_y = buffer.ref_values[0][IndexAt(0)];
    float max_y = min_y;
    for (size_t i = 0; i < n; ++i) {
        const size_t index = IndexAt(i);
        for (size_t ch = 0; ch < ChannelCount; ++ch) {
            min_y = std::min(min_y, std::min(buffer.ref_values[ch][index], buffer.est_values[ch][index]));
            max_y = std::max(max_y, std::max(buffer.ref_values[ch][index], buffer.est_values[ch][index]));
        }
    }
    if (max_y <= min_y) {
        min_y -= 1.0f;
        max_y += 1.0f;
    }
    const float padding = 0.12f * (max_y - min_y);
    min_y -= padding;
    max_y += padding;

    const auto MapX = [&](const float ts) {
        return plot_p0.x + (ts - min_time) / (max_time - min_time) * (plot_p1.x - plot_p0.x);
    };
    const auto MapY = [&](const float value) {
        return plot_p1.y - (value - min_y) / (max_y - min_y) * (plot_p1.y - plot_p0.y);
    };

    draw_list->AddRectFilled(plot_p0, plot_p1, IM_COL32(12, 14, 20, 255), 6.0f);
    draw_list->AddRect(plot_p0, plot_p1, IM_COL32(62, 72, 88, 255), 6.0f);

    constexpr int kTickCount = 5;
    for (int tick = 0; tick < kTickCount; ++tick) {
        const float t = static_cast<float>(tick) / static_cast<float>(kTickCount - 1);
        const float y = plot_p0.y + (plot_p1.y - plot_p0.y) * t;
        draw_list->AddLine(ImVec2(plot_p0.x, y), ImVec2(plot_p1.x, y), IM_COL32(50, 56, 68, 255), 1.0f);
    }

    static const ImU32 ref_colors[] = {
        IM_COL32(80, 200, 120, 255),
        IM_COL32(255, 160, 90, 255),
        IM_COL32(210, 110, 240, 255),
    };
    static const ImU32 est_colors[] = {
        IM_COL32(80, 160, 240, 255),
        IM_COL32(255, 100, 100, 255),
        IM_COL32(120, 220, 220, 255),
    };

    for (size_t ch = 0; ch < ChannelCount; ++ch) {
        for (size_t i = 1; i < n; ++i) {
            const size_t prev = IndexAt(i - 1);
            const size_t curr = IndexAt(i);
            draw_list->AddLine(
                ImVec2(MapX(buffer.time_s[prev]), MapY(buffer.ref_values[ch][prev])),
                ImVec2(MapX(buffer.time_s[curr]), MapY(buffer.ref_values[ch][curr])),
                ref_colors[ch % 3], 2.8f);
            draw_list->AddLine(
                ImVec2(MapX(buffer.time_s[prev]), MapY(buffer.est_values[ch][prev])),
                ImVec2(MapX(buffer.time_s[curr]), MapY(buffer.est_values[ch][curr])),
                est_colors[ch % 3], 2.8f);
        }
    }

    float legend_x = p0.x + 12.0f;
    for (size_t ch = 0; ch < ChannelCount; ++ch) {
        draw_list->AddText(ImVec2(legend_x, p0.y + 8.0f), ref_colors[ch % 3], channel_labels[ch]);
        legend_x += 52.0f;
        const std::string est_label = std::string(channel_labels[ch]) + "(est)";
        draw_list->AddText(ImVec2(legend_x, p0.y + 8.0f), est_colors[ch % 3], est_label.c_str());
        legend_x += 92.0f;
    }
}

void DrawBaseTrajectoryPlot(const char* canvas_id,
                            const BaseTrajectoryBuffer& buffer,
                            const BaseStateData& ref_state,
                            const BaseStateData& est_state,
                            const ScanData& scan,
                            float& half_range_m) {
    ImGui::InvisibleButton(canvas_id, ImVec2(-1.0f, -1.0f));
    const ImVec2 p0 = ImGui::GetItemRectMin();
    const ImVec2 p1 = ImGui::GetItemRectMax();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    const ImVec2 plot_p0(p0.x + 20.0f, p0.y + 20.0f);
    const ImVec2 plot_p1(p1.x - 20.0f, p1.y - 20.0f);

    if (ImGui::IsItemHovered()) {
        const float wheel = ImGui::GetIO().MouseWheel;
        if (std::fabs(wheel) > 1e-6f) {
            const float zoom_factor = wheel > 0.0f ? 0.9f : 1.1f;
            half_range_m = std::clamp(half_range_m * zoom_factor, 0.5f, 50.0f);
        }
    }

    draw_list->AddRectFilled(p0, p1, IM_COL32(16, 18, 24, 255), 8.0f);
    draw_list->AddRect(p0, p1, IM_COL32(84, 98, 120, 255), 8.0f);
    if (plot_p1.x <= plot_p0.x || plot_p1.y <= plot_p0.y) {
        return;
    }

    const size_t n = buffer.Size();
    if (n == 0) {
        draw_list->AddText(ImVec2(p0.x + 14.0f, p0.y + 14.0f), IM_COL32(180, 186, 198, 255),
                           "waiting for base_state data");
        return;
    }

    const auto IndexAt = [&](const size_t i) -> size_t {
        if (!buffer.full) {
            return i;
        }
        return (buffer.head + i) % buffer.ref_points.size();
    };

    const float center_x = static_cast<float>(est_state.x);
    const float center_y = static_cast<float>(est_state.y);
    const float min_x = center_x - half_range_m;
    const float max_x = center_x + half_range_m;
    const float min_y = center_y - half_range_m;
    const float max_y = center_y + half_range_m;

    const auto MapX = [&](const float x) {
        return plot_p0.x + (x - min_x) / (max_x - min_x) * (plot_p1.x - plot_p0.x);
    };
    const auto MapY = [&](const float y) {
        return plot_p1.y - (y - min_y) / (max_y - min_y) * (plot_p1.y - plot_p0.y);
    };

    draw_list->AddRectFilled(plot_p0, plot_p1, IM_COL32(12, 14, 20, 255), 6.0f);
    draw_list->AddRect(plot_p0, plot_p1, IM_COL32(62, 72, 88, 255), 6.0f);

    const float center_px = MapX(center_x);
    const float center_py = MapY(center_y);
    draw_list->AddLine(ImVec2(center_px, plot_p0.y), ImVec2(center_px, plot_p1.y), IM_COL32(50, 56, 68, 255), 1.0f);
    draw_list->AddLine(ImVec2(plot_p0.x, center_py), ImVec2(plot_p1.x, center_py), IM_COL32(50, 56, 68, 255), 1.0f);

    for (size_t i = 1; i < n; ++i) {
        const size_t prev = IndexAt(i - 1);
        const size_t curr = IndexAt(i);
        const ImVec2 ref_prev(MapX(buffer.ref_points[prev].x), MapY(buffer.ref_points[prev].y));
        const ImVec2 ref_curr(MapX(buffer.ref_points[curr].x), MapY(buffer.ref_points[curr].y));
        const ImVec2 est_prev(MapX(buffer.est_points[prev].x), MapY(buffer.est_points[prev].y));
        const ImVec2 est_curr(MapX(buffer.est_points[curr].x), MapY(buffer.est_points[curr].y));

        if ((ref_prev.x >= plot_p0.x || ref_curr.x >= plot_p0.x) &&
            (ref_prev.x <= plot_p1.x || ref_curr.x <= plot_p1.x) &&
            (ref_prev.y >= plot_p0.y || ref_curr.y >= plot_p0.y) &&
            (ref_prev.y <= plot_p1.y || ref_curr.y <= plot_p1.y)) {
            draw_list->AddLine(ref_prev, ref_curr, IM_COL32(80, 200, 120, 255), 3.5f);
        }
        if ((est_prev.x >= plot_p0.x || est_curr.x >= plot_p0.x) &&
            (est_prev.x <= plot_p1.x || est_curr.x <= plot_p1.x) &&
            (est_prev.y >= plot_p0.y || est_curr.y >= plot_p0.y) &&
            (est_prev.y <= plot_p1.y || est_curr.y <= plot_p1.y)) {
            draw_list->AddLine(est_prev, est_curr, IM_COL32(80, 160, 240, 255), 3.5f);
        }
    }

    const ImVec2 ref_now(MapX(static_cast<float>(ref_state.x)), MapY(static_cast<float>(ref_state.y)));
    const ImVec2 est_now(MapX(static_cast<float>(est_state.x)), MapY(static_cast<float>(est_state.y)));

    if (scan.received && scan.angle_increment != 0.0f) {
        const size_t scan_stride = std::max<size_t>(1, scan.ranges.size() / 180);
        for (size_t i = 0; i < scan.ranges.size(); i += scan_stride) {
            const float range = scan.ranges[i];
            if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
                continue;
            }

            const float beam_angle = scan.angle_min + static_cast<float>(i) * scan.angle_increment;
            const double world_angle = est_state.yaw + static_cast<double>(beam_angle);
            const float hit_x = static_cast<float>(est_state.x + static_cast<double>(range) * std::cos(world_angle));
            const float hit_y = static_cast<float>(est_state.y + static_cast<double>(range) * std::sin(world_angle));
            const ImVec2 hit(MapX(hit_x), MapY(hit_y));
            if (hit.x < plot_p0.x || hit.x > plot_p1.x || hit.y < plot_p0.y || hit.y > plot_p1.y) {
                continue;
            }
            draw_list->AddRectFilled(
                ImVec2(hit.x - 1.5f, hit.y - 1.5f),
                ImVec2(hit.x + 1.5f, hit.y + 1.5f),
                IM_COL32(220, 220, 110, 200));
        }
    }

    if (ref_now.x >= plot_p0.x && ref_now.x <= plot_p1.x && ref_now.y >= plot_p0.y && ref_now.y <= plot_p1.y) {
        draw_list->AddCircleFilled(ref_now, 7.0f, IM_COL32(255, 80, 80, 255));
    }
    draw_list->AddCircleFilled(est_now, 7.0f, IM_COL32(255, 80, 80, 255));

    const float heading_length_px = 20.0f;
    const ImVec2 heading_tip(
        est_now.x + heading_length_px * std::cos(est_state.yaw),
        est_now.y - heading_length_px * std::sin(est_state.yaw));
    draw_list->AddLine(est_now, heading_tip, IM_COL32(255, 255, 255, 220), 3.0f);

    draw_list->AddText(ImVec2(p0.x + 14.0f, p0.y + 10.0f), IM_COL32(80, 200, 120, 255), "ref");
    draw_list->AddText(ImVec2(p0.x + 54.0f, p0.y + 10.0f), IM_COL32(80, 160, 240, 255), "est");
    draw_list->AddText(ImVec2(p0.x + 94.0f, p0.y + 10.0f), IM_COL32(220, 220, 110, 255), "scan");

    char range_label[96];
    std::snprintf(range_label, sizeof(range_label), "center=(%.2f, %.2f)  zoom=+/-%.1fm", center_x, center_y, half_range_m);
    const ImVec2 label_size = ImGui::CalcTextSize(range_label);
    draw_list->AddText(ImVec2(p1.x - label_size.x - 14.0f, p0.y + 10.0f), IM_COL32(180, 186, 198, 255), range_label);
}

template void DrawMultiChannelTimeSeriesPlot<2>(const char*,
                                                const char*,
                                                const MultiSeriesBuffer<2>&,
                                                const std::array<const char*, 2>&,
                                                float);
template void DrawMultiChannelTimeSeriesPlot<3>(const char*,
                                                const char*,
                                                const MultiSeriesBuffer<3>&,
                                                const std::array<const char*, 3>&,
                                                float);

void LoadUiTestConfigFromSnapshot(const UiSnapshot& snapshot,
                                  int& ui_joint_index,
                                  float& ui_amplitude_rad,
                                  float& ui_period_s) {
    ui_joint_index = std::clamp(snapshot.test_config.joint_index, 0, static_cast<int>(kJointCount));
    ui_amplitude_rad = static_cast<float>(std::max(0.0, snapshot.test_config.amplitude_rad));
    ui_period_s = static_cast<float>(FrequencyToPeriodSeconds(snapshot.test_config.frequency_hz));
}

}  // namespace internal
}  // namespace ui_ads_ver
