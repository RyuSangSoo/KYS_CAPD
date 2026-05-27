// ads_state_ui_mujoco.cpp
// Simulation 탭 안의 MuJoCo 모델 로딩, 카메라 조작, 캔버스 렌더링 로직을 분리한 파일입니다.

#include "ads_state_ui_internal.hpp"

#include "imgui.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

#if UI_ADS_VER_HAVE_MUJOCO
#include <mujoco/mujoco.h>
#endif

namespace ui_ads_ver {
namespace internal {

#if UI_ADS_VER_HAVE_MUJOCO
namespace {

void CacheMujocoJointAddress(MujocoViewerState& state) {
    state.qpos_address.fill(-1);
    if (state.model == nullptr) {
        return;
    }

    std::array<bool, kJointCount> assigned {};
    const std::array<std::array<const char*, 2>, kJointCount> preferred_joint_names {{
        {{"joint1", "LEG1_Joint"}},
        {{"joint2", "LEG2_Joint"}},
        {{"joint3", "LEG3_Joint"}},
        {{"joint4", "LEG4_Joint"}},
    }};
    for (size_t i = 0; i < kJointCount; ++i) {
        for (const char* joint_name : preferred_joint_names[i]) {
            const int joint_id = mj_name2id(state.model, mjOBJ_JOINT, joint_name);
            if (joint_id >= 0) {
                state.qpos_address[i] = state.model->jnt_qposadr[joint_id];
                assigned[i] = true;
                break;
            }
        }
    }

    int fallback_joint = 0;
    for (size_t i = 0; i < kJointCount; ++i) {
        if (assigned[i]) {
            continue;
        }
        while (fallback_joint < state.model->njnt) {
            const mjtJoint joint_type = static_cast<mjtJoint>(state.model->jnt_type[fallback_joint]);
            if (joint_type == mjJNT_HINGE || joint_type == mjJNT_SLIDE) {
                state.qpos_address[i] = state.model->jnt_qposadr[fallback_joint++];
                break;
            }
            ++fallback_joint;
        }
    }
}

void SyncMujocoPoseFromSnapshot(MujocoViewerState& state, const UiSnapshot& snapshot) {
    if (!state.loaded || state.model == nullptr || state.data == nullptr) {
        return;
    }

    const std::array<double, kJointCount>* source_rad = nullptr;
    if (state.follow_joint_sim && snapshot.sim.received) {
        source_rad = &snapshot.sim.position_rad;
    } else if (!state.follow_joint_sim && snapshot.ref.received) {
        source_rad = &snapshot.ref.position_rad;
    } else if (snapshot.sim.received) {
        source_rad = &snapshot.sim.position_rad;
    } else if (snapshot.ref.received) {
        source_rad = &snapshot.ref.position_rad;
    }

    if (source_rad == nullptr) {
        return;
    }

    for (size_t i = 0; i < kJointCount; ++i) {
        const int qpos_address = state.qpos_address[i];
        if (qpos_address < 0 || qpos_address >= state.model->nq) {
            continue;
        }
        state.data->qpos[qpos_address] = (*source_rad)[i];
    }
    mj_forward(state.model, state.data);
}

}  // namespace
#endif

void FreeMujocoModel(MujocoViewerState& state) {
#if UI_ADS_VER_HAVE_MUJOCO
    if (state.loaded) {
        mjv_freeScene(&state.scene);
        mjr_freeContext(&state.context);
    }
    if (state.data != nullptr) {
        mj_deleteData(state.data);
        state.data = nullptr;
    }
    if (state.model != nullptr) {
        mj_deleteModel(state.model);
        state.model = nullptr;
    }
    state.qpos_address.fill(-1);
    state.loaded = false;
#else
    (void)state;
#endif
}

void ResetMujocoCamera(MujocoViewerState& state) {
#if UI_ADS_VER_HAVE_MUJOCO
    if (state.model == nullptr) {
        return;
    }

    mjv_defaultCamera(&state.camera);
    state.camera.type = mjCAMERA_FREE;
    state.camera.lookat[0] = state.model->stat.center[0];
    state.camera.lookat[1] = state.model->stat.center[1];
    state.camera.lookat[2] = state.model->stat.center[2];
    state.camera.distance = std::max(1.4, 2.4 * static_cast<double>(state.model->stat.extent));
    state.camera.azimuth = 122.0;
    state.camera.elevation = -18.0;
#else
    (void)state;
#endif
}

bool LoadMujocoModel(MujocoViewerState& state, const std::string& model_path) {
#if UI_ADS_VER_HAVE_MUJOCO
    if (model_path.empty()) {
        state.loaded = false;
        state.error = "Model path is empty";
        state.status = "MuJoCo load failed";
        return false;
    }

    char error_buffer[1000] = "";
    mjModel* model = mj_loadXML(model_path.c_str(), nullptr, error_buffer, sizeof(error_buffer));
    if (model == nullptr) {
        state.loaded = false;
        state.error = error_buffer;
        state.status = "MuJoCo load failed";
        return false;
    }

    mjData* data = mj_makeData(model);
    if (data == nullptr) {
        mj_deleteModel(model);
        state.loaded = false;
        state.error = "mj_makeData failed";
        state.status = "MuJoCo load failed";
        return false;
    }

    FreeMujocoModel(state);

    state.model = model;
    state.data = data;
    mjv_defaultOption(&state.option);
    mjv_defaultScene(&state.scene);
    mjr_defaultContext(&state.context);
    mjv_makeScene(state.model, &state.scene, 4000);
    mjr_makeContext(state.model, &state.context, mjFONTSCALE_150);
    mjr_setBuffer(mjFB_WINDOW, &state.context);
    CacheMujocoJointAddress(state);
    ResetMujocoCamera(state);

    state.loaded = true;
    state.loaded_path = model_path;
    state.status = "MuJoCo model loaded";
    state.error.clear();
    return true;
#else
    state.loaded = false;
    state.status = "MuJoCo support is not compiled in";
    state.error = "Rebuild ui_ads_ver with MuJoCo installed";
    (void)model_path;
    return false;
#endif
}

void HandleMujocoCameraInput(MujocoViewerState& state,
                             const bool hovered,
                             const ImVec2& canvas_size) {
#if UI_ADS_VER_HAVE_MUJOCO
    if (!state.loaded || state.model == nullptr || !hovered || canvas_size.y <= 1.0f) {
        return;
    }

    ImGuiIO& io = ImGui::GetIO();
    if (std::fabs(io.MouseWheel) > 1e-6f) {
        mjv_moveCamera(state.model, mjMOUSE_ZOOM, 0.0, -0.05 * io.MouseWheel, &state.scene, &state.camera);
    }

    const double dx = static_cast<double>(io.MouseDelta.x);
    const double dy = static_cast<double>(io.MouseDelta.y);
    if (std::abs(dx) < 1e-9 && std::abs(dy) < 1e-9) {
        return;
    }

    mjtMouse action = mjMOUSE_ZOOM;
    if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        action = io.KeyShift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
        action = io.KeyShift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (!ImGui::IsMouseDown(ImGuiMouseButton_Middle)) {
        return;
    }

    mjv_moveCamera(state.model, action, dx / canvas_size.y, dy / canvas_size.y, &state.scene, &state.camera);
#else
    (void)state;
    (void)hovered;
    (void)canvas_size;
#endif
}

void RenderMujocoCanvas(MujocoViewerState& state,
                        const UiSnapshot& snapshot,
                        const MujocoCanvasRequest& canvas_request,
                        const int framebuffer_height,
                        const ImVec2& framebuffer_scale) {
#if UI_ADS_VER_HAVE_MUJOCO
    if (!state.loaded || state.model == nullptr || state.data == nullptr || !canvas_request.visible) {
        return;
    }

    const float scale_x = framebuffer_scale.x > 0.0f ? framebuffer_scale.x : 1.0f;
    const float scale_y = framebuffer_scale.y > 0.0f ? framebuffer_scale.y : 1.0f;
    const int left = static_cast<int>(std::round(canvas_request.min.x * scale_x));
    const int top = static_cast<int>(std::round(canvas_request.min.y * scale_y));
    const int width = std::max(1, static_cast<int>(std::round((canvas_request.max.x - canvas_request.min.x) * scale_x)));
    const int height = std::max(1, static_cast<int>(std::round((canvas_request.max.y - canvas_request.min.y) * scale_y)));
    const int bottom = framebuffer_height - (top + height);
    if (height <= 0 || width <= 0 || bottom < 0) {
        return;
    }

    SyncMujocoPoseFromSnapshot(state, snapshot);
    state.option.frame = state.show_geom_frame ? mjFRAME_GEOM : mjFRAME_NONE;

    const mjrRect viewport {left, bottom, width, height};
    mjr_setBuffer(mjFB_WINDOW, &state.context);
    mjv_updateScene(state.model, state.data, &state.option, nullptr, &state.camera, mjCAT_ALL, &state.scene);
    mjr_render(viewport, &state.scene, &state.context);
#else
    (void)state;
    (void)snapshot;
    (void)canvas_request;
    (void)framebuffer_height;
    (void)framebuffer_scale;
#endif
}

}  // namespace internal
}  // namespace ui_ads_ver
