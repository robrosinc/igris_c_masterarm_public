/**
 * @file masterarm_gui_client.cpp
 * @brief Source for the masterarm_gui_client example
 */

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <signal.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cfloat>
#include <ctime>
#include <cstdio>
#include <deque>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "igris_c_sdk/channel_factory.hpp"
#include "igris_c_sdk/igris_c_client.hpp"
#include "igris_c_sdk/publisher.hpp"
#include "igris_c_sdk/subscriber.hpp"
#include "rbrs_masterarm.hpp"

using namespace igris_c_sdk;
using namespace igris_c::msg::dds;

namespace {

constexpr float kBaseWindowWidth  = 1600.0f;
constexpr float kBaseWindowHeight = 920.0f;
constexpr size_t kArmJointCount   = 7;
constexpr size_t kHandAxisCount   = 2;
constexpr size_t kHandMotorCountPerSide = 6;
constexpr size_t kMasterArmDof    = 9;
constexpr int kPublishPeriodMs    = 50;
constexpr float kHandInputMaxRad  = 1.4f;
constexpr float kMaxConvergenceTimeSec = 3.0f;
constexpr auto kMasterArmReconnectDelay = std::chrono::milliseconds(2000);
constexpr auto kMasterArmReadTimeout = std::chrono::milliseconds(1000);

const std::array<int, kArmJointCount> kLeftRobotJointIndices  = {15, 16, 17, 18, 19, 20, 21};
const std::array<int, kArmJointCount> kRightRobotJointIndices = {22, 23, 24, 25, 26, 27, 28};

const std::array<const char *, kArmJointCount> kArmJointNames = {
    "Shoulder Pitch", "Shoulder Roll", "Shoulder Yaw", "Elbow Pitch", "Wrist Yaw", "Wrist Front", "Wrist Back",
};

const std::array<const char *, kHandMotorCountPerSide> kRightHandMotorNames = {
    "R_Thumb", "R_Index", "R_Middle", "R_Ring", "R_Pinky", "R_Spread",
};
const std::array<const char *, kHandMotorCountPerSide> kLeftHandMotorNames = {
    "L_Thumb", "L_Index", "L_Middle", "L_Ring", "L_Pinky", "L_Spread",
};
const std::array<uint16_t, 12> kHandMotorIds                 = {11, 12, 13, 14, 15, 16, 21, 22, 23, 24, 25, 26};
const std::array<float, kHandAxisCount> kRightHandAxisDir    = {-1.0f, 1.0f};
const std::array<float, kHandAxisCount> kLeftHandAxisDir     = {1.0f, -1.0f};

const std::array<float, 31> kAxisDir = {
    0,  0,  0,                  // Waist
    0,  0,  0,  0,  0,  0,      // Left leg
    0,  0,  0,  0,  0,  0,      // Right leg
    1,  1,  -1, 1,  -1, -1, 1,  // Left arm
    -1, 1,  -1, -1, -1, -1, 1,  // Right arm
    -1, -1,                     // Neck
};

const std::array<std::array<float, 2>, 31> kMotorLimits = {{
    {0, 0},        {0, 0},        {0, 0},        {0, 0},        {0, 0},        {0, 0},        {0, 0},        {0, 0},
    {0, 0},        {0, 0},        {0, 0},        {0, 0},        {0, 0},        {0, 0},        {0, 0},        {-3.141f, 1.047f},
    {-0.170f, 3.140f}, {-1.570f, 1.570f}, {-2.0f, 0.0f}, {-1.570f, 1.570f}, {-0.98f, 0.75f}, {-0.98f, 0.75f},
    {-3.141f, 1.047f}, {-3.140f, 0.170f}, {-1.570f, 1.570f}, {-2.0f, 0.0f}, {-1.570f, 1.570f}, {-0.75f, 0.98f},
    {-0.75f, 0.98f}, {0, 0}, {0, 0},
}};

const std::array<std::array<float, 2>, 31> kKpKd = {{
    {200.0f, 15.0f}, {200.0f, 15.0f}, {200.0f, 15.0f}, {500.0f, 3.0f},  {200.0f, 0.5f},  {50.0f, 0.5f},  {500.0f, 3.0f},
    {300.0f, 1.5f},  {300.0f, 1.5f},  {500.0f, 3.0f},  {200.0f, 0.5f},  {50.0f, 0.5f},  {500.0f, 3.0f},  {300.0f, 1.5f},
    {300.0f, 1.5f},  {75.0f, 0.5f},   {75.0f, 0.5f},   {45.0f, 0.15f}, {45.0f, 0.15f}, {20.0f, 0.5f},  {20.0f, 0.5f},
    {20.0f, 0.5f},   {75.0f, 0.5f},   {75.0f, 0.5f},   {45.0f, 0.15f}, {45.0f, 0.15f}, {20.0f, 0.5f},  {20.0f, 0.5f},
    {20.0f, 0.5f},   {2.0f, 0.05f},   {5.0f, 0.1f},
}};

std::atomic<bool> g_running(true);
std::atomic<bool> g_lowcmd_publish_active(false);
std::atomic<bool> g_handcmd_publish_active(false);
std::atomic<uint32_t> g_lowstate_received_count(0);
std::atomic<uint32_t> g_robotstate_received_count(0);
std::atomic<uint32_t> g_masterarm_read_count(0);
std::atomic<uint32_t> g_lowcmd_publish_count(0);
std::atomic<uint32_t> g_handcmd_publish_count(0);
std::atomic<uint32_t> g_handstate_received_count(0);
std::atomic<bool> g_masterarm_connected(false);
std::atomic<bool> g_robotstate_connected(false);
std::atomic<bool> g_hand_initializing(false);

LowState g_latest_lowstate;
RobotState g_latest_robotstate;
HandState g_latest_handstate;
LowCmd g_last_lowcmd;
HandCmd g_last_handcmd;
std::chrono::steady_clock::time_point g_last_robotstate_time;
std::chrono::steady_clock::time_point g_last_masterarm_read_time;

std::mutex g_lowstate_mutex;
std::mutex g_handstate_mutex;
std::mutex g_robotstate_mutex;
std::mutex g_masterarm_mutex;
std::mutex g_last_cmd_mutex;
std::mutex g_log_mutex;

std::array<float, 31> g_motor_state_q = {};
std::array<float, 31> g_joint_state_q = {};
std::array<float, kMasterArmDof> g_left_masterarm_raw = {};
std::array<float, kMasterArmDof> g_right_masterarm_raw = {};
std::array<float, kArmJointCount> g_left_cmd_arm = {};
std::array<float, kArmJointCount> g_right_cmd_arm = {};
std::array<float, kArmJointCount> g_left_pub_arm = {};
std::array<float, kArmJointCount> g_right_pub_arm = {};
std::array<float, kHandAxisCount> g_left_hand_cmd = {};
std::array<float, kHandAxisCount> g_right_hand_cmd = {};
std::array<float, kArmJointCount> g_left_kp = {
    kKpKd[15][0], kKpKd[16][0], kKpKd[17][0], kKpKd[18][0], kKpKd[19][0], kKpKd[20][0], kKpKd[21][0],
};
std::array<float, kArmJointCount> g_left_kd = {
    kKpKd[15][1], kKpKd[16][1], kKpKd[17][1], kKpKd[18][1], kKpKd[19][1], kKpKd[20][1], kKpKd[21][1],
};
std::array<float, kArmJointCount> g_right_kp = {
    kKpKd[22][0], kKpKd[23][0], kKpKd[24][0], kKpKd[25][0], kKpKd[26][0], kKpKd[27][0], kKpKd[28][0],
};
std::array<float, kArmJointCount> g_right_kd = {
    kKpKd[22][1], kKpKd[23][1], kKpKd[24][1], kKpKd[25][1], kKpKd[26][1], kKpKd[27][1], kKpKd[28][1],
};
std::array<float, kArmJointCount> g_left_kp_edit = g_left_kp;
std::array<float, kArmJointCount> g_left_kd_edit = g_left_kd;
std::array<float, kArmJointCount> g_right_kp_edit = g_right_kp;
std::array<float, kArmJointCount> g_right_kd_edit = g_right_kd;
std::atomic<float> g_convergence_time(1.5f);

std::deque<std::string> g_response_log;
constexpr size_t kMaxLogLines = 60;
bool g_show_service_confirm_popup = false;
std::string g_pending_service_label;
std::function<void()> g_pending_service_action;
bool g_show_gain_confirm_popup = false;
bool g_pending_gain_is_left    = true;
int g_pending_gain_joint       = -1;
bool g_pending_gain_is_kp      = true;
float g_pending_gain_old_value = 0.0f;
float g_pending_gain_new_value = 0.0f;

void SignalHandler(int) { g_running = false; }

void AddLog(const std::string &msg) {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    auto now    = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm     = *std::localtime(&time_t);
    char time_str[32];
    std::strftime(time_str, sizeof(time_str), "%H:%M:%S", &tm);
    g_response_log.push_back(std::string("[") + time_str + "] " + msg);
    if (g_response_log.size() > kMaxLogLines) {
        g_response_log.pop_front();
    }
}

float ClampUiScale(float scale) { return std::clamp(scale, 1.0f, 2.3f); }

float ComputeUiScale(float width, float height) {
    return ClampUiScale(std::min(width / kBaseWindowWidth, height / kBaseWindowHeight));
}

void SectionHeader(const char *text) {
    ImGui::SetWindowFontScale(1.2f);
    ImGui::TextUnformatted(text);
    ImGui::SetWindowFontScale(1.0f);
    ImGui::Separator();
}

void SubsectionHeader(const char *text) {
    ImGui::SetWindowFontScale(1.08f);
    ImGui::TextColored(ImVec4(0.6f, 0.85f, 1.0f, 1.0f), "%s", text);
    ImGui::SetWindowFontScale(1.0f);
}

bool ScaledButton(const char *label, const ImVec2 &size) {
    ImGui::SetWindowFontScale(1.1f);
    const bool clicked = ImGui::Button(label, size);
    ImGui::SetWindowFontScale(1.0f);
    return clicked;
}

void PushSkyTableStyle() {
    ImGui::PushStyleColor(ImGuiCol_TableHeaderBg, ImVec4(0.16f, 0.27f, 0.36f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_TableRowBg, ImVec4(0.05f, 0.09f, 0.13f, 0.55f));
    ImGui::PushStyleColor(ImGuiCol_TableRowBgAlt, ImVec4(0.08f, 0.14f, 0.20f, 0.55f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.35f, 0.55f, 0.72f, 0.60f));
}

void PopSkyTableStyle() { ImGui::PopStyleColor(4); }

void RequestServiceConfirm(const char *label, std::function<void()> action) {
    g_pending_service_label      = label;
    g_pending_service_action     = std::move(action);
    g_show_service_confirm_popup = true;
}

const char *RobotControlStateToString(RobotControlState state) {
    switch (state) {
    case RobotControlState::ROBOT_STATE_NOT_READY:
        return "NOT_READY";
    case RobotControlState::ROBOT_STATE_IDLE:
        return "IDLE";
    case RobotControlState::ROBOT_STATE_STOP:
        return "STOP";
    case RobotControlState::ROBOT_STATE_LOW:
        return "LOW";
    case RobotControlState::ROBOT_STATE_HIGH_MOTION_ACTIVE:
        return "HIGH_MOTION_ACTIVE";
    case RobotControlState::ROBOT_STATE_HIGH_MOTION_STANDBY:
        return "HIGH_MOTION_STANDBY";
    case RobotControlState::ROBOT_STATE_WALK_LOW:
        return "WALK_LOW";
    case RobotControlState::ROBOT_STATE_WALK_HIGH_MOTION_ACTIVE:
        return "WALK_HIGH_MOTION_ACTIVE";
    case RobotControlState::ROBOT_STATE_WALK_HIGH_MOTION_STANDBY:
        return "WALK_HIGH_MOTION_STANDBY";
    case RobotControlState::ROBOT_STATE_WHOLE_BODY_CONTROL:
        return "WHOLE_BODY_CONTROL";
    default:
        return "UNKNOWN";
    }
}

enum class MasterArmConnectionState : int {
    Disconnected = 0,
    Stale = 1,
    Connected = 2,
};

MasterArmConnectionState GetMasterArmConnectionState() {
    if (!g_masterarm_connected.load()) {
        return MasterArmConnectionState::Disconnected;
    }

    const auto now          = std::chrono::steady_clock::now();
    const auto read_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_last_masterarm_read_time);
    if (read_elapsed < kMasterArmReadTimeout) {
        return MasterArmConnectionState::Connected;
    }
    return MasterArmConnectionState::Stale;
}

ImVec4 GetMasterArmConnectionColor(MasterArmConnectionState state) {
    switch (state) {
    case MasterArmConnectionState::Connected:
        return ImVec4(0, 1, 0, 1);
    case MasterArmConnectionState::Stale:
        return ImVec4(1.0f, 0.75f, 0.2f, 1.0f);
    case MasterArmConnectionState::Disconnected:
    default:
        return ImVec4(1, 0, 0, 1);
    }
}

const char *MasterArmConnectionStateToString(MasterArmConnectionState state) {
    switch (state) {
    case MasterArmConnectionState::Connected:
        return "Connected";
    case MasterArmConnectionState::Stale:
        return "Stale";
    case MasterArmConnectionState::Disconnected:
    default:
        return "Disconnected";
    }
}

float MasterArmRawToRobotCmd(bool is_left, size_t arm_idx, float raw_value) {
    const int joint_index = is_left ? kLeftRobotJointIndices[arm_idx] : kRightRobotJointIndices[arm_idx];
    const float mapped    = raw_value * kAxisDir[static_cast<size_t>(joint_index)];
    return std::clamp(mapped, kMotorLimits[static_cast<size_t>(joint_index)][0], kMotorLimits[static_cast<size_t>(joint_index)][1]);
}

void UpdateMappedCommands() {
    for (size_t i = 0; i < kArmJointCount; ++i) {
        g_left_cmd_arm[i]  = MasterArmRawToRobotCmd(true, i, g_left_masterarm_raw[i]);
        g_right_cmd_arm[i] = MasterArmRawToRobotCmd(false, i, g_right_masterarm_raw[i]);
    }
}

float MasterArmRawToHandCmd(bool is_left, size_t axis_idx, float raw_value) {
    const auto &axis_dir = is_left ? kLeftHandAxisDir : kRightHandAxisDir;
    return std::clamp((axis_dir[axis_idx] * raw_value) / kHandInputMaxRad, 0.0f, 1.0f);
}

float ComputeConvergenceAlpha() {
    const float duration = g_convergence_time.load();
    if (duration <= 0.01f) {
        return 1.0f;
    }
    constexpr float kPublishHz = 1000.0f / static_cast<float>(kPublishPeriodMs);
    return 1.0f - std::pow(0.05f, 1.0f / (duration * kPublishHz));
}

void LowStateCallback(const LowState &state) {
    std::lock_guard<std::mutex> lock(g_lowstate_mutex);
    g_latest_lowstate = state;
    for (size_t i = 0; i < g_motor_state_q.size(); ++i) {
        g_motor_state_q[i] = state.motor_state()[i].q();
        g_joint_state_q[i] = state.joint_state()[i].q();
    }
    g_lowstate_received_count++;
}

void HandStateCallback(const HandState &state) {
    std::lock_guard<std::mutex> lock(g_handstate_mutex);
    g_latest_handstate = state;
    g_handstate_received_count++;
}

void RobotStateCallback(const RobotState &state) {
    std::lock_guard<std::mutex> lock(g_robotstate_mutex);
    g_latest_robotstate = state;
    g_robotstate_received_count++;
    g_last_robotstate_time = std::chrono::steady_clock::now();
    g_robotstate_connected = true;
}

void PrintUsage(const char *argv0) {
    std::cerr << "\nUsage:\n"
              << "  " << argv0 << " <port> <baud> [domain_id] [namespace]\n"
              << "  " << argv0 << " /dev/ttyUSB0 1000000 0 robot1\n"
              << "  " << argv0 << " /dev/ttyUSB1 2000000 10\n"
              << "Notes:\n"
              << "  - 'port' must be a valid serial device path.\n"
              << "  - 'baud' must be a positive integer.\n"
              << "  - 'domain_id' defaults to 0.\n"
              << "  - 'namespace' is optional. Leave it empty to use no DDS namespace.\n"
              << std::endl;
}

void CallInitBmsAsync(const std::shared_ptr<IgrisC_Client> &client, BmsInitType type, const char *label) {
    AddLog(std::string("Calling InitBms(") + label + ")...");
    std::thread([client, type, label]() {
        const auto res = client->InitBms(type, 60000);
        AddLog(std::string("InitBms(") + label + "): " + (res.success() ? "SUCCESS" : "FAILED") + " - " + res.message());
    }).detach();
}

void CallSetTorqueAsync(const std::shared_ptr<IgrisC_Client> &client, TorqueType type, const char *label) {
    AddLog(std::string("Calling SetTorque(") + label + ")...");
    std::thread([client, type, label]() {
        const auto res = client->SetTorque(type, 60000);
        AddLog(std::string("SetTorque(") + label + "): " + (res.success() ? "SUCCESS" : "FAILED") + " - " + res.message());
    }).detach();
}

void CallSendControlModeCommandAsync(const std::shared_ptr<IgrisC_Client> &client, ControlModeCommandType type, const char *label) {
    AddLog(std::string("Calling ControlModeCommand(") + label + ")...");
    std::thread([client, type, label]() {
        const auto res = client->SendControlModeCommand(type, "", false, 60000);
        AddLog(std::string("ControlModeCommand(") + label + "): " + (res.success() ? "SUCCESS" : "FAILED") + " - " + res.message());
    }).detach();
}

void CallSendMujocoSimCmdAsync(const std::shared_ptr<IgrisC_Client> &client, MujocoSimCommandType type, const char *label) {
    AddLog(std::string("Calling MujocoSim(") + label + ")...");
    std::thread([client, type, label]() {
        const auto res = client->SendMujocoSimCmd(type, 5000);
        AddLog(std::string("MujocoSim(") + label + "): " + (res.success() ? "SUCCESS" : "FAILED") + " - " + res.message());
    }).detach();
}

void CallInitHandWithPublishStopAsync(const std::shared_ptr<IgrisC_Client> &client) {
    if (g_hand_initializing.load()) {
        AddLog("Hand init already in progress");
        return;
    }

    std::thread([client]() {
        g_handcmd_publish_active = false;
        g_hand_initializing      = true;
        AddLog("Stopped hand publishing for init...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        const auto res = client->InitHand(60000);
        AddLog(std::string("InitHand(): ") + (res.success() ? "SUCCESS" : "FAILED") + " - " + res.message());
        AddLog("Hand publishing remains stopped until restarted");
        g_hand_initializing = false;
    }).detach();
}

void MasterArmReadThread(const std::string port, int baud) {
    while (g_running) {
        try {
            RBRSMasterArm sdk(port, baud, 0);
            sdk.open();

            g_masterarm_connected = true;
            g_last_masterarm_read_time = std::chrono::steady_clock::now();
            AddLog("MasterArm connected");

            while (g_running) {
                sdk.read();
                const auto positions = sdk.get_positions();
                const auto ids       = sdk.get_all_ids();

                {
                    std::lock_guard<std::mutex> lock(g_masterarm_mutex);
                    for (size_t i = 0; i < positions.size() && i < ids.size(); ++i) {
                        const uint8_t id = ids[i];
                        if (id >= 11 && id <= 17) {
                            g_right_masterarm_raw[static_cast<size_t>(id - 11)] = positions[i];
                        } else if (id == 18) {
                            g_right_masterarm_raw[7] = positions[i];
                        } else if (id == 19) {
                            g_right_masterarm_raw[8] = positions[i];
                        } else if (id >= 21 && id <= 27) {
                            g_left_masterarm_raw[static_cast<size_t>(id - 21)] = positions[i];
                        } else if (id == 28) {
                            g_left_masterarm_raw[7] = positions[i];
                        } else if (id == 29) {
                            g_left_masterarm_raw[8] = positions[i];
                        }
                    }
                    UpdateMappedCommands();
                }

                g_last_masterarm_read_time = std::chrono::steady_clock::now();
                g_masterarm_read_count++;
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }

            sdk.close();
        } catch (const std::exception &e) {
            g_masterarm_connected = false;
            AddLog(std::string("MasterArm error: ") + e.what());
        } catch (...) {
            g_masterarm_connected = false;
            AddLog("MasterArm error: unknown exception");
        }

        if (!g_running) {
            break;
        }

        AddLog("Retrying MasterArm connection in 2s...");
        std::this_thread::sleep_for(kMasterArmReconnectDelay);
    }
}

void DrawHandCompareTable(const char *label, bool is_left) {
    std::array<float, kHandMotorCountPerSide> hand_cmd_values = {};
    std::array<float, kHandMotorCountPerSide> hand_state_values = {};

    {
        std::lock_guard<std::mutex> lock(g_masterarm_mutex);
        const auto &hand_axis_values = is_left ? g_left_hand_cmd : g_right_hand_cmd;
        for (size_t i = 0; i < 5; ++i) {
            hand_cmd_values[i] = hand_axis_values[1];
        }
        hand_cmd_values[5] = hand_axis_values[0];
    }
    {
        std::lock_guard<std::mutex> lock(g_handstate_mutex);
        const auto &motor_states = g_latest_handstate.motor_state();
        if (motor_states.size() >= kHandMotorIds.size()) {
            const size_t motor_start = is_left ? 6 : 0;
            for (size_t i = 0; i < kHandMotorCountPerSide; ++i) {
                hand_state_values[i] = motor_states[motor_start + i].q();
            }
        }
    }

    const auto &hand_names = is_left ? kLeftHandMotorNames : kRightHandMotorNames;
    SubsectionHeader(label);
    PushSkyTableStyle();
    if (ImGui::BeginTable((std::string(label) + "_hand").c_str(), 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Hand Motor");
        ImGui::TableSetupColumn("HandCmd");
        ImGui::TableSetupColumn("HandState");
        ImGui::TableSetupColumn("Diff");
        ImGui::TableHeadersRow();

        for (size_t i = 0; i < kHandMotorCountPerSide; ++i) {
            const float diff = hand_cmd_values[i] - hand_state_values[i];
            const bool highlight = std::abs(diff) > 0.1f;
            ImGui::TableNextRow();
            if (highlight) {
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, ImGui::GetColorU32(ImVec4(0.45f, 0.08f, 0.08f, 0.35f)));
            }
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(hand_names[i]);
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.3f", hand_cmd_values[i]);
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%.3f", hand_state_values[i]);
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%+.3f", diff);
        }

        ImGui::EndTable();
    }
    PopSkyTableStyle();
}

void PublishThread(Publisher<LowCmd> *lowcmd_pub, Publisher<HandCmd> *handcmd_pub) {
    while (g_running) {
        LowCmd lowcmd;
        HandCmd handcmd;
        lowcmd.kinematic_modes().fill(KinematicMode::MS);

        for (int i = 0; i < 31; ++i) {
            auto &motor_cmd = lowcmd.motors()[i];
            motor_cmd.id(i);
            motor_cmd.q(0.0f);
            motor_cmd.dq(0.0f);
            motor_cmd.tau(0.0f);
            motor_cmd.kp(0.0f);
            motor_cmd.kd(0.0f);
        }

        handcmd.motor_cmd().resize(kHandMotorIds.size());
        for (size_t i = 0; i < kHandMotorIds.size(); ++i) {
            auto &motor_cmd = handcmd.motor_cmd()[i];
            motor_cmd.id(kHandMotorIds[i]);
            motor_cmd.q(0.0f);
            motor_cmd.dq(0.0f);
            motor_cmd.tau(0.0f);
            motor_cmd.kp(0.0f);
            motor_cmd.kd(0.0f);
        }

        {
            std::lock_guard<std::mutex> lock(g_masterarm_mutex);
            const float alpha = ComputeConvergenceAlpha();

            for (size_t i = 0; i < kArmJointCount; ++i) {
                const int left_joint_index  = kLeftRobotJointIndices[i];
                const int right_joint_index = kRightRobotJointIndices[i];

                g_left_pub_arm[i] += (g_left_cmd_arm[i] - g_left_pub_arm[i]) * alpha;
                g_right_pub_arm[i] += (g_right_cmd_arm[i] - g_right_pub_arm[i]) * alpha;

                auto &left_cmd = lowcmd.motors()[left_joint_index];
                left_cmd.q(g_left_pub_arm[i]);
                left_cmd.kp(g_left_kp[i]);
                left_cmd.kd(g_left_kd[i]);

                auto &right_cmd = lowcmd.motors()[right_joint_index];
                right_cmd.q(g_right_pub_arm[i]);
                right_cmd.kp(g_right_kp[i]);
                right_cmd.kd(g_right_kd[i]);
            }

            const float right_thumb_target = MasterArmRawToHandCmd(false, 0, g_right_masterarm_raw[7]);
            const float right_grip_target  = MasterArmRawToHandCmd(false, 1, g_right_masterarm_raw[8]);
            const float left_thumb_target  = MasterArmRawToHandCmd(true, 0, g_left_masterarm_raw[7]);
            const float left_grip_target   = MasterArmRawToHandCmd(true, 1, g_left_masterarm_raw[8]);

            g_right_hand_cmd[0] += (right_thumb_target - g_right_hand_cmd[0]) * alpha;
            g_right_hand_cmd[1] += (right_grip_target - g_right_hand_cmd[1]) * alpha;
            g_left_hand_cmd[0] += (left_thumb_target - g_left_hand_cmd[0]) * alpha;
            g_left_hand_cmd[1] += (left_grip_target - g_left_hand_cmd[1]) * alpha;

            for (int i = 0; i < 5; ++i) {
                handcmd.motor_cmd()[static_cast<size_t>(i)].q(g_right_hand_cmd[1]);
                handcmd.motor_cmd()[static_cast<size_t>(i + 6)].q(g_left_hand_cmd[1]);
            }
            handcmd.motor_cmd()[5].q(g_right_hand_cmd[0]);
            handcmd.motor_cmd()[11].q(g_left_hand_cmd[0]);
        }

        if (g_lowcmd_publish_active.load()) {
            lowcmd_pub->write(lowcmd);
            g_lowcmd_publish_count++;
        }
        if (g_handcmd_publish_active.load()) {
            handcmd_pub->write(handcmd);
            g_handcmd_publish_count++;
        }

        {
            std::lock_guard<std::mutex> lock(g_last_cmd_mutex);
            g_last_lowcmd  = lowcmd;
            g_last_handcmd = handcmd;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kPublishPeriodMs));
    }
}

void DrawArmCompareTable(const char *label, bool is_left) {
    const auto &robot_indices = is_left ? kLeftRobotJointIndices : kRightRobotJointIndices;
    std::array<float, kArmJointCount> cmd_values = {};
    std::array<float, 31> motor_state_q          = {};
    auto &kp_values = is_left ? g_left_kp : g_right_kp;
    auto &kd_values = is_left ? g_left_kd : g_right_kd;
    auto &kp_edit_values = is_left ? g_left_kp_edit : g_right_kp_edit;
    auto &kd_edit_values = is_left ? g_left_kd_edit : g_right_kd_edit;

    {
        std::lock_guard<std::mutex> lock(g_masterarm_mutex);
        cmd_values = is_left ? g_left_pub_arm : g_right_pub_arm;
    }
    {
        std::lock_guard<std::mutex> lock(g_lowstate_mutex);
        motor_state_q = g_motor_state_q;
    }

    SubsectionHeader(label);
    PushSkyTableStyle();
    const float kp_width = 70.0f;
    const float kd_width = 60.0f;
    if (ImGui::BeginTable(label, 6, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Joint");
        ImGui::TableSetupColumn("LowCmd");
        ImGui::TableSetupColumn("LowState");
        ImGui::TableSetupColumn("Diff");
        ImGui::TableSetupColumn("Kp", ImGuiTableColumnFlags_WidthFixed, kp_width);
        ImGui::TableSetupColumn("Kd", ImGuiTableColumnFlags_WidthFixed, kd_width);
        ImGui::TableHeadersRow();

        for (size_t i = 0; i < kArmJointCount; ++i) {
            const int joint_index = robot_indices[i];
            const float cmd_q     = cmd_values[i];
            const float state_q   = motor_state_q[static_cast<size_t>(joint_index)];
            const float diff      = cmd_q - state_q;
            const bool highlight  = std::abs(diff) > 0.1f;

            ImGui::TableNextRow();
            if (highlight) {
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, ImGui::GetColorU32(ImVec4(0.45f, 0.08f, 0.08f, 0.35f)));
            }

            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(kArmJointNames[i]);
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.3f", cmd_q);
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%.3f", state_q);
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%+.3f", diff);
            ImGui::TableSetColumnIndex(4);
            ImGui::SetNextItemWidth(kp_width - 8.0f);
            char kp_id[64];
            std::snprintf(kp_id, sizeof(kp_id), "##kp_%s_%zu", is_left ? "L" : "R", i);
            const bool kp_enter_commit =
                ImGui::InputFloat(kp_id, &kp_edit_values[i], 0.0f, 0.0f, "%.2f", ImGuiInputTextFlags_EnterReturnsTrue);
            const bool kp_focus_out_commit = ImGui::IsItemDeactivatedAfterEdit();
            if ((kp_enter_commit || kp_focus_out_commit) && kp_edit_values[i] != kp_values[i] && !ImGui::IsPopupOpen("Confirm Gain Change")) {
                g_pending_gain_is_left    = is_left;
                g_pending_gain_joint      = static_cast<int>(i);
                g_pending_gain_is_kp      = true;
                g_pending_gain_old_value  = kp_values[i];
                g_pending_gain_new_value  = std::max(0.0f, kp_edit_values[i]);
                g_show_gain_confirm_popup = true;
            } else if (!ImGui::IsItemActive()) {
                kp_edit_values[i] = kp_values[i];
            }
            ImGui::TableSetColumnIndex(5);
            ImGui::SetNextItemWidth(kd_width - 8.0f);
            char kd_id[64];
            std::snprintf(kd_id, sizeof(kd_id), "##kd_%s_%zu", is_left ? "L" : "R", i);
            const bool kd_enter_commit =
                ImGui::InputFloat(kd_id, &kd_edit_values[i], 0.0f, 0.0f, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue);
            const bool kd_focus_out_commit = ImGui::IsItemDeactivatedAfterEdit();
            if ((kd_enter_commit || kd_focus_out_commit) && kd_edit_values[i] != kd_values[i] && !ImGui::IsPopupOpen("Confirm Gain Change")) {
                g_pending_gain_is_left    = is_left;
                g_pending_gain_joint      = static_cast<int>(i);
                g_pending_gain_is_kp      = false;
                g_pending_gain_old_value  = kd_values[i];
                g_pending_gain_new_value  = std::max(0.0f, kd_edit_values[i]);
                g_show_gain_confirm_popup = true;
            } else if (!ImGui::IsItemActive()) {
                kd_edit_values[i] = kd_values[i];
            }
        }

        ImGui::EndTable();
    }
    PopSkyTableStyle();
}

}  // namespace

int main(int argc, char **argv) {
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    if (argc < 3) {
        PrintUsage(argv[0]);
        return 1;
    }

    const std::string port = argv[1];
    const int baud         = std::stoi(argv[2]);
    const int domain_id    = argc >= 4 ? std::stoi(argv[3]) : 0;
    const std::string robot_namespace = argc >= 5 ? argv[4] : "";

    if (port.empty() || baud <= 0 || domain_id < 0) {
        std::cerr << "Invalid startup arguments."
                  << " port='" << port << "', baud=" << baud << ", domain_id=" << domain_id << std::endl;
        PrintUsage(argv[0]);
        return 1;
    }

    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return 1;
    }

    const char *glsl_version = "#version 330";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow *window =
        glfwCreateWindow(static_cast<int>(kBaseWindowWidth), static_cast<int>(kBaseWindowHeight), "IGRIS-C MasterArm GUI Client", nullptr, nullptr);
    if (window == nullptr) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    io.IniFilename = nullptr;
    ImGui::StyleColorsDark();
    ImGuiStyle base_style  = ImGui::GetStyle();
    float applied_ui_scale = 0.0f;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    ChannelFactory::Instance()->Init(domain_id, robot_namespace);
    if (!ChannelFactory::Instance()->IsInitialized()) {
        std::cerr << "Failed to initialize ChannelFactory" << std::endl;
        return 1;
    }

    auto client = std::make_shared<IgrisC_Client>();
    client->Init();
    client->SetTimeout(5.0f);

    Subscriber<LowState> lowstate_sub("rt/lowstate");
    if (!lowstate_sub.init(LowStateCallback)) {
        std::cerr << "Failed to initialize LowState subscriber" << std::endl;
        return 1;
    }

    Subscriber<RobotState> robotstate_sub("rt/robotstate");
    if (!robotstate_sub.init(RobotStateCallback)) {
        AddLog("Failed to subscribe to RobotState");
    } else {
        AddLog("RobotState subscription started");
    }

    Subscriber<HandState> handstate_sub("rt/handstate");
    if (!handstate_sub.init(HandStateCallback)) {
        AddLog("Failed to subscribe to HandState");
    } else {
        AddLog("HandState subscription started");
    }

    Publisher<LowCmd> lowcmd_pub("rt/lowcmd");
    Publisher<HandCmd> handcmd_pub("rt/handcmd");
    if (!lowcmd_pub.init() || !handcmd_pub.init()) {
        std::cerr << "Failed to initialize publishers" << std::endl;
        return 1;
    }

    std::thread masterarm_thread(MasterArmReadThread, port, baud);
    std::thread publish_thread(PublishThread, &lowcmd_pub, &handcmd_pub);

    AddLog("GUI initialized");

    while (!glfwWindowShouldClose(window) && g_running.load()) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        const float ui_scale = ComputeUiScale(io.DisplaySize.x, io.DisplaySize.y);
        if (std::abs(ui_scale - applied_ui_scale) > 0.001f) {
            ImGui::GetStyle() = base_style;
            ImGui::GetStyle().ScaleAllSizes(ui_scale);
            io.FontGlobalScale = ui_scale;
            applied_ui_scale   = ui_scale;
        }

        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(io.DisplaySize);
        ImGui::Begin("IGRIS-C MasterArm GUI Client", nullptr,
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                         ImGuiWindowFlags_NoBringToFrontOnFocus);

        const float right_width      = io.DisplaySize.x * (280.0f / kBaseWindowWidth);
        const float center_width     = 760.0f * ui_scale;
        const float left_width       = -(center_width + right_width + 2.0f * ImGui::GetStyle().ItemSpacing.x);
        const float log_panel_height = 180.0f * ui_scale;
        const float top_height       = -(log_panel_height + ImGui::GetStyle().ItemSpacing.y);
        const float button_height    = 38.0f * ui_scale;
        const float large_button_height = 46.0f * ui_scale;
        const float popup_button_width = 180.0f * ui_scale;

        ImGui::BeginChild("LeftPanel", ImVec2(left_width, top_height), true);
        {
            SectionHeader("MasterArm vs LowState");
            DrawArmCompareTable("Left Arm", true);
            ImGui::Spacing();
            DrawHandCompareTable("Left Hand", true);
            ImGui::Spacing();
            DrawArmCompareTable("Right Arm", false);
            ImGui::Spacing();
            DrawHandCompareTable("Right Hand", false);
        }
        ImGui::EndChild();

        ImGui::SameLine();

        ImGui::BeginChild("CenterPanel", ImVec2(center_width, top_height), true);
        {
            ImGui::BeginChild("DdsConnInfo", ImVec2(0.0f, 0), ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_Border);
            {
                SectionHeader("DDS Connection Status");

                auto *factory        = ChannelFactory::Instance();
                const std::string ns = factory->GetNamespace();
                ImGui::PushTextWrapPos(ImGui::GetContentRegionAvail().x);
                ImGui::Text("Domain:    %d", factory->GetDomainId());
                ImGui::Text("Namespace: %s", ns.empty() ? "(none)" : ns.c_str());
                ImGui::PopTextWrapPos();
                ImGui::Separator();

                const auto now     = std::chrono::steady_clock::now();
                const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_last_robotstate_time).count();
                const bool is_connected = g_robotstate_connected.load() && (elapsed < 2000);

                bool is_sim_env = false;
                if (is_connected) {
                    ImGui::TextColored(ImVec4(0, 1, 0, 1), "Status: Connected");
                    {
                        std::lock_guard<std::mutex> lock(g_robotstate_mutex);
                        ImGui::Text("Robot State: %s", RobotControlStateToString(g_latest_robotstate.state()));
                        is_sim_env = (g_latest_robotstate.environment() == RobotEnvironment::ROBOT_ENV_SIM);
                        ImGui::TextColored(is_sim_env ? ImVec4(0.5f, 0.8f, 1.0f, 1.0f) : ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "Environment: %s",
                                           is_sim_env ? "SIM" : "REAL");
                    }
                } else {
                    ImGui::TextColored(ImVec4(1, 0, 0, 1), "Status: Disconnected");
                    ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1), "Robot State: --");
                    ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1), "Environment: --");
                }

                if (is_connected && is_sim_env) {
                    ImGui::Separator();
                    SubsectionHeader("Mujoco Sim");
                    const float spacing      = ImGui::GetStyle().ItemSpacing.x;
                    const float button_width = (ImGui::GetContentRegionAvail().x - spacing) / 2.0f;
                    if (ScaledButton("Pause", ImVec2(button_width, button_height))) {
                        CallSendMujocoSimCmdAsync(client, MujocoSimCommandType::MUJOCO_SIM_PAUSE, "PAUSE");
                    }
                    ImGui::SameLine();
                    if (ScaledButton("Resume", ImVec2(button_width, button_height))) {
                        CallSendMujocoSimCmdAsync(client, MujocoSimCommandType::MUJOCO_SIM_RESUME, "RESUME");
                    }
                    if (ScaledButton("Reload Model", ImVec2(button_width, button_height))) {
                        CallSendMujocoSimCmdAsync(client, MujocoSimCommandType::MUJOCO_SIM_RELOAD, "RELOAD");
                    }
                    ImGui::SameLine();
                    if (ScaledButton("Reset State", ImVec2(button_width, button_height))) {
                        CallSendMujocoSimCmdAsync(client, MujocoSimCommandType::MUJOCO_SIM_RESET, "RESET");
                    }
                }
            }
            ImGui::EndChild();

            ImGui::Spacing();

            ImGui::BeginChild("ImuStateBox", ImVec2(0.0f, 0), ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_Border);
            {
                SectionHeader("IMU State");

                if (g_lowstate_received_count.load() > 0) {
                    std::lock_guard<std::mutex> lock(g_lowstate_mutex);
                    const auto &imu          = g_latest_lowstate.imu_state();
                    const ImVec4 label_color = ImVec4(0.5f, 1.0f, 0.5f, 1.0f);

                    ImGui::TextColored(label_color, "Quaternion:");
                    ImGui::Text("  w: %8.4f   x: %8.4f   y: %8.4f   z: %8.4f", imu.quaternion()[0], imu.quaternion()[1], imu.quaternion()[2],
                                imu.quaternion()[3]);

                    ImGui::TextColored(label_color, "Gyroscope (rad/s):");
                    ImGui::Text("  x: %8.4f   y: %8.4f   z: %8.4f", imu.gyroscope()[0], imu.gyroscope()[1], imu.gyroscope()[2]);

                    ImGui::TextColored(label_color, "Accelerometer (m/s^2):");
                    ImGui::Text("  x: %8.4f   y: %8.4f   z: %8.4f", imu.accelerometer()[0], imu.accelerometer()[1], imu.accelerometer()[2]);

                    ImGui::TextColored(label_color, "Roll-Pitch-Yaw (rad):");
                    ImGui::Text("  R: %8.4f   P: %8.4f   Y: %8.4f", imu.rpy()[0], imu.rpy()[1], imu.rpy()[2]);
                } else {
                    ImGui::TextColored(ImVec4(1, 1, 0, 1), "Waiting for IMU data...");
                }
            }
            ImGui::EndChild();

            ImGui::Spacing();

            ImGui::BeginChild("MasterArmStateBox", ImVec2(0.0f, 0), ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_Border);
            {
                SectionHeader("MasterArm State");
                const auto masterarm_state = GetMasterArmConnectionState();
                ImGui::TextColored(GetMasterArmConnectionColor(masterarm_state), "MasterArm: %s",
                                   MasterArmConnectionStateToString(masterarm_state));
                if (masterarm_state == MasterArmConnectionState::Stale) {
                    ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f), "Read timeout. Waiting for thread recovery...");
                }
                if (g_hand_initializing.load()) {
                    ImGui::TextColored(ImVec4(1, 1, 0, 1), "Hand init in progress");
                }
                ImGui::Separator();

                const float small_button_width = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
                if (!g_lowcmd_publish_active.load()) {
                    if (ScaledButton("Start LowCmd", ImVec2(small_button_width, button_height))) {
                        g_lowcmd_publish_active = true;
                        AddLog("LowCmd publishing started");
                    }
                } else {
                    if (ScaledButton("Stop LowCmd", ImVec2(small_button_width, button_height))) {
                        g_lowcmd_publish_active = false;
                        AddLog("LowCmd publishing stopped");
                    }
                }
                ImGui::SameLine();
                if (!g_handcmd_publish_active.load()) {
                    if (ScaledButton("Start HandCmd", ImVec2(small_button_width, button_height))) {
                        g_handcmd_publish_active = true;
                        AddLog("HandCmd publishing started");
                    }
                } else {
                    if (ScaledButton("Stop HandCmd", ImVec2(small_button_width, button_height))) {
                        g_handcmd_publish_active = false;
                        AddLog("HandCmd publishing stopped");
                    }
                }
                ImGui::Spacing();
                if (!(g_lowcmd_publish_active.load() && g_handcmd_publish_active.load())) {
                    if (ScaledButton("Start Both", ImVec2(-1, large_button_height))) {
                        g_lowcmd_publish_active = true;
                        if (!g_hand_initializing.load()) {
                            g_handcmd_publish_active = true;
                        }
                        AddLog(g_hand_initializing.load() ? "Started LowCmd publishing only (Hand init in progress)" : "Started both publishing");
                    }
                } else {
                    if (ScaledButton("Stop Both", ImVec2(-1, large_button_height))) {
                        g_lowcmd_publish_active  = false;
                        g_handcmd_publish_active = false;
                        AddLog("Stopped both publishing");
                    }
                }
                ImGui::Spacing();
                ImGui::TextColored(g_lowcmd_publish_active.load() ? ImVec4(0, 1, 0, 1) : ImVec4(1, 0, 0, 1), "LowCmd Published: %s",
                                   g_lowcmd_publish_active.load() ? "ON" : "OFF");
                ImGui::TextColored(g_handcmd_publish_active.load() ? ImVec4(0, 1, 0, 1) : ImVec4(1, 0, 0, 1), "HandCmd Published: %s",
                                   g_handcmd_publish_active.load() ? "ON" : "OFF");
                ImGui::Separator();
                float convergence_time = g_convergence_time.load();
                const std::string duration_text = "Duration: 9.9s";
                const float duration_text_width = ImGui::CalcTextSize(duration_text.c_str()).x;
                const float item_spacing = ImGui::GetStyle().ItemSpacing.x;
                const float slider_width = std::max(80.0f, ImGui::GetContentRegionAvail().x - duration_text_width - item_spacing);
                ImGui::SetNextItemWidth(slider_width);
                if (ImGui::SliderFloat("##convergence", &convergence_time, 0.0f, kMaxConvergenceTimeSec, "")) {
                    g_convergence_time = convergence_time;
                }
                ImGui::SameLine();
                ImGui::Text("Duration: %.1fs", g_convergence_time.load());
            }
            ImGui::EndChild();
        }
        ImGui::EndChild();

        ImGui::SameLine();

        ImGui::BeginChild("RightPanel", ImVec2(right_width, top_height), true);
        {
            SectionHeader("Service API Commands");

            SubsectionHeader("Power & Torque");
            if (ScaledButton("Init BMS", ImVec2(-1, button_height))) {
                RequestServiceConfirm("Init BMS", [client]() { CallInitBmsAsync(client, BmsInitType::BMS_INIT, "BMS_INIT"); });
            }
            if (ScaledButton("Init Motor", ImVec2(-1, button_height))) {
                RequestServiceConfirm("Init Motor", [client]() { CallInitBmsAsync(client, BmsInitType::MOTOR_INIT, "MOTOR_INIT"); });
            }
            if (ScaledButton("Init BMS + Motor", ImVec2(-1, button_height))) {
                RequestServiceConfirm("Init BMS + Motor",
                                      [client]() { CallInitBmsAsync(client, BmsInitType::BMS_AND_MOTOR_INIT, "BMS_AND_MOTOR_INIT"); });
            }
            if (ScaledButton("Torque ON", ImVec2(-1, button_height))) {
                RequestServiceConfirm("Torque ON", [client]() { CallSetTorqueAsync(client, TorqueType::TORQUE_ON, "TORQUE_ON"); });
            }
            if (ScaledButton("Torque OFF", ImVec2(-1, button_height))) {
                RequestServiceConfirm("Torque OFF", [client]() { CallSetTorqueAsync(client, TorqueType::TORQUE_OFF, "TORQUE_OFF"); });
            }

            SubsectionHeader("Control Mode");
            if (ScaledButton("High-Level", ImVec2(-1, button_height))) {
                RequestServiceConfirm("Control Mode: High-Level", [client]() {
                    CallSendControlModeCommandAsync(client, ControlModeCommandType::CONTROL_MODE_CMD_HIGH_LEVEL_JOINT_CONTROL,
                                                    "HIGH_LEVEL_JOINT_CONTROL");
                });
            }
            if (ScaledButton("Low-Level", ImVec2(-1, button_height))) {
                RequestServiceConfirm("Control Mode: Low-Level", [client]() {
                    CallSendControlModeCommandAsync(client, ControlModeCommandType::CONTROL_MODE_CMD_LOW_LEVEL_JOINT_CONTROL,
                                                    "LOW_LEVEL_JOINT_CONTROL");
                });
            }
            if (ScaledButton("Joint Position Hold", ImVec2(-1, button_height))) {
                RequestServiceConfirm("Joint Position Hold", [client]() {
                    CallSendControlModeCommandAsync(client, ControlModeCommandType::CONTROL_MODE_CMD_JOINT_POSITION_HOLD,
                                                    "JOINT_POSITION_HOLD");
                });
            }
            if (ScaledButton("Motion Stop", ImVec2(-1, button_height))) {
                RequestServiceConfirm("Motion Stop", [client]() {
                    CallSendControlModeCommandAsync(client, ControlModeCommandType::CONTROL_MODE_CMD_MOTION_STOP, "MOTION_STOP");
                });
            }
            if (ScaledButton("Walkmode", ImVec2(-1, button_height))) {
                RequestServiceConfirm("Walkmode", [client]() {
                    CallSendControlModeCommandAsync(client, ControlModeCommandType::CONTROL_MODE_CMD_WALKMODE_ON, "WALKMODE_ON");
                });
            }
            if (ScaledButton("Low-Level Walkmode", ImVec2(-1, button_height))) {
                RequestServiceConfirm("Low-Level Walkmode", [client]() {
                    CallSendControlModeCommandAsync(client, ControlModeCommandType::CONTROL_MODE_CMD_LOW_LEVEL_WALKMODE_ON,
                                                    "LOW_LEVEL_WALKMODE_ON");
                });
            }
            if (ScaledButton("High-Level Walkmode", ImVec2(-1, button_height))) {
                RequestServiceConfirm("High-Level Walkmode", [client]() {
                    CallSendControlModeCommandAsync(client, ControlModeCommandType::CONTROL_MODE_CMD_HIGH_LEVEL_WALKMODE_ON,
                                                    "HIGH_LEVEL_WALKMODE_ON");
                });
            }

            ImGui::Spacing();
            SubsectionHeader("Hand Init");
            if (g_hand_initializing.load()) {
                ImGui::BeginDisabled();
            }
            if (ScaledButton("Initialize Hand (Calibrate)", ImVec2(-1, button_height))) {
                CallInitHandWithPublishStopAsync(client);
            }
            if (g_hand_initializing.load()) {
                ImGui::EndDisabled();
            }
        }
        ImGui::EndChild();

        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + ImGui::GetStyle().ItemSpacing.y);
        ImGui::BeginChild("LogPanel", ImVec2(0, 0), true);
        {
            SectionHeader("Response Log");

            ImGui::BeginChild("LogScroll", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);
            {
                std::lock_guard<std::mutex> lock(g_log_mutex);
                for (const auto &line : g_response_log) {
                    ImGui::TextUnformatted(line.c_str());
                }
            }
            if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) {
                ImGui::SetScrollHereY(1.0f);
            }
            ImGui::EndChild();
        }
        ImGui::EndChild();

        if (g_show_service_confirm_popup) {
            ImGui::OpenPopup("Confirm Service Command");
            g_show_service_confirm_popup = false;
        }

        ImVec2 svc_center = ImGui::GetMainViewport()->GetCenter();
        ImGui::SetNextWindowPos(svc_center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
        if (ImGui::BeginPopupModal("Confirm Service Command", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextUnformatted("Execute the following command?");
            ImGui::Spacing();
            ImGui::SetWindowFontScale(1.3f);
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.4f, 1.0f), "%s", g_pending_service_label.c_str());
            ImGui::SetWindowFontScale(1.0f);
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.5f, 1.0f), "This affects the live robot.");
            ImGui::Separator();

            const bool popup_just_opened = ImGui::IsWindowAppearing();
            if (popup_just_opened) {
                ImGui::SetKeyboardFocusHere();
            }
            const bool enter_pressed =
                !popup_just_opened && (ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter));
            const bool escape_pressed = ImGui::IsKeyPressed(ImGuiKey_Escape);

            if (ScaledButton("Confirm", ImVec2(popup_button_width, button_height)) || enter_pressed) {
                if (g_pending_service_action) {
                    g_pending_service_action();
                }
                g_pending_service_action = nullptr;
                g_pending_service_label.clear();
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ScaledButton("Cancel", ImVec2(popup_button_width, button_height)) || escape_pressed) {
                g_pending_service_action = nullptr;
                g_pending_service_label.clear();
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        if (g_show_gain_confirm_popup) {
            ImGui::OpenPopup("Confirm Gain Change");
            g_show_gain_confirm_popup = false;
        }

        ImVec2 gain_center = ImGui::GetMainViewport()->GetCenter();
        ImGui::SetNextWindowPos(gain_center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
        if (ImGui::BeginPopupModal("Confirm Gain Change", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            const char *gain_name = g_pending_gain_is_kp ? "Kp" : "Kd";
            const char *side_name = g_pending_gain_is_left ? "Left" : "Right";
            const char *joint_name = kArmJointNames[static_cast<size_t>(g_pending_gain_joint)];
            ImGui::Text("Change %s for %s %s?", gain_name, side_name, joint_name);
            ImGui::Spacing();
            ImGui::Text("Old: %.3f", g_pending_gain_old_value);
            ImGui::Text("New: %.3f", g_pending_gain_new_value);
            ImGui::Separator();

            const bool popup_just_opened = ImGui::IsWindowAppearing();
            if (popup_just_opened) {
                ImGui::SetKeyboardFocusHere();
            }
            const bool enter_pressed =
                !popup_just_opened && (ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter));
            const bool escape_pressed = ImGui::IsKeyPressed(ImGuiKey_Escape);

            if (ScaledButton("Confirm", ImVec2(popup_button_width, button_height)) || enter_pressed) {
                auto &kp_values = g_pending_gain_is_left ? g_left_kp : g_right_kp;
                auto &kd_values = g_pending_gain_is_left ? g_left_kd : g_right_kd;
                auto &kp_edit_values = g_pending_gain_is_left ? g_left_kp_edit : g_right_kp_edit;
                auto &kd_edit_values = g_pending_gain_is_left ? g_left_kd_edit : g_right_kd_edit;
                if (g_pending_gain_is_kp) {
                    kp_values[static_cast<size_t>(g_pending_gain_joint)] = g_pending_gain_new_value;
                    kp_edit_values[static_cast<size_t>(g_pending_gain_joint)] = g_pending_gain_new_value;
                } else {
                    kd_values[static_cast<size_t>(g_pending_gain_joint)] = g_pending_gain_new_value;
                    kd_edit_values[static_cast<size_t>(g_pending_gain_joint)] = g_pending_gain_new_value;
                }
                AddLog(std::string("Gain changed: ") + side_name + " " + joint_name + " " + gain_name + " = " +
                       std::to_string(g_pending_gain_new_value));
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ScaledButton("Cancel", ImVec2(popup_button_width, button_height)) || escape_pressed) {
                auto &kp_edit_values = g_pending_gain_is_left ? g_left_kp_edit : g_right_kp_edit;
                auto &kd_edit_values = g_pending_gain_is_left ? g_left_kd_edit : g_right_kd_edit;
                if (g_pending_gain_is_kp) {
                    kp_edit_values[static_cast<size_t>(g_pending_gain_joint)] = g_pending_gain_old_value;
                } else {
                    kd_edit_values[static_cast<size_t>(g_pending_gain_joint)] = g_pending_gain_old_value;
                }
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        ImGui::End();

        ImGui::Render();
        int display_w = 0, display_h = 0;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.08f, 0.08f, 0.09f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    g_running = false;
    g_lowcmd_publish_active = false;
    g_handcmd_publish_active = false;

    if (masterarm_thread.joinable()) {
        masterarm_thread.join();
    }
    if (publish_thread.joinable()) {
        publish_thread.join();
    }

    lowcmd_pub.stop();
    handcmd_pub.stop();
    lowstate_sub.stop();
    robotstate_sub.stop();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
