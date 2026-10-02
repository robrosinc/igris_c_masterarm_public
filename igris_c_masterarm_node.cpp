/**
 * @file igris_c_masterarm_node.cpp
 * @author colson (colson@robros.co.kr)
 * @brief
 * @version 0.1
 * @date 2025-12-22
 *
 * @copyright Copyright (c) 2025
 *
 */

// std
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"

// igris_c_sdk
#include "igris_c_sdk/channel_factory.hpp"
#include "igris_c_sdk/igris_c_client.hpp"
#include "igris_c_sdk/namespace_resolver.hpp"
#include "igris_c_sdk/publisher.hpp"
#include "igris_c_sdk/subscriber.hpp"

// masterarm
#include "rbrs_masterarm.hpp"

using namespace igris_c_sdk;

bool g_running          = true;
static int g_show_motor = 1;  // 1 = motor, 0 = joint

std::vector<float> left_target_joint_positions  = {0, 0, 0, 0, 0, 0, 0, 0, 0};
std::vector<float> rigit_target_joint_positions = {0, 0, 0, 0, 0, 0, 0, 0, 0};

uint8_t g_robot_control_state = igris_c_sdk::msg::dds_::RobotState_Constants::ROBOT_STATE_NOT_READY;

bool isLowLevelControlMode(uint8_t state) {
    return state == igris_c_sdk::msg::dds_::RobotState_Constants::ROBOT_STATE_LOW ||
           state == igris_c_sdk::msg::dds_::RobotState_Constants::ROBOT_STATE_WALK_LOW;
}

void printParameterHints() {
    std::cerr << "\nParameter hints:\n"
              << "  ros2 launch igris_c_masterarm igris_c_masterarm.launch.py "
              << "port:=/dev/ttyUSB0 baud:=1000000 domain_id:=0 namespace:=robot1\n"
              << "  ros2 launch igris_c_masterarm igris_c_masterarm.launch.py "
              << "port:=/dev/ttyUSB1 baud:=2000000 domain_id:=10\n"
              << "Notes:\n"
              << "  - 'port' must be a valid serial device path.\n"
              << "  - 'baud' must be a positive integer.\n"
              << "  - 'domain_id' must be zero or a positive integer.\n"
              << "  - 'namespace' is optional. Leave it empty to use no DDS namespace.\n"
              << "  - 'dds_topic_naming' is 'native' (default) or 'ros_compatible'; it must match\n"
              << "    the robot's igris_c.network.transport (native pairs with cyclonedds_native) or neither side sees the other.\n"
              << std::endl;
}

// Hand motor IDs (matches dxl_hand_controller order)
static const std::array<uint16_t, 12> HAND_MOTOR_IDS = {11, 12, 13, 14, 15, 16, 21, 22, 23, 24, 25, 26};

// Motor names for display
static const std::array<const char *, 12> HAND_MOTOR_NAMES = {
    "R_Thumb",   // ID 11
    "R_Index",   // ID 12
    "R_Middle",  // ID 13
    "R_Ring",    // ID 14
    "R_Pinky",   // ID 15
    "R_Spread",  // ID 16
    "L_Thumb",   // ID 21
    "L_Index",   // ID 22
    "L_Middle",  // ID 23
    "L_Ring",    // ID 24
    "L_Pinky",   // ID 25
    "L_Spread"   // ID 26
};

static const std::vector<std::vector<float>> KpKd = {
    {50.0, 0.8},  {25.0, 0.8},  {25.0, 0.8},                                                         // Waist
    {500.0, 3.0}, {200.0, 0.5}, {50.0, 0.5},  {500.0, 3.0}, {300.0, 1.5}, {300.0, 1.5},              // Left leg
    {500.0, 3.0}, {200.0, 0.5}, {50.0, 0.5},  {500.0, 3.0}, {300.0, 1.5}, {300.0, 1.5},              // Right leg
    {50.0, 0.5},  {50.0, 0.5},  {30.0, 0.15}, {30.0, 0.15}, {5.0, 0.1},   {5.0, 0.1},   {5.0, 0.1},  // Left arm
    {50.0, 0.5},  {50.0, 0.5},  {30.0, 0.15}, {30.0, 0.15}, {5.0, 0.1},   {5.0, 0.1},   {5.0, 0.1},  // Right arm
    {2.0, 0.05},  {5.0, 0.1}                                                                         // Neck
};

static const std::vector<std::vector<float>> softKpKd = {
    {10, 0.05},  {10, 0.05}, {10, 0.05},                                                 // Waist
    {10, 0.05},  {10, 0.05}, {10, 0.05}, {10, 0.05}, {10, 0.05}, {10, 0.05},             // Left leg
    {10, 0.05},  {10, 0.05}, {10, 0.05}, {10, 0.05}, {10, 0.05}, {10, 0.05},             // Right leg
    {10, 0.05},  {10, 0.05}, {10, 0.05}, {10, 0.05}, {5, 0.05},  {5, 0.05},  {5, 0.05},  // Left arm
    {10, 0.05},  {10, 0.05}, {10, 0.05}, {10, 0.05}, {5, 0.05},  {5, 0.05},  {5, 0.05},  // Right arm
    {2.0, 0.05}, {5.0, 0.1}                                                              // Neck
};

static const std::vector<float> axisDir = {
    0,  0,  0,                  // Waist
    0,  0,  0,  0,  0,  0,      // Left leg
    0,  0,  0,  0,  0,  0,      // Right leg
    1,  1,  -1, 1,  -1, -1, 1,  // Left arm
    -1, 1,  -1, -1, -1, -1, 1,  // Right arm
    -1, -1,                     // Neck
};

static const std::vector<std::vector<float>> motorLimit = {
    // clang-format off
    {0, 0}, {0, 0}, {0, 0}, // Waist
    {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, // Left leg
    {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, // Right leg
    {-3.141, 1.047}, {-0.170, 3.140}, {-1.570, 1.570}, {-2.0, 0}, {-1.570, 1.570}, {-0.98, 0.75}, {-0.98, 0.75}, // Left arm
    {-3.141, 1.047}, {-3.140, 0.170}, {-1.570, 1.570}, {-2.0, 0}, {-1.570, 1.570}, {-0.75, 0.98}, {-0.75, 0.98}, // Right arm
    {0, 0}, {0, 0}, // Neck
    // clang-format on
};

void lowCmdPublishThread(Publisher<igris_c_sdk::msg::dds_::LowCmd_> *jointPublisher,
                         Publisher<igris_c_sdk::msg::dds_::HandCmd_> *handPublisher) {
    igris_c_sdk::msg::dds_::LowCmd_ cmd;
    igris_c_sdk::msg::dds_::HandCmd_ handCmd;

    bool use_joint_mode = (g_show_motor == 0);  // 0 = joint, 1 = motor
    cmd.kinematic_modes().fill(use_joint_mode ? igris_c_sdk::msg::dds_::LowCmd_Constants::KINEMATIC_MODE_PJS
                                              : igris_c_sdk::msg::dds_::LowCmd_Constants::KINEMATIC_MODE_MS);

    for (int i = 0; i < 31; i++) {
        auto &motor_cmd = cmd.motors()[i];
        motor_cmd.id(i);
        motor_cmd.q(0);
        motor_cmd.dq(0.0f);
        motor_cmd.tau(0.0f);
        motor_cmd.kp(0);
        motor_cmd.kd(0);
    }

    // Waist (indices 0-2), held at 0 with the stiff gains. Nothing else commands it: this node
    // only writes the arms (15-28), and the teleop walk policy's action_joints are the 12 leg
    // joints, so the zeroed kp/kd above survived all the way to the actuators and left the waist
    // free-swinging in walk modes. Held here rather than by the controller because a kp of 0 is
    // a legitimate request elsewhere, and this node is the one that publishes it.
    for (int i = 0; i < 3; i++) {
        auto &waist_cmd = cmd.motors()[i];
        waist_cmd.q(0.0f);
        waist_cmd.kp(KpKd[i][0]);
        waist_cmd.kd(KpKd[i][1]);
    }

    for (int i = 0; i < 12; i++) {
        auto &motor_cmd = handCmd.motor_cmd()[i];
        motor_cmd.id(HAND_MOTOR_IDS[i]);
        motor_cmd.q(0);
        motor_cmd.dq(0.0f);
        motor_cmd.tau(0.0f);
        motor_cmd.kp(0);
        motor_cmd.kd(0);
    }

    while (g_running) {

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        for (int i = 0; i < 5; i++) {
            if (rigit_target_joint_positions[8] > 0.7805)
                handCmd.motor_cmd()[i].q(1.0f);
            else
                handCmd.motor_cmd()[i].q(0.0f);
        }
        handCmd.motor_cmd()[5].q(rigit_target_joint_positions[7] < -0.7805 ? 1.0f : 0.0f);  // Left thumb

        for (int i = 0; i < 5; i++) {
            if (left_target_joint_positions[8] < -0.7805)
                handCmd.motor_cmd()[i + 6].q(1.0f);
            else
                handCmd.motor_cmd()[i + 6].q(0.0f);
        }
        handCmd.motor_cmd()[11].q(left_target_joint_positions[7] > 0.7805 ? 1.0f : 0.0f);  // Right thumb
        handPublisher->write(handCmd);

        if (!isLowLevelControlMode(g_robot_control_state)) {
            std::cout << "Waiting for LOW_LEVEL-family control mode..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }

        auto &left_shoulder_pitch_cmd = cmd.motors()[15];
        left_shoulder_pitch_cmd.q(std::clamp(left_target_joint_positions[0] * axisDir[15], motorLimit[15][0], motorLimit[15][1]));
        left_shoulder_pitch_cmd.kp(softKpKd[15][0]);
        left_shoulder_pitch_cmd.kd(softKpKd[15][1]);
        auto &left_shoulder_roll_cmd = cmd.motors()[16];
        left_shoulder_roll_cmd.q(std::clamp(left_target_joint_positions[1] * axisDir[16], motorLimit[16][0], motorLimit[16][1]));
        left_shoulder_roll_cmd.kp(softKpKd[16][0]);
        left_shoulder_roll_cmd.kd(softKpKd[16][1]);
        auto &left_shoulder_yaw_cmd = cmd.motors()[17];
        left_shoulder_yaw_cmd.q(std::clamp(left_target_joint_positions[2] * axisDir[17], motorLimit[17][0], motorLimit[17][1]));
        left_shoulder_yaw_cmd.kp(softKpKd[17][0]);
        left_shoulder_yaw_cmd.kd(softKpKd[17][1]);
        auto &left_elbow_pitch_cmd = cmd.motors()[18];
        left_elbow_pitch_cmd.q(std::clamp(left_target_joint_positions[3] * axisDir[18], motorLimit[18][0], motorLimit[18][1]));
        left_elbow_pitch_cmd.kp(softKpKd[18][0]);
        left_elbow_pitch_cmd.kd(softKpKd[18][1]);
        auto &left_wrist_yaw_cmd = cmd.motors()[19];
        left_wrist_yaw_cmd.q(std::clamp(left_target_joint_positions[4] * axisDir[19], motorLimit[19][0], motorLimit[19][1]));
        left_wrist_yaw_cmd.kp(softKpKd[19][0]);
        left_wrist_yaw_cmd.kd(softKpKd[19][1]);
        auto &left_wrist_front_cmd = cmd.motors()[20];
        left_wrist_front_cmd.q(std::clamp(left_target_joint_positions[5] * axisDir[20], motorLimit[20][0], motorLimit[20][1]));
        left_wrist_front_cmd.kp(softKpKd[20][0]);
        left_wrist_front_cmd.kd(softKpKd[20][1]);
        auto &left_wrist_back_cmd = cmd.motors()[21];
        left_wrist_back_cmd.q(std::clamp(left_target_joint_positions[6] * axisDir[21], motorLimit[21][0], motorLimit[21][1]));
        left_wrist_back_cmd.kp(softKpKd[21][0]);
        left_wrist_back_cmd.kd(softKpKd[21][1]);

        auto &right_shoulder_pitch_cmd = cmd.motors()[22];
        right_shoulder_pitch_cmd.q(std::clamp(rigit_target_joint_positions[0] * axisDir[22], motorLimit[22][0], motorLimit[22][1]));
        right_shoulder_pitch_cmd.kp(softKpKd[22][0]);
        right_shoulder_pitch_cmd.kd(softKpKd[22][1]);
        auto &right_shoulder_roll_cmd = cmd.motors()[23];
        right_shoulder_roll_cmd.q(std::clamp(rigit_target_joint_positions[1] * axisDir[23], motorLimit[23][0], motorLimit[23][1]));
        right_shoulder_roll_cmd.kp(softKpKd[23][0]);
        right_shoulder_roll_cmd.kd(softKpKd[23][1]);
        auto &right_shoulder_yaw_cmd = cmd.motors()[24];
        right_shoulder_yaw_cmd.q(std::clamp(rigit_target_joint_positions[2] * axisDir[24], motorLimit[24][0], motorLimit[24][1]));
        right_shoulder_yaw_cmd.kp(softKpKd[24][0]);
        right_shoulder_yaw_cmd.kd(softKpKd[24][1]);
        auto &right_elbow_pitch_cmd = cmd.motors()[25];
        right_elbow_pitch_cmd.q(std::clamp(rigit_target_joint_positions[3] * axisDir[25], motorLimit[25][0], motorLimit[25][1]));
        right_elbow_pitch_cmd.kp(softKpKd[25][0]);
        right_elbow_pitch_cmd.kd(softKpKd[25][1]);
        auto &right_wrist_yaw_cmd = cmd.motors()[26];
        right_wrist_yaw_cmd.q(std::clamp(rigit_target_joint_positions[4] * axisDir[26], motorLimit[26][0], motorLimit[26][1]));
        right_wrist_yaw_cmd.kp(softKpKd[26][0]);
        right_wrist_yaw_cmd.kd(softKpKd[26][1]);
        auto &right_wrist_front_cmd = cmd.motors()[27];
        right_wrist_front_cmd.q(std::clamp(rigit_target_joint_positions[5] * axisDir[27], motorLimit[27][0], motorLimit[27][1]));
        right_wrist_front_cmd.kp(softKpKd[27][0]);
        right_wrist_front_cmd.kd(softKpKd[27][1]);
        auto &right_wrist_back_cmd = cmd.motors()[28];
        right_wrist_back_cmd.q(std::clamp(rigit_target_joint_positions[6] * axisDir[28], motorLimit[28][0], motorLimit[28][1]));
        right_wrist_back_cmd.kp(softKpKd[28][0]);
        right_wrist_back_cmd.kd(softKpKd[28][1]);

        jointPublisher->write(cmd);
    }
}

void robotStateCallback(const igris_c_sdk::msg::dds_::RobotState_ &state) { g_robot_control_state = state.state(); }

int main(int argc, char *argv[]) {
    // Before rclcpp::init: it consumes --ros-args but passes -h through untouched, so without
    // this the node just started up and connected. The GUI example answers -h through its own
    // argc check; this is the node's half of that.
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            printParameterHints();
            return 0;
        }
    }

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("igris_c_masterarm_node");

    const auto port            = node->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    const auto baud            = node->declare_parameter<int>("baud", 1000000);
    const auto domain_id_param = node->declare_parameter<int>("domain_id", 0);
    const auto robot_namespace = node->declare_parameter<std::string>("namespace", "");
    // Must match the layout half of the robot's igris_c.network.transport: the two discover each other only
    // if they agree, and a mismatch is silent rather than an error.
    const auto topic_naming_param = node->declare_parameter<std::string>("dds_topic_naming", "native");

    int domain_id = domain_id_param;
    if (argc > 1) {
        domain_id = std::atoi(argv[1]);
    }

    if (port.empty() || baud <= 0 || domain_id < 0) {
        std::cerr << "Invalid startup parameters."
                  << " port='" << port << "', baud=" << baud << ", domain_id=" << domain_id << std::endl;
        printParameterHints();
        return 1;
    }

    std::string naming_warning;
    const auto topic_naming = igris_c_sdk::parse_topic_naming(topic_naming_param, &naming_warning);
    if (!naming_warning.empty()) {
        std::cerr << "[masterarm] " << naming_warning << std::endl;
    }

    // Same block the SDK examples print (igris_c_sdk_examples::printBanner): same title line,
    // same label column, same order - a masterarm run and an SDK example run read identically.
    // The port and baud, which the SDK banner has no field for, follow in the same column.
    std::cout << "=== IGRIS-C Masterarm Node (Cyclone DDS) ===\n";
    std::cout << "domain id      : " << domain_id << "\n";
    std::cout << "namespace      : " << (robot_namespace.empty() ? "(none)" : robot_namespace) << "\n";
    std::cout << "topic naming   : " << igris_c_sdk::to_config_string(topic_naming) << "\n";
    std::cout << "port           : " << port << "\n";
    std::cout << "baud           : " << baud << "\n";
    std::cout << std::endl;

    ChannelFactory::Instance()->Init(domain_id, robot_namespace, "", topic_naming);

    if (!ChannelFactory::Instance()->IsInitialized()) {
        std::cerr << "Failed to initialize ChannelFactory" << std::endl;
        return 1;
    }

    // Initialize IgrisC_Client
    std::cout << "Initializing IgrisC_Client..." << std::endl;
    IgrisC_Client client;
    client.Init();
    client.SetTimeout(5.0f);

    // Create publisher
    std::cout << "Initializing LowCmd publisher..." << std::endl;
    Publisher<igris_c_sdk::msg::dds_::LowCmd_> lowcmd_pub("lowcmd", QosProfile::SensorData());
    Publisher<igris_c_sdk::msg::dds_::HandCmd_> handcmd_pub("handcmd", QosProfile::SensorData());

    if (!lowcmd_pub.init()) {
        std::cerr << "Failed to initialize LowCmd publisher" << std::endl;
        return 1;
    }
    if (!handcmd_pub.init()) {
        std::cerr << "Failed to initialize HandCmd publisher" << std::endl;
        return 1;
    }

    std::thread publish_thread(lowCmdPublishThread, &lowcmd_pub, &handcmd_pub);

    Subscriber<igris_c_sdk::msg::dds_::LowState_> lowstate_sub("lowstate", QosProfile::SensorData());

    std::cout << "Initialize robotstate subscriber..." << std::endl;
    Subscriber<igris_c_sdk::msg::dds_::RobotState_> robotstateSub("robotstate");
    if (!robotstateSub.init(robotStateCallback)) {
        std::cerr << "Failed to initialize RobotState subscriber" << std::endl;
        return 1;
    }

    RBRSMasterArm sdk(port, baud, 0);

    sdk.open();
    // sdk.set_toggle_current(10);
    sdk.read();

    std::vector<F32> positions  = sdk.get_positions();   // [rad]
    std::vector<F32> velocities = sdk.get_velocities();  // [rad/s]
    std::vector<I16> currents   = sdk.get_currents();    // [mA]
    std::vector<U8> all_ids     = sdk.get_all_ids();

    std::cout << "all ids: " << std::endl;
    for (auto id : all_ids) {
        std::cout << static_cast<int>(id) << " ";
    }
    std::cout << std::endl;
    // return 1;

    std::vector<F32> nowPositions;
    std::vector<U8> ids;
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        sdk.read();

        nowPositions = sdk.get_positions();
        ids          = sdk.get_all_ids();

        for (int i = 0; i < ids.size(); i++) {
            uint8_t id = ids[i];
            if (id >= 10 && id <= 19)  // Right arm
            {
                rigit_target_joint_positions[i] = nowPositions[i];
            } else if (id >= 20 && id <= 29)  // Left arm
            {
                left_target_joint_positions[i - 9] = nowPositions[i];
            }
        }

        std::cout << "------" << std::endl;
        for (int i = 0; i < 7; i++) {
            std::cout << "R Joint " << i << " Pos: " << rigit_target_joint_positions[i] * axisDir[i + 22] << std::endl;
        }
        std::cout << "R thumb: " << rigit_target_joint_positions[7] << std::endl;
        std::cout << "R grip: " << rigit_target_joint_positions[8] << std::endl;
        for (int i = 0; i < 7; i++) {
            std::cout << "L Joint " << i << " Pos: " << left_target_joint_positions[i] * axisDir[i + 15] << std::endl;
        }
        std::cout << "L thmb: " << left_target_joint_positions[7] << std::endl;
        std::cout << "L grip: " << left_target_joint_positions[8] << std::endl;
        std::cout << "------" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Close connection
    sdk.close();

    return 0;
}
