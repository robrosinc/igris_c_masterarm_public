/**
 * @file rbrs_masterarm.hpp
 * @author Myeongjin Lee (menggu1234@robros.co.kr)
 * @brief
 * @copyright Copyright 2025 ROBROS INC.
 *
 */

#ifndef RBRS_MASTERARM_HPP
#define RBRS_MASTERARM_HPP

#include "motor.hpp"
#include "utils.hpp"

#include <dynamixel_sdk/dynamixel_sdk.h>

#include <iostream>
#include <string>
#include <vector>

#define ID_LEN 18

#define TOGGLE_CURRENT 40

// Dynamixel Protocol 2.0 constants
#define DXL_PROTOCOL_2_0 2.0
#define ADDR_DXL2_PRESENT_POSITION 132
#define ADDR_DXL2_TORQUE_ENABLE 64
#define ADDR_DXL2_OPERATING_MODE 11
#define ADDR_DXL2_LED 65
#define ADDR_DXL2_GOAL_POSITION 116
#define ADDR_DXL2_GOAL_CURRENT 102
#define ADDR_DXL2_PRESENT_VELOCITY 128
#define ADDR_DXL2_PRESENT_CURRENT 126
#define LEN_DXL2_POSITION 4
#define LEN_DXL2_VELOCITY 4
#define LEN_DXL2_CURRENT 2

// Dynamixel Protocol 1.0 constants
#define DXL_PROTOCOL_1_0 1.0
#define ADDR_DXL1_PRESENT_POSITION 56
#define ADDR_DXL1_TORQUE_ENABLE 40
#define ADDR_DXL1_OPERATING_MODE 33
#define ADDR_DXL1_GOAL_POSITION 42
#define ADDR_DXL1_GOAL_CURRENT 44
#define ADDR_DXL1_PRESENT_SPEED 58
#define ADDR_DXL1_PRESENT_LOAD 60
#define LEN_DXL1_POSITION 2
#define LEN_DXL1_SPEED 2
#define LEN_DXL1_LOAD 2

class RBRSMasterArm
{
public:
  RBRSMasterArm(const std::string & port, int baudrate, int type = 0);
  ~RBRSMasterArm();

  void open();
  void read();
  void set_toggle_current(U16 current);
  std::vector<F32> get_positions() const;   // [rad]
  std::vector<F32> get_velocities() const;  // [rad/s]
  std::vector<I16> get_currents() const;    // [mA]

  std::vector<I16> get_position_raw() const;
  std::vector<I16> get_velocity_raw() const;
  std::vector<I16> get_current_raw() const;
  std::vector<uint8_t> get_all_ids() const;
  void close();

private:
  int type = 0;
  int baudrate = 0;
  std::string port = "";

  const U8 right_arm[7] = {11, 12, 13, 14, 15, 16, 17};
  const U8 left_arm[7] = {21, 22, 23, 24, 25, 26, 27};
  const U8 right_hand[2] = {18, 19};
  const U8 left_hand[2] = {28, 29};

  std::vector<U8> all_ids;
  std::vector<U8> arm_ids;
  std::vector<U8> hand_ids;

  dynamixel::PortHandler * port_handler = nullptr;
  dynamixel::PacketHandler * packet_handler = nullptr;

  dynamixel::GroupSyncRead * group_sync_read_pos = nullptr;
  dynamixel::GroupSyncRead * group_sync_read_vel = nullptr;
  dynamixel::GroupSyncRead * group_sync_read_cur = nullptr;

  Motor * motor = nullptr;

  void init_motor();
  void init_motor_protocol_2_0();
  void init_motor_protocol_1_0();

  void read_motor_protocol_2_0();
  void read_motor_protocol_1_0();

  void divide_byte(std::vector<U8> & data, int address, int byte_size);

  std::vector<F32> positions;
  std::vector<F32> velocities;
  std::vector<I16> currents;

  std::vector<I16> position_raw;
  std::vector<I16> velocity_raw;
  std::vector<I16> current_raw;
};

#endif  // RBRS_MASTERARM_HPP