/**
 * @file utils.hpp
 * @author Myeongjin Lee (menggu1234@robros.co.kr)
 * @brief
 * @copyright Copyright 2025 ROBROS INC.
 *
 */

#ifndef UTILS_HPP
#define UTILS_HPP

#include <dynamixel_sdk/dynamixel_sdk.h>

#include <cstdint>
#include <vector>

#define U8 uint8_t
#define U16 uint16_t
#define U32 uint32_t
#define I8 int8_t
#define I16 int16_t
#define I32 int32_t
#define F32 float
#define PI 3.14159265358979323846

I32 uint32_to_int32(U32 value);
I16 uint16_to_int16(U16 value);

F32 dxl_to_degree(I32 position);          // [dxl] -> [degree]
I32 degree_to_dxl(F32 degree);            // [degree] -> [dxl]
F32 dxl_to_radian(I32 position);          // [dxl] -> [rad]
F32 dxl_to_radian_per_second(I32 speed);  // [dxl] -> [rad/s]
I32 radian_to_dxl(F32 radian);            // [rad] -> [dxl]
F32 clip(F32 value, F32 min, F32 max);
F32 normalize(F32 value, F32 min, F32 max);

void sync_write_torque(
  dynamixel::PortHandler * port_handler, dynamixel::PacketHandler * packet_handler,
  std::vector<U8> ids, U8 status);

void sync_write_operating_mode(
  dynamixel::PortHandler * port_handler, dynamixel::PacketHandler * packet_handler,
  std::vector<U8> ids, U8 mode);

void sync_write_led(
  dynamixel::PortHandler * port_handler, dynamixel::PacketHandler * packet_handler,
  std::vector<U8> ids, U8 status);

#endif  // UTILS_HPP
