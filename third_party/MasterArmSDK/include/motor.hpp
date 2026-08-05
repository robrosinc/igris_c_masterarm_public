/**
 * @file motor.hpp
 * @author Myeongjin Lee (menggu1234@robros.co.kr)
 * @brief
 * @copyright Copyright 2025 ROBROS INC.
 *
 */

#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "serial.hpp"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#define HIBYTE(w) ((U8)((((U64)(w)) >> 8) & 0xff))
#define LOBYTE(w) ((U8)(((U64)(w)) & 0xff))

#define HIWORD(l) ((U16)((((U64)(l)) >> 16) & 0xffff))
#define LOWORD(l) ((U16)(((U64)(l)) & 0xffff))

static constexpr I16 GOAL_CURRENT_MIN = -2047;
static constexpr I16 GOAL_CURRENT_MAX = 2047;

class Motor
{
public:
  Motor(const std::string & port, int baudrate);
  ~Motor();

  void open();
  void close();
  void set_timeout(int timeout_ms);

  bool ping(U8 id) const;
  bool is_connected() const;
  void verify_connection() const;

  void write1ByteCommand(U8 id, U8 address, U8 data);
  void write2ByteCommand(U8 id, U8 address, U16 data);
  void write4ByteCommand(U8 id, U8 address, U32 data);

  U8 read1ByteResponse(U8 id, U8 address);
  U16 read2ByteResponse(U8 id, U8 address);
  U32 read4ByteResponse(U8 id, U8 address);

  std::map<U8, std::vector<U8>> syncread(
    U8 start_address, const std::vector<U8> & ids, U8 data_length) const;

private:
  Serial * serial = nullptr;
  std::string port;
  int baudrate;

  void divide_bytes(std::vector<U8> & out, U32 value, int byte_size);
  U8 checksum(const std::vector<U8> & data) const;

  std::vector<U8> generate_packet(U8 id, U8 instruction, const std::vector<U8> & data) const;

  U32 readNByteResponse(U8 id, U8 address, U8 n) const;
  void readWriteResponse(U8 id) const;
};

#endif
