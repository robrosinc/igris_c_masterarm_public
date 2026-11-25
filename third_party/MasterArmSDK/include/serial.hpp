/**
 * @file serial.hpp
 * @author Myeongjin Lee (menggu1234@robros.co.kr)
 * @brief
 * @copyright Copyright 2025 ROBROS INC.
 *
 */

#ifndef SERIAL_HPP
#define SERIAL_HPP

#pragma once

#include <termios.h>

#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#define U8 uint8_t
#define U16 uint16_t
#define U32 uint32_t
#define U64 uint64_t
#define I8 int8_t
#define I16 int16_t
#define I32 int32_t
#define I64 int64_t
#define F32 float
#define F64 double
#define PI 3.14159265358979323846

class Serial
{
public:
  Serial(const std::string & port, int baudrate);
  ~Serial();

  void open();
  void close();

  bool is_open() const noexcept { return fd >= 0; }

  void write(const std::vector<U8> & data);
  void read(std::vector<U8> & data);

  bool read_with_timeout(std::vector<U8> & data, int timeout_ms);

  void flush();
  void flush_input();
  void flush_output();

  void set_timeout(int timeout_ms);
  void set_baudrate(int baudrate);

  U32 get_fd() const noexcept { return fd; }

private:
  std::string port;
  int baudrate;
  int fd;

  static speed_t baudrate_to_speed(int baudrate);
};

#endif