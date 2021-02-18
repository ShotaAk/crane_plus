// Copyright 2021 RT Corporation, ShotaAk
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "crane_x7_hardware/crane_x7_driver.hpp"

constexpr double PROTOCOL_VERSION = 2.0;
constexpr int DXL_HOME_POSITION = 2048;
constexpr double TO_RADIANS = (180.0 / 2048.0) * M_PI / 180.0;
constexpr double TO_DXL_POS = 1.0 / TO_RADIANS;

// Dynamixel XM Series address table
// Ref: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
constexpr uint16_t ADDR_TORQUE_ENABLE = 64;
constexpr uint16_t ADDR_GOAL_POSITION = 116;
constexpr uint16_t ADDR_PRESENT_POSITION = 132;

CraneX7Driver::CraneX7Driver(
  const std::string port_name, const int baudrate,
  std::vector<uint8_t> id_list)
: baudrate_(baudrate), id_list_(id_list)
{
  dxl_port_handler_ = std::shared_ptr<dynamixel::PortHandler>(
    dynamixel::PortHandler::getPortHandler(port_name.c_str()));
  dxl_packet_handler_ = std::shared_ptr<dynamixel::PacketHandler>(
    dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION));
}

CraneX7Driver::~CraneX7Driver()
{
  close_port();
}

bool CraneX7Driver::open_port(void)
{
  if (!dxl_port_handler_->openPort()) {
    last_error_log_ = std::string(__func__) + ": unable to open dynamixel port: " +
      dxl_port_handler_->getPortName();
    return false;
  }

  if (!dxl_port_handler_->setBaudRate(baudrate_)) {
    last_error_log_ = std::string(__func__) + ": unable to set baudrate" +
      std::to_string(dxl_port_handler_->getBaudRate());
    return false;
  }

  return true;
}

void CraneX7Driver::close_port(void)
{
  dxl_port_handler_->closePort();
}

std::string CraneX7Driver::get_last_error_log(void)
{
  return last_error_log_;
}

bool CraneX7Driver::torque_enable(const bool enable)
{
  bool retval = true;
  for (auto dxl_id : id_list_) {
    uint8_t dxl_error = 0;
    int dxl_result = dxl_packet_handler_->write1ByteTxRx(
      dxl_port_handler_.get(),
      dxl_id, ADDR_TORQUE_ENABLE, enable, &dxl_error);

    if (!parse_dxl_error(std::string(__func__), dxl_id, dxl_result, dxl_error)) {
      retval = false;
    }
  }

  return retval;
}

bool CraneX7Driver::write_goal_joint_positions(const std::vector<double> & goal_positions)
{
  if (goal_positions.size() != id_list_.size()) {
    last_error_log_ = std::string(__func__) + ": vectors size does not match: " +
      " goal_positions:" + std::to_string(goal_positions.size()) +
      ", id_list:" + std::to_string(id_list_.size());
    return false;
  }

  bool retval = true;

  for (size_t i = 0; i < goal_positions.size(); i++) {
    uint8_t dxl_error = 0;
    uint32_t goal_position = radian_to_dxl_pos(goal_positions[i]);
    auto dxl_id = id_list_[i];
    int dxl_result = dxl_packet_handler_->write4ByteTxRx(
      dxl_port_handler_.get(),
      dxl_id, ADDR_GOAL_POSITION, goal_position, &dxl_error);

    if (!parse_dxl_error(std::string(__func__), dxl_id, dxl_result, dxl_error)) {
      retval = false;
    }
  }

  return retval;
}


bool CraneX7Driver::read_present_joint_positions(std::vector<double> * joint_positions)
{
  std::vector<uint32_t> buffer;
  bool retval = read_4byte_list(ADDR_PRESENT_POSITION, &buffer);

  for (auto data : buffer) {
    joint_positions->push_back(dxl_pos_to_radian(data));
  }

  return retval;
}

bool CraneX7Driver::read_1byte_list(const uint16_t address, std::vector<uint8_t> * buffer)
{
  bool retval = true;
  for (auto dxl_id : id_list_) {
    uint8_t dxl_error = 0;
    uint8_t data = 0;
    int dxl_result = dxl_packet_handler_->read1ByteTxRx(
      dxl_port_handler_.get(),
      dxl_id, address, &data, &dxl_error);

    if (!parse_dxl_error(std::string(__func__), dxl_id, dxl_result, dxl_error)) {
      retval = false;
    }

    buffer->push_back(data);
  }

  return retval;
}

bool CraneX7Driver::read_4byte_list(const uint16_t address, std::vector<uint32_t> * buffer)
{
  bool retval = true;
  for (auto dxl_id : id_list_) {
    uint8_t dxl_error = 0;
    uint32_t data = 0;
    int dxl_result = dxl_packet_handler_->read4ByteTxRx(
      dxl_port_handler_.get(),
      dxl_id, address, &data, &dxl_error);

    if (!parse_dxl_error(std::string(__func__), dxl_id, dxl_result, dxl_error)) {
      retval = false;
    }

    buffer->push_back(data);
  }

  return retval;
}

bool CraneX7Driver::parse_dxl_error(
  const std::string func_name, const uint8_t dxl_id,
  const int dxl_comm_result, const uint8_t dxl_packet_error)
{
  bool retval = true;

  if (dxl_comm_result != COMM_SUCCESS) {
    last_error_log_ = func_name + ": dxl_id: " + std::to_string(dxl_id) + " :" +
      std::string(dxl_packet_handler_->getTxRxResult(dxl_comm_result));
    retval = false;
  }

  if (dxl_packet_error != 0) {
    last_error_log_ = func_name + ": dxl_id: " + std::to_string(dxl_id) + " :" +
      std::string(dxl_packet_handler_->getRxPacketError(dxl_packet_error));
    retval = false;
  }

  return retval;
}

double CraneX7Driver::dxl_pos_to_radian(const uint32_t position)
{
  return (position - DXL_HOME_POSITION) * TO_RADIANS;
}

uint32_t CraneX7Driver::radian_to_dxl_pos(const double position)
{
  return position * TO_DXL_POS + DXL_HOME_POSITION;
}
