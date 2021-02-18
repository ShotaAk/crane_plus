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

#ifndef CRANE_X7_HARDWARE__CRANE_X7_DRIVER_HPP_
#define CRANE_X7_HARDWARE__CRANE_X7_DRIVER_HPP_

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <memory>
#include <string>
#include <vector>

class CraneX7Driver
{
public:
  CraneX7Driver(const std::string port_name, const int baudrate, std::vector<uint8_t> id_list);
  ~CraneX7Driver();

  bool open_port(void);
  void close_port(void);
  std::string get_last_error_log(void);

  bool torque_enable(const bool enable);
  bool write_goal_joint_positions(const std::vector<double> & goal_positions);
  bool read_present_joint_positions(std::vector<double> * joint_positions);

private:
  std::shared_ptr<dynamixel::PortHandler> dxl_port_handler_;
  std::shared_ptr<dynamixel::PacketHandler> dxl_packet_handler_;
  int baudrate_;
  std::vector<uint8_t> id_list_;
  std::string last_error_log_;

  bool read_1byte_list(const uint16_t address, std::vector<uint8_t> * buffer);
  bool read_4byte_list(const uint16_t address, std::vector<uint32_t> * buffer);
  bool parse_dxl_error(
    const std::string func_name, const uint8_t dxl_id,
    const int dxl_comm_result, const uint8_t dxl_packet_error);
  double dxl_pos_to_radian(const uint32_t position);
  uint32_t radian_to_dxl_pos(const double position);
};

#endif  // CRANE_X7_HARDWARE__CRANE_X7_DRIVER_HPP_
