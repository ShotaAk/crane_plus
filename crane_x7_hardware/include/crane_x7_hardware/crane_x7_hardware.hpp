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


#ifndef CRANE_X7_HARDWARE__CRANE_X7_HARDWARE_HPP_
#define CRANE_X7_HARDWARE__CRANE_X7_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "crane_x7_hardware/crane_x7_driver.hpp"
#include "crane_x7_hardware/visibility_control.h"
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

using hardware_interface::return_type;

namespace crane_x7_hardware
{
class CraneX7Hardware : public
  hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CraneX7Hardware)

  CRANE_X7_HARDWARE_PUBLIC
  ~CraneX7Hardware();

  CRANE_X7_HARDWARE_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo & info) override;

  CRANE_X7_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  CRANE_X7_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CRANE_X7_HARDWARE_PUBLIC
  return_type start() override;

  CRANE_X7_HARDWARE_PUBLIC
  return_type stop() override;

  CRANE_X7_HARDWARE_PUBLIC
  return_type read() override;

  CRANE_X7_HARDWARE_PUBLIC
  return_type write() override;

private:
  bool communication_timeout();

  std::shared_ptr<CraneX7Driver> driver_;
  double timeout_seconds_;

  std::vector<double> hw_position_commands_;
  std::vector<double> hw_position_states_;

  rclcpp::Time prev_comm_timestamp_;
};
}  // namespace crane_x7_hardware

#endif  // CRANE_X7_HARDWARE__CRANE_X7_HARDWARE_HPP_
