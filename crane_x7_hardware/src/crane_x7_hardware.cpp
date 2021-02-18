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


#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "crane_x7_hardware/crane_x7_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"


namespace crane_x7_hardware
{

CraneX7Hardware::~CraneX7Hardware()
{
  driver_->torque_enable(false);
  driver_->close_port();
}

return_type CraneX7Hardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  // Get parameters from URDF
  // Initialize member variables
  std::string port_name = info_.hardware_parameters["port_name"];
  int baudrate = std::stoi(info_.hardware_parameters["baudrate"]);
  timeout_seconds_ = std::stod(info_.hardware_parameters["timeout_seconds"]);

  std::vector<uint8_t> dxl_id_list;
  for (auto joint : info_.joints) {
    if (joint.parameters["dxl_id"] != "") {
      dxl_id_list.push_back(std::stoi(joint.parameters["dxl_id"]));
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("CraneX7Hardware"),
        "Joint '%s' does not have 'dxl_id' parameter.",
        joint.name.c_str());
      return return_type::ERROR;
    }
  }

  hw_position_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Open a crane_x7_driver
  driver_ = std::make_shared<CraneX7Driver>(port_name, baudrate, dxl_id_list);
  if (!driver_->open_port()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CraneX7Hardware"), driver_->get_last_error_log());
    return return_type::ERROR;
  }
  if (!driver_->torque_enable(false)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CraneX7Hardware"), driver_->get_last_error_log());
    return return_type::ERROR;
  }

  // Verify that the interface required by CraneX7Hardware is set in the URDF.
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("CraneX7Hardware"),
        "Joint '%s' has %d command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_ERROR(
        rclcpp::get_logger("CraneX7Hardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return return_type::ERROR;
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface>
CraneX7Hardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_position_states_[i])
    );
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CraneX7Hardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_position_commands_[i])
    );
  }

  return command_interfaces;
}

return_type CraneX7Hardware::start()
{
  if (!driver_->torque_enable(true)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CraneX7Hardware"),
      driver_->get_last_error_log());
    return return_type::ERROR;
  }
  // Set current timestamp to disable the communication timeout.
  prev_comm_timestamp_ = rclcpp::Clock().now();

  // Set current joint positions to hw_position_commands.
  read();
  for (uint i = 0; i < hw_position_commands_.size(); i++) {
    hw_position_commands_[i] = hw_position_states_[i];
  }

  status_ = hardware_interface::status::STARTED;
  return return_type::OK;
}

return_type CraneX7Hardware::stop()
{
  driver_->torque_enable(false);
  status_ = hardware_interface::status::STOPPED;
  return return_type::OK;
}

return_type CraneX7Hardware::read()
{
  if (communication_timeout()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CraneX7Hardware"), "Communication timeout!");
    return return_type::ERROR;
  }

  std::vector<double> joint_positions;
  if (!driver_->read_present_joint_positions(&joint_positions)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CraneX7Hardware"),
      driver_->get_last_error_log());
    return return_type::ERROR;
  } else {
    for (uint i = 0; i < hw_position_states_.size(); ++i) {
      hw_position_states_[i] = joint_positions[i];
    }
  }

  prev_comm_timestamp_ = rclcpp::Clock().now();
  return return_type::OK;
}

return_type CraneX7Hardware::write()
{
  if (communication_timeout()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CraneX7Hardware"), "Communication timeout!");
    return return_type::ERROR;
  }

  if (!driver_->write_goal_joint_positions(hw_position_commands_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CraneX7Hardware"),
      driver_->get_last_error_log());
    return return_type::ERROR;
  }

  prev_comm_timestamp_ = rclcpp::Clock().now();
  return return_type::OK;
}

bool CraneX7Hardware::communication_timeout()
{
  if (rclcpp::Clock().now().seconds() - prev_comm_timestamp_.seconds() >= timeout_seconds_) {
    return true;
  } else {
    return false;
  }
}

}  // namespace crane_x7_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  crane_x7_hardware::CraneX7Hardware,
  hardware_interface::SystemInterface
)
