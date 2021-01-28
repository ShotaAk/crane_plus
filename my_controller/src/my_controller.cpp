// Copyright 2021 RT Corporation
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


#include <string>
#include <vector>

#include "my_controller/my_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"


namespace my_controller
{
using hardware_interface::LoanedCommandInterface;

MyController::MyController()
: controller_interface::ControllerInterface(),
  joint_names_({})
{}

controller_interface::return_type
MyController::init(const std::string & controller_name)
{
  // initialize lifecycle node
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::SUCCESS) {
    return ret;
  }

  // with the lifecycle node being initialized, we can declare parameters
  node_->declare_parameter<std::vector<std::string>>("joints", joint_names_);

  return controller_interface::return_type::SUCCESS;
}

CallbackReturn MyController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = node_->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MyController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size());
  for (const auto & joint_name  : joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return conf;
}

controller_interface::InterfaceConfiguration
MyController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(2 * joint_names_.size());
  for (const auto & joint_name  : joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    conf.names.push_back(joint_name + "/" + "load");
  }
  return conf;
}

// Fill ordered_interfaces with references to the matching interfaces
// in the same order as in joint_names
template<typename T>
bool get_ordered_interfaces(
  std::vector<T> & unordered_interfaces, const std::vector<std::string> & joint_names,
  const std::string & interface_type, std::vector<std::reference_wrapper<T>> & ordered_interfaces)
{
  // command_interfaces or state_interfacesから特定のinterfaceを抽出する
  // interfaces_[joint1(pos), joint1(vel), joint2(pos), joint2(vel), ...]
  // -> position_interfaces[joint1(pos), joint2(pos), ...]
  for (const auto & joint_name : joint_names) {
    for (auto & command_interface : unordered_interfaces) {
      if ((command_interface.get_name() == joint_name) &&
        (command_interface.get_interface_name() == interface_type))
      {
        ordered_interfaces.push_back(std::ref(command_interface));
      }
    }
  }

  return joint_names.size() == ordered_interfaces.size();
}

CallbackReturn MyController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!get_ordered_interfaces(
      command_interfaces_, joint_names_, hardware_interface::HW_IF_POSITION,
      joint_position_command_interface_))
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Expected %u position command interfaces, got %u",
      joint_names_.size(), joint_position_command_interface_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!get_ordered_interfaces(
      state_interfaces_, joint_names_, hardware_interface::HW_IF_POSITION,
      joint_position_state_interface_))
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Expected %u position state interfaces, got %u",
      joint_names_.size(), joint_position_state_interface_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!get_ordered_interfaces(
      state_interfaces_, joint_names_, "load",
      joint_load_state_interface_))
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Expected %u load state interfaces, got %u",
      joint_names_.size(), joint_load_state_interface_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn MyController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_position_command_interface_.clear();
  joint_position_state_interface_.clear();
  joint_load_state_interface_.clear();
  release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type MyController::update()
{
  return controller_interface::return_type::SUCCESS;
}

}  // namespace my_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  my_controller::MyController, controller_interface::ControllerInterface)
