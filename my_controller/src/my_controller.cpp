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
#include "hardware_interface/loaned_command_interface.hpp"


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
  return controller_interface::return_type::SUCCESS;
}

CallbackReturn MyController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MyController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
MyController::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

template<typename T>
bool get_ordered_interfaces(
  std::vector<T> & unordered_interfaces, const std::vector<std::string> & joint_names,
  const std::string & interface_type, std::vector<std::reference_wrapper<T>> & ordered_interfaces)
{
  return joint_names.size() == ordered_interfaces.size();
}

CallbackReturn MyController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn MyController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
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
