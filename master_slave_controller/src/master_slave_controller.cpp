// Copyright 2021 ShotaAk
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


#include <cmath>
#include <string>
#include <vector>

#include "master_slave_controller/master_slave_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"


namespace master_slave_controller
{
using hardware_interface::LoanedCommandInterface;

MasterSlaveController::MasterSlaveController()
: controller_interface::ControllerInterface(),
  master_joint_names_({}),
  slave_joint_names_({})
{}

controller_interface::return_type
MasterSlaveController::init(const std::string & controller_name)
{
  // initialize lifecycle node
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::SUCCESS) {
    return ret;
  }

  // with the lifecycle node being initialized, we can declare parameters
  node_->declare_parameter<std::vector<std::string>>("master_joints", master_joint_names_);
  node_->declare_parameter<std::vector<std::string>>("slave_joints", slave_joint_names_);
  node_->declare_parameter<std::vector<bool>>("invert_inputs", invert_inputs_);

  return controller_interface::return_type::SUCCESS;
}

CallbackReturn MasterSlaveController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  master_joint_names_ = node_->get_parameter("master_joints").as_string_array();
  if (master_joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'master_joints' parameter was empty");
    return CallbackReturn::ERROR;
  }

  slave_joint_names_ = node_->get_parameter("slave_joints").as_string_array();
  if (slave_joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'slave_joints' parameter was empty");
    return CallbackReturn::ERROR;
  }

  invert_inputs_ = node_->get_parameter("invert_inputs").as_bool_array();
  if (invert_inputs_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'invert_inputs' parameter was empty");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MasterSlaveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(2 * master_joint_names_.size() + slave_joint_names_.size());
  for (const auto & joint_name  : master_joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto & joint_name  : master_joint_names_) {
    conf.names.push_back(joint_name + "/torque_limit");
  }
  for (const auto & joint_name  : slave_joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return conf;
}

controller_interface::InterfaceConfiguration
MasterSlaveController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(master_joint_names_.size() + slave_joint_names_.size());
  for (const auto & joint_name  : master_joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto & joint_name  : slave_joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
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

CallbackReturn MasterSlaveController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!get_ordered_interfaces(
      command_interfaces_, master_joint_names_, hardware_interface::HW_IF_POSITION,
      master_joint_position_command_interface_))
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Expected %u position command interfaces of master joints, got %u",
      master_joint_names_.size(), master_joint_position_command_interface_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!get_ordered_interfaces(
      command_interfaces_, master_joint_names_, "torque_limit",
      master_joint_torque_limit_command_interface_))
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Expected %u torque_limit command interfaces of master joints, got %u",
      master_joint_names_.size(), master_joint_torque_limit_command_interface_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!get_ordered_interfaces(
      state_interfaces_, master_joint_names_, hardware_interface::HW_IF_POSITION,
      master_joint_position_state_interface_))
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Expected %u position state interfaces of master joints, got %u",
      master_joint_names_.size(), master_joint_position_state_interface_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  if (!get_ordered_interfaces(
      command_interfaces_, slave_joint_names_, hardware_interface::HW_IF_POSITION,
      slave_joint_position_command_interface_))
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Expected %u position command interfaces of slave joints, got %u",
      slave_joint_names_.size(), slave_joint_position_command_interface_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!get_ordered_interfaces(
      state_interfaces_, slave_joint_names_, hardware_interface::HW_IF_POSITION,
      slave_joint_position_state_interface_))
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Expected %u position state interfaces of slave joints, got %u",
      slave_joint_names_.size(), slave_joint_position_state_interface_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn MasterSlaveController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  master_joint_position_command_interface_.clear();
  master_joint_torque_limit_command_interface_.clear();
  master_joint_position_state_interface_.clear();
  slave_joint_position_command_interface_.clear();
  slave_joint_position_state_interface_.clear();
  release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type MasterSlaveController::update()
{
  // current state update
  const auto master_joint_num = master_joint_names_.size();
  const auto slave_joint_num = slave_joint_names_.size();

  if(master_joint_num != slave_joint_num){
    RCLCPP_ERROR(
      node_->get_logger(),
      "Master joints [%d] and slave joints [%d] are not same size",
      master_joint_num, slave_joint_num);
    return controller_interface::return_type::ERROR;
  }

  for (auto index = 0ul; index < master_joint_num; ++index) {
    // マスターのトルクOFF
    master_joint_torque_limit_command_interface_[index].get().set_value(0.0);

    // マスターの現在角度をスレーブの目標角度にセットする
    auto master_pos = master_joint_position_state_interface_[index].get().get_value();

    if(invert_inputs_[index]){
      master_pos *= -1.0;
    }
    slave_joint_position_command_interface_[index].get().set_value(master_pos);
  }
  return controller_interface::return_type::SUCCESS;
}

}  // namespace master_slave_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  master_slave_controller::MasterSlaveController, controller_interface::ControllerInterface)
