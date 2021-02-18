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


#ifndef MASTER_SLAVE_CONTROLLER__MASTER_SLAVE_CONTROLLER_HPP_
#define MASTER_SLAVE_CONTROLLER__MASTER_SLAVE_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "master_slave_controller/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"


namespace master_slave_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


class MasterSlaveController : public controller_interface::ControllerInterface
{
public:
  MASTER_SLAVE_CONTROLLER_PUBLIC
  MasterSlaveController();

  MASTER_SLAVE_CONTROLLER_PUBLIC
  controller_interface::return_type init(const std::string & controller_name) override;

  MASTER_SLAVE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  MASTER_SLAVE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  MASTER_SLAVE_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  MASTER_SLAVE_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  MASTER_SLAVE_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  MASTER_SLAVE_CONTROLLER_PUBLIC
  controller_interface::return_type update() override;

protected:
  std::vector<std::string> master_joint_names_;
  std::vector<std::string> slave_joint_names_;

  // For convenience, we have ordered the interfaces so i-th position matches i-th index
  // in joint_names_
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  master_joint_position_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  slave_joint_position_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  master_joint_torque_limit_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
  master_joint_position_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
  slave_joint_position_state_interface_;
};

}  // namespace master_slave_controller

#endif  // MASTER_SLAVE_CONTROLLER__MASTER_SLAVE_CONTROLLER_HPP_
