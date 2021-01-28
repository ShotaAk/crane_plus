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


#ifndef MY_CONTROLLER__MY_CONTROLLER_HPP_
#define MY_CONTROLLER__MY_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "my_controller/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"


namespace my_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


class MyController : public controller_interface::ControllerInterface
{
public:
  MY_CONTROLLER_PUBLIC
  MyController();

  MY_CONTROLLER_PUBLIC
  controller_interface::return_type init(const std::string & controller_name) override;

  MY_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  MY_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  MY_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  MY_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  MY_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  MY_CONTROLLER_PUBLIC
  controller_interface::return_type update() override;

protected:
};

}  // namespace my_controller

#endif  // MY_CONTROLLER__MY_CONTROLLER_HPP_
