# Copyright 2021 RT Corporation, ShotaAk
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Get URDF via xacro
    robot_description_path = os.path.join(
        get_package_share_directory('crane_plus_description'),
        'urdf',
        'crane_plus.urdf.xacro')
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    crane_plus_controllers = os.path.join(
        get_package_share_directory('crane_plus_control'),
        'config',
        'master_slave.yaml'
        )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, crane_plus_controllers],
        output={
          'stdout': 'screen',
          'stderr': 'screen',
          },
        )

    joint_state_controller = ExecuteProcess(
      cmd=['ros2', 'control', 'load_start_controller', 'joint_state_controller'],
      output='screen',
      shell=True
    )

    crane_master_slave_controller = ExecuteProcess(
      cmd=['ros2', 'control', 'load_start_controller', 'crane_master_slave_controller'],
      output='screen',
      shell=True
    )

    return LaunchDescription([
      controller_manager,
      joint_state_controller,
      crane_master_slave_controller
    ])
