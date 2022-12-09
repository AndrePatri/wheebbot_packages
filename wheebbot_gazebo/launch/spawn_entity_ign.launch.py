# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""
Spawn entity into an already running ignition node.
This node launches both a robot_state_publisher node, which is needed for providing the robot_description;
the robot_description is used by the spawner node to spawn the URDF of the robot which, by default, is the WheeBBot.

"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command,ThisLaunchFileDir,LaunchConfiguration
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription

from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    package_share_path = get_package_share_path('wheebbot_description')
    default_model_path = package_share_path/'/urdf/wheebbot_ign.urdf.xacro'
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,'robot_description': robot_description}]
    )

    spawn_node = Node(package='ros_ign_gazebo', executable='create',
                arguments=[
                    '-name', 'wheebbot',
                    '-topic', 'robot_description',
                    ],
                output='screen',
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        spawn_node,
    ])
