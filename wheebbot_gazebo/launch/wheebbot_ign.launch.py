# Copyright 2020 Open Source Robotics Foundation, Inc.
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
Launch Ignition Gazebo with command line arguments and spawn model.
By default, the wheebbot.world is loaded at startup and WheeBBot is spawned.
Also launch the rviz2 node for visualizing the robot.

"""

from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.actions import ExecuteProcess
from launch.actions import Shutdown

from launch_ros.parameter_descriptions import ParameterValue

from launch.substitutions import Command, LaunchConfiguration,PythonExpression

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    package_share_path = get_package_share_path('wheebbot_description')
    default_model_path = package_share_path/'urdf/wheebbot_full.urdf.xacro'
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model'),\
                                                ' is_floating_base:=true']),
                                       value_type=str)
    default_rviz_config_path = package_share_path/'rviz/wheebbot.rviz'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
           ':'.join([environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                     environ.get('LD_LIBRARY_PATH', default='')])}

    command = [
        'ign gazebo',
        # Pass through arguments to ign gazebo
        LaunchConfiguration('world'),
        '--verbose', LaunchConfiguration('verbose'),
    ]

    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')                                 
    is_gui_arg=DeclareLaunchArgument('gui', default_value='true',
                                description='Set to "false" to run headless.')
    run_headless_arg=DeclareLaunchArgument('headless', default_value='false',
                                description='Set to "true" to run headless (only server).')
    verbosity_arg=DeclareLaunchArgument('verbose', default_value='4',
                                description='Output verbosity (0-4), defaults to 1.')                         
    world_arg=DeclareLaunchArgument('world', default_value='wheebbot_ign.world',
                                description='World to be loaded (its location needs to be in IGN_GAZEBO_RESOURCE_PATH)') 
    is_paused_arg=DeclareLaunchArgument('paused', default_value='false',
                                description='Set to "false" to avoid starting from a paused simulation')                             

    use_sim_time_arg=DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true')

    rviz_arg = DeclareLaunchArgument(name = 'rvizconfig', default_value = str(default_rviz_config_path),
                                     description = 'Absolute path to rviz config file')

    ign_gazebo_node = ExecuteProcess(
            cmd = command,
            output = 'screen',
            shell = True,
            on_exit = Shutdown(),
            additional_env = env,
    )

    spawn_entity_node = Node(package = 'ros_gz_sim', executable = 'create',
                arguments=[
                    '-name', 'wheebbot',
                    '-topic', 'robot_description',
                    ],
                output='screen',
                )

    ros_ign_bridge_node = Node(
        package = 'ros_gz_bridge',
        executable = 'parameter_bridge',
        arguments=[
                # Clock (IGN -> ROS2)
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                # Joint states (IGN -> ROS2)
                '/world/wheebbot_ign_world/model/wheebbot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
                ],
        remappings=[
            ('/world/wheebbot_ign_world/model/wheebbot/joint_state', 'joint_states'),
        ],
        output='screen',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        model_arg,
        world_arg,
        verbosity_arg,
        use_sim_time_arg,
        rviz_arg,
        ign_gazebo_node,
        spawn_entity_node,
        ros_ign_bridge_node,
        robot_state_publisher_node,
        rviz_node,
    ])
