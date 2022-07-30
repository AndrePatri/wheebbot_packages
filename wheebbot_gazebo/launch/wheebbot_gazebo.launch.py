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
Launch Gazebo Classic with command line arguments and spawn model.
By default, the wheebbot.world is loaded at startup and WheeBBot is spawned.

"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command,ThisLaunchFileDir,LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path


def generate_launch_description():

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
    package_share_path = get_package_share_path('wheebbot')
    default_model_path = package_share_path/'description/urdf/wheebbot.urdf.xacro'
    default_rviz_config_path = package_share_path/'rviz/wheebbot.rviz'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')            
    is_gui_arg=DeclareLaunchArgument('gui', default_value='true',
                                description='Set to "false" to run headless.')
    is_server_arg=DeclareLaunchArgument('server', default_value='true',
                                description='Set to "false" not to run gzserver.')
    is_gzserver_verbose_arg=DeclareLaunchArgument('gzserver_v', default_value='true',
                                description='Set to "true" to verbose the output.')                         
    is_gzclient_verbose_arg=DeclareLaunchArgument('gzclient_v', default_value='true',
                                description='Set to "true" to verbose the output.')
    world_arg=DeclareLaunchArgument('world', default_value='wheebbot.world',
                                description='World to be loaded (its location needs to be in GAZEBO_RESOURCE_PATH)') 
    is_paused_arg=DeclareLaunchArgument('paused', default_value='false',
                                description='Set to "false" to avoid starting from a paused simulation')                             
    physics_type_arg=DeclareLaunchArgument('physics', default_value='ode',
                                description='Physics engine. Allowed types: ode|bullet|dart|simbody ') 

    use_sim_time_arg=DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')

    rviz_arg = DeclareLaunchArgument(name = 'rvizconfig', default_value = str(default_rviz_config_path),
                                     description = 'Absolute path to rviz config file')

    gzclient_node=IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gzserver.launch.py']),
            condition=IfCondition(LaunchConfiguration('server')),
            launch_arguments={
                'verbose': LaunchConfiguration('gzserver_v'),
                'world': LaunchConfiguration('world'),
                'pause': LaunchConfiguration('paused'),
                'physics': LaunchConfiguration('physics') 
            }.items()
        )
    gzserver_node=IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gzclient.launch.py']),
            condition=IfCondition(LaunchConfiguration('gui')),
            launch_arguments={
                'verbose': LaunchConfiguration('gzclient_v')
            }.items()
        )   

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,'robot_description': robot_description}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    spawn_entity_node = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'wheebbot', '-topic', 'robot_description'],
                        output='screen')

    return LaunchDescription([
        use_sim_time_arg,
        model_arg,
        is_gui_arg,
        is_server_arg,
        is_gzserver_verbose_arg,
        is_gzclient_verbose_arg,
        world_arg,
        is_paused_arg,
        physics_type_arg,
        rviz_arg,
        gzclient_node,
        gzserver_node,
        robot_state_publisher_node,
        spawn_entity_node,
        rviz_node,
    ])
