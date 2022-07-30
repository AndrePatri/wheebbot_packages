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
Launch Gazebo Classic with command line arguments.
By default, the wheebbot.world is loaded at startup.

"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir,LaunchConfiguration

def generate_launch_description():
  
    is_gui_arg=DeclareLaunchArgument('gui', default_value='true',
                                description='Set to "false" to run headless.')
    is_server_arg=DeclareLaunchArgument('server', default_value='true',
                                description='Set to "false" not to run gzserver.')
    is_gzserver_verbose_arg=DeclareLaunchArgument('gzserver_v', default_value='false',
                                description='Set to "true" to verbose the output.')                         
    is_gzclient_verbose_arg=DeclareLaunchArgument('gzclient_v', default_value='false',
                                description='Set to "true" to verbose the output.')
    world_arg=DeclareLaunchArgument('world', default_value='wheebbot.world',
                                description='World to be loaded (its location needs to be in GAZEBO_RESOURCE_PATH)') 
    is_paused_arg=DeclareLaunchArgument('paused', default_value='false',
                                description='Set to "false" to avoid starting from a paused simulation')                             
    physics_type_arg=DeclareLaunchArgument('physics', default_value='ode',
                                description='Physics engine. Allowed types: ode|bullet|dart|simbody ') 

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
  
    return LaunchDescription([
        is_gui_arg,
        is_server_arg,
        is_gzserver_verbose_arg,
        is_gzclient_verbose_arg,
        world_arg,
        is_paused_arg,
        physics_type_arg,
        gzclient_node,
        gzserver_node,
        ])
