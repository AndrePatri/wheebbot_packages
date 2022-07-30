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
Launch Ignition Gazebo with command line arguments.
By default, the wheebbot.world is loaded at startup.

"""

from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.actions import ExecuteProcess
from launch.actions import Shutdown

from launch.substitutions import PythonExpression

from launch_ros.actions import Node

def generate_launch_description():

    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
           ':'.join([environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                     environ.get('LD_LIBRARY_PATH', default='')])}

    command = [
        'ign gazebo',
        # Pass through arguments to ign gazebo
        LaunchConfiguration('world'),
        '--verbose', LaunchConfiguration('verbose'),

    ]
    
    is_gui_arg=DeclareLaunchArgument('gui', default_value='true',
                                description='Set to "false" to run headless.')
    run_headless_arg=DeclareLaunchArgument('headless', default_value='false',
                                description='Set to "true" to run headless (only server).')
    verbosity_arg=DeclareLaunchArgument('verbose', default_value='1',
                                description='Output verbosity (0-4), defaults to 1.')                         
    world_arg=DeclareLaunchArgument('world', default_value='wheebbot_ign.world',
                                description='World to be loaded (its location needs to be in IGN_GAZEBO_RESOURCE_PATH)') 
    is_paused_arg=DeclareLaunchArgument('paused', default_value='false',
                                description='Set to "false" to avoid starting from a paused simulation')                             

    return LaunchDescription([
        world_arg,
        verbosity_arg,
        ExecuteProcess(
            cmd=command,
            output='screen',
            shell=True,
            on_exit=Shutdown(),
            additional_env=env,
        )
    ])

# Add boolean commands if true
def _boolean_command(arg):
    cmd = ['"--', arg, '" if "true" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd


# Add string commands if not empty
def _arg_command(arg):
    cmd = ['"--', arg, '" if "" != "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd


# Add gazebo_ros plugins if true
def _plugin_command(arg):
    cmd = ['"-s libgazebo_ros_', arg, '.so" if "true" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd

