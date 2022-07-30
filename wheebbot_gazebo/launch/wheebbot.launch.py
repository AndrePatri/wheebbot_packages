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
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command,ThisLaunchFileDir,LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path
from launch.actions import ExecuteProcess
from launch.actions import Shutdown
from os import environ

def generate_launch_description():

    wheebbot_description_path = get_package_share_path('wheebbot_description')
    default_model_path = wheebbot_description_path/'urdf/wheebbot.urdf.xacro'
    default_rviz_config_path = wheebbot_description_path/'config/rviz/wheebbot.rviz'

    ## launch arguments
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')            
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
    verbosity_arg=DeclareLaunchArgument('verbose', default_value='1',
                            description='Output verbosity (0-4), defaults to 1.')

    is_gazebo_classic_arg=DeclareLaunchArgument(
            'is_gazebo_classic',
            default_value='true',
            description='Whether to use Gazebo Classic as a simulator or (Ignition) Gazebo')

    use_sim_time_arg=DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')

    rviz_arg = DeclareLaunchArgument(name = 'rvizconfig', default_value = str(default_rviz_config_path),
                                     description = 'Absolute path to rviz config file')


    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model'),\
                                                ' is_gazebo_classic:=', LaunchConfiguration('is_gazebo_classic')]),
                                       value_type=str)

    ## nodes

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time', default='true'),\
                    'robot_description': robot_description}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time', default='true')}]
    )

    # Gazebo classic    
    gzclient_node=IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gzserver.launch.py']),
            condition=(IfCondition(LaunchConfiguration('is_gazebo_classic'))),
            launch_arguments={
                'verbose': LaunchConfiguration('gzserver_v'),
                'world': LaunchConfiguration('world'),
                'pause': LaunchConfiguration('paused'),
                'physics': LaunchConfiguration('physics') 
            }.items()
        )

    gzserver_node=IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/gzclient.launch.py']),
            condition=(IfCondition(LaunchConfiguration('is_gazebo_classic'))),
            launch_arguments={
                'verbose': LaunchConfiguration('gzclient_v')
            }.items()
        )   

    spawn_entity_node_gz = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-entity', 'wheebbot', '-topic', 'robot_description'],
                    output='screen', 
                    condition= IfCondition(LaunchConfiguration('is_gazebo_classic')))
                            
    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
        ':'.join([environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                    environ.get('LD_LIBRARY_PATH', default='')])}

    ign_gazebo_node = ExecuteProcess(
        cmd = [\
            'ign gazebo',\
            LaunchConfiguration('world'), \
            '--verbose', LaunchConfiguration('verbose'), \
        ],
        output = 'screen',
        shell = True,
        on_exit = Shutdown(),
        additional_env = env,
        condition= UnlessCondition(LaunchConfiguration('is_gazebo_classic')),
    )

    spawn_entity_node_ign = Node(package = 'ros_ign_gazebo', executable = 'create',
        arguments=[
            '-name', 'wheebbot',
            '-topic', 'robot_description',
            ],
        output='screen',
        condition= UnlessCondition(LaunchConfiguration('is_gazebo_classic')),
    )

    ros_ign_bridge_node = Node(
        package = 'ros_ign_bridge',
        executable = 'parameter_bridge',
        arguments=[
                # Clock (IGN -> ROS2)
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                # Joint states (IGN -> ROS2)
                '/world/wheebbot_ign_world/model/wheebbot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
                ],
        remappings=[
            ('/world/wheebbot_ign_world/model/wheebbot/joint_state', 'joint_states'),
        ],
        output='screen',
        condition= UnlessCondition(LaunchConfiguration('is_gazebo_classic')),
    )

    # returning arguments and nodes
    return LaunchDescription([
        is_gazebo_classic_arg,
        use_sim_time_arg,
        model_arg,
        is_gzserver_verbose_arg,
        is_gzclient_verbose_arg,
        verbosity_arg,
        world_arg,
        is_paused_arg,
        physics_type_arg,
        rviz_arg,
        gzclient_node,
        gzserver_node,
        ign_gazebo_node,
        ros_ign_bridge_node, 
        robot_state_publisher_node,
        spawn_entity_node_gz,
        spawn_entity_node_ign,
        rviz_node,
    ])