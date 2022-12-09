"""
Launches RViz2 and the robot_state_publisher nodes. 
(an external node publishing joint state is necessary to be able to compute the TFs).
By default, the used model for the robot_description is the WheeBBot (ignition URDF).

"""
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.conditions import IfCondition

def generate_launch_description():
    
    package_share_path = get_package_share_path('wheebbot')

    default_model_path = package_share_path/'/urdf/wheebbot_full.urdf.xacro'
    default_rviz_config_path = package_share_path/'/config/rviz/wheebbot.rviz'

    use_sim_time = LaunchConfiguration('use_sim_time', default = 'false')

    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value = 'false',
            description = 'Use simulation (Gazebo) clock if true')

    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                             description='Absolute path to rviz config file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type = str)

    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        name = 'robot_state_publisher',
        parameters = [{'use_sim_time': use_sim_time,'robot_description': robot_description}]
    )

    rviz_node = Node(
        package ='rviz2',
        executable ='rviz2',
        name = 'rviz2',
        output = 'screen',
        arguments = ['-d', LaunchConfiguration('rvizconfig')],
        parameters = [{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        use_sim_time_arg,
        rviz_node,
        robot_state_publisher_node,
    ])
