from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

use_sim_time = True
use_sim_time_param = [{'use_sim_time': use_sim_time}]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value="False", description='Launch RViz2?'),
        DeclareLaunchArgument('config', default_value="localization.summitxl.lua", description='The configuration file for the Summit XL robot.'),
        DeclareLaunchArgument('map_file', description='The pbstream to load.'),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            parameters=use_sim_time_param,
            remappings=[('/points2', '/robot/top_laser/point_cloud'), 
                        ('/imu', '/synchronized/imu'), 
                        ('/odom', '/synchronized/odom')],
            arguments=["-configuration_directory", PathJoinSubstitution([FindPackageShare('era_5g_cartographer'), 'config']), 
                       "-configuration_basename", LaunchConfiguration('config'),
                       "-load_state_filename", LaunchConfiguration('map_file')] 
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=use_sim_time_param,
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),        
    ])


