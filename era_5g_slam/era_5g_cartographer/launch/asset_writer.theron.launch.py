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
        DeclareLaunchArgument('config', default_value="asset_writer.2d.lua", description='The configuration file for the Summit XL robot.'),
        DeclareLaunchArgument('bag_name', description='The name of the bagfile.'),
        DeclareLaunchArgument('pbstream_file', description='The name of the bagfile.'),
        Node(
            package='cartographer_ros',
            executable='cartographer_assets_writer',
            name='assets_writer_node',
            parameters=use_sim_time_param,
            remappings=[('/odom', '/synchronized/odom'),
                        ('/scan_1', '/synchronized/scan_1'),
                        ('/scan_2', '/synchronized/scan_2'),
                        ('/imu', '/synchronized/imu')],
            arguments=["-configuration_directory", PathJoinSubstitution([FindPackageShare('era_5g_cartographer'), 'config']), 
                       "-configuration_basename", LaunchConfiguration('config'),
                       "-bag_filenames", LaunchConfiguration('bag_name'),
                       "-pose_graph_filename", LaunchConfiguration('pbstream_file'),]
            
        ),     
    ])
