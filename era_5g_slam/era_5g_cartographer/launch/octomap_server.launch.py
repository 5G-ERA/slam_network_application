from launch import LaunchDescription
from launch_ros.actions import Node


use_sim_time = True

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            parameters=[{'use_sim_time': use_sim_time,
                         'resolution': 0.05,
                         'frame_id': 'robot/odom',
                         'base_frame_id': 'robot/base_footprint',
                         'filter_ground': False,
                         'occupancy_min_z': 0.1,
                         'occupancy_max_z': 2.5,
                         'sensor_model.max_range': 10.0}],
            remappings=[('cloud_in', '/robot/top_laser/point_cloud')]
            
        )
    ])
    