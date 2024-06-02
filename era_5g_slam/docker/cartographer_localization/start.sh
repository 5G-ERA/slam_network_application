#!/bin/bash

source /opt/ros/humble/setup.sh
source /root/slam_ws/install/setup.sh
cd

exec ros2 run point_cloud_transport republish --ros-args -p in_transport:=draco -p out_transport:=raw --remap in/draco:=/synchronized/draco --remap out:=/robot/top_laser/point_cloud &
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/root/map_data/map.yaml -p use_sim_time:=true &
sleep 2
ros2 run nav2_util lifecycle_bringup map_server &
#exec ros2 run era_5g_slam_commander map_downloader /root/shared_dir/ &
#exec ros2 run era_5g_slam_commander tf_remover --ros-args --remap tf_in:=/tf --remap tf_out:=/tf_filtered
#exec ros2 run era_5g_slam_commander map_creator -t -o /root/shared_dir/ --launch-mapping mapping.launch.py --launch-localization localization.launch.py --launch-octomap-server octomap_server.launch.py
exec ros2 launch era_5g_cartographer localization.launch.py map_file:=/root/map_data/map.pbstream

