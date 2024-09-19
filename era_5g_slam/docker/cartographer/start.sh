#!/bin/bash

source /opt/ros/humble/setup.sh
source /root/slam_ws/install/setup.sh
cd

exec ros2 run point_cloud_transport republish --in_transport draco --out_transport raw --ros-args --remap in/draco:=/out/draco --remap out:=/robot/top_laser/point_cloud &
exec ros2 run era_5g_slam_commander map_downloader /root/shared_dir/ &
exec ros2 run era_5g_slam_commander map_creator -t -o /root/shared_dir/ --launch-mapping mapping.launch.py --launch-localization localization.launch.py --launch-octomap-server octomap_server.launch.py


