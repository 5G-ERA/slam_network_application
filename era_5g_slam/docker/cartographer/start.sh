#!/bin/bash

source /opt/ros/humble/setup.sh
source /root/slam_ws/install/setup.sh
cd

#exec ros2 launch era_5g_cartographer mapping.launch.py &
exec ros2 run era_5g_slam_commander map_creator -t -o /root/shared_dir/ --launch-mapping mapping.launch.py --launch-localization localization.launch.py


