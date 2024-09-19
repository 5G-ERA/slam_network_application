#/bin/bash

ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/root/map_data/map.yaml -p use_sim_time:=true &

#ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/robot/era5g_wp6_demo/maps/map_04/map.yaml -p use_sim_time:=true &


sleep 2

ros2 run nav2_util lifecycle_bringup map_server 

sleep 10


