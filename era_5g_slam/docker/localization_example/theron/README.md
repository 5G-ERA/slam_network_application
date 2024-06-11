

# Notes about running SLAM on the Theron robot


### ROS2 Nodes

All Python ROS2 nodes are stored in:

https://github.com/5G-ERA/era_5g_experiments/tree/main/robotnik/theron/nodes

(On the Theron robot, they are in: /home/robot/era5g_wp6_demo/nodes)


#### Fix odometry node
First the odometry data needs to be fixed. For this, there is an odom_fix.py re-publisher node:

To run it, simply use:

```bash
python3 odom_fix.py
```

#### Sync node
The data from laser sensors, imu and odometry are synced to prevent overloading the Rely Network Application. 

For this purpose, there is a sync node. Run it with:

```bash
python3 sync.py
```

Note: We also tried adding these two nodes (odom_fix and sync) to the docker compose file, but on Theron this resulted into constantly crashing rviz (likely because of Foxy), so it's better to start them manually.


### Visualizing the map for rviz

The final map of the Robotnik entrance area (with covered bottom parts of glass walls next to the reception desk)  is stored in:

https://github.com/5G-ERA/slam_network_application/tree/netapp/era_5g_slam/docker/cartographer_localization_theron/map

(On the Theron robot, it is in: /home/robot/era5g_wp6_demo/maps/map_05/)

To visualize the entrance map (to verify that everything works in rviz), run:


```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/robot/era5g_wp6_demo/maps/map_05/map.yaml -p use_sim_time:=true &
```

and then

```bash
ros2 run nav2_util lifecycle_bringup map_server &
```

(Note: Normally, visualizing the map would be done from the remote server, but we were not able to make this work for Theron, so we run the visualization commands locally, directly on the robot.)


If you later want to kill the background jobs that visualize the map, you can simply run ```jobs``` to inspect their IDs and then ```kill -9 %1``` (or ```kill -9 %2``` etc.).


### Localization

To run the localization using the Relay Network Application and the Middleware, use the following docker compose file (docker-compose.yml):

https://github.com/5G-ERA/slam_network_application/blob/netapp/era_5g_slam/docker/localization_example/theron/docker-compose.yml

```bash
docker compose up
```


### Switching between two localization sources

Alternatively, there is a possibility to use two Middlewares and switch between the localizations obtained from them.


For this purpose, also run the switch node:

```bash
python3 switch.py
```

Use the two docker files with different configurations:

```bash
docker compose -f compose.source1.yml up
```

and

```bash
docker compose -f compose.source2.yml up
```

Then localization source can be switched by calling:

```bash
ros2 service call /switch_tf std_srvs/srv/Trigger
```

