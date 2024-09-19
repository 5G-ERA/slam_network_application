# Notes about creating a SLAM map on the Theron robot

### Preparation

Build this repository using colcon:

```bash
mkdir workspace && cd workspace
mkdir src && cd src
git clone git@github.com:5G-ERA/slam_network_application.git
cd slam_network_application
# make sure to switch to the "netapp" branch
git switch netapp
cd ../..
colcon build
```

Before using the launch files, etc., make sure to source the workspace setup file:

```bash
source install/setup.sh
```


### ROS2 Nodes

All Python ROS2 nodes are stored in:

https://github.com/5G-ERA/era_5g_experiments/tree/main/robotnik/theron/nodes

(On the Theron robot, they are in: /home/robot/era5g_wp6_demo/nodes)


### Fix odometry node
First, the odometry data needs to be fixed. For this, there is an odom_fix.py re-publisher node:

To run it, simply use:

```bash
python3 odom_fix.py
```


### Recording bag (optional)

Optionally, robot's mapping ride can be recorded into bag:

```bash
ros2 bag record /tf_static /tf /robot/imu/data /robot/rear_laser/scan /robot/front_laser/scan /robot/robotnik_base_control/odom/fixed
```

And later played using:

```bash
ros2 bag play map_bag_entrance/ --clock
```


### Mapping

Start the mapping with:

```bash
ros2 launch era_5g_cartographer mapping.theron.launch.py
```
 
and drive the robot slowly around, in order to build a map.

The process of building the map can also be inspected in rviz.


### Saving the map

Once enough data has been gathered and the map is ready, save it.

First, finish the trajectory:

```bash
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory trajectory_id:\ 0
```

Then save the map using commands:

```bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState filename:\ \'/home/robot/map.pbstream\'
```

and

```bash
ros2 run nav2_map_server map_saver_cli
```

Alltogether, three files will be generated with names similar to: map.pbstream, map-123.yml and map-123.pgm. Rename the files to "map.yml" and "map.pgm", and make sure to also edit the contents of the .yml file, so that the reference to the .pgm file matches its new name.


### Building the container

A docker container can be built using the file provided in:

https://github.com/5G-ERA/slam_network_application/tree/netapp/era_5g_slam/docker/cartographer_localization_theron

First, place all three map files into the **map sub-directory**!

Before building a new container, do not forget to change the tag.

```bash
docker build . -f docker/cartographer_localization_theron/Dockerfile -t but5gera/cartographer_localization_theron:office-0.2-CHANGE_THE_TAG
```

The container includes the newly created map and the localization can be tested using the docker compose scripts in:

https://github.com/5G-ERA/slam_network_application/tree/netapp/era_5g_slam/docker/localization_example/theron_edge

and 

https://github.com/5G-ERA/slam_network_application/tree/netapp/era_5g_slam/docker/localization_example/theron


