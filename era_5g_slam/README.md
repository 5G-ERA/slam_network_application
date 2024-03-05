# Cartographer Network Application

This repository contains the configuration and instructions for setting up the Cartographer network application. It provides a step-by-step guide on how to build Docker images and deploy the application.

## Table of Contents

- [Cartographer Network Application](#cartographer-network-application)
  - [Table of Contents](#table-of-contents)
  - [Configuration](#configuration)
  - [Instalation](#instalation)
  - [Usage](#usage)
    - [Native deployment](#native-deployment)
    - [Docker compose deployment](#docker-compose-deployment)
    - [Default configuration](#default-configuration)
  - [Contributing](#contributing)
  - [Build Docker](#build-docker)
  - [License](#license)



## Configuration

The Cartographer configurations are located in the `era_5g_cartographer/config` folder. By default, there are two configuration files for the Summit XL robot - `mapping.summitxl.lua` and `localization.summitxl.lua`. The first one is configured to provide the complet SLAM (i.e. localization and mapping at the same time), the second one only provides the localization, therefore, it requires a pre-recorded map (in the form of pbstream file). Correspondingly, the `launch` folder contains two launch files, `mapping.launch.py` and `localization.launch.py`. 

The `*.lua` configuration files specifies robot-specific parameters of the SLAM, e.g., the name of the reference frame, odometry frame, SLAM finetuning parameters etc. 

## Instalation

Only needed when you need to run the network application in the native mode (see next). This guide assumes that the Relay is already installed.

1. Create ROS2 Workspace (see https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
2. Clone the repository inside the workspave `git clone git@github.com:5G-ERA/slam_network_application.git`
3. Build the workspace `colcon build`

## Usage

The Cartographer network application can be used in two modes: natively and with docker compose. The native approach is intended mainly for development, experimenting with the parameters and tunning the application performance. For the real deployments, the docker compose is suggested. 

### Native deployment

To run the network applications, three instances of Relay are required, alongside with the era_5g_slam_commander and the Cartographer itself:

1. Run the Realy servers: TBA
2. Run the era_5g_slam_commander
  1. `ros2 run era_5g_slam_commander map_creator -o ~/tmp/ --launch-mapping mapping.launch.py --launch-localization localization.launch.py`
3. The map_creator requires three parameters:
  1. Path to the folder for saving the map (-o)
  2. Name of the mapping launch file (--launch-mapping)
  3. Name of the localization launch file (--launch-localization)

### Docker compose deployment

To deploy the network application with the docker compose, you need to create two docker-compose files. One for the robot and the other one for the remote computer. The folders `docker/client` and `docker/server` contains example compose files for deplyoment of the network application's server and client on the same compter in two separated ROS environments. These files can be used as a base for your own configuration. To adapt the configuration, these are the main changes required.

For server part: 

1. Cartographer service
  1. Path to the shared volume, where the maps are stored.
2. Relay services
  1. The numbers of ports exposed for the client part for each relay server.

For client part:

1. Addresses of relay servers according to their location and port number specified in the server compose file.
2. The ROS_DOMAIN_ID of all relay clients according to your robot configuration.

For both parts:

1. The configuration of the topics and TFs acording to the robot configuration.

### Default configuration

This is the default configuration provided for the Summit XL robot in the reference docker compose files.

Topics from robot to cloud:

* 3D Laser
  * Name: /robot/top_laser/point_cloud
  * Type: sensor_msgs/msg/PointCloud2
  * Compression: Draco
* IMU
  * Name: /robot/imu/data
  * Type: sensor_msgs/msg/Imu
* Odometry
  * Name: /robot/odometry/filtered
  * Type: nav_msgs/msg/Odometry

Topics from cloud to robot:

* Map
  * Name: /map
  * Type: nav_msgs/msg/OccupancyGrid

Services:

* Start mapping
  * Name: /start_mapping
  * Type: std_srvs/srv/Trigger

* Finish mapping
  * Name: /finish_mapping
  * Type: era_5g_interfaces/srv/TriggerMap

* Start localization
  * Name: /start_localization
  * Type: era_5g_interfaces/srv/TriggerMap

* Stop localization
  * Name: /stop_localization
  * Type: std_srvs/srv/Trigger


Transformations from robot to cloud:

* Odometry
  * Source: robot/base_footprint
  * Target: robot/odom
* IMU
  * Source: robot/imu_link
  * Target: robot/base_footprint
* Laser
  * Source: robot/top_laser_link
  * Target: robot/base_footprint

Transformations from cloud to robot:

* Map
  * Source: robot/odom
  * Target: map




## Contributing

Contributions are welcome! If you find any issues or have suggestions for improvements, please open an issue or submit a pull request.

## Build Docker

This is only necessary if you need to build your own, custom version of the network application. Otherwise, use the docker images publicly available on the dockerhub. 

To build the Cartographer network application docker images, follow these steps:

1. Clone the repository: `git clone git@github.com:5G-ERA/slam_network_application.git`
2. Enter the package folder: `cd slam_network_application/era_5g_slam`
3. Build the docker image: `docker build -f docker/cartographer/Dockerfile -t but5gera/era_5g_slam:VERSION .`

Since the netwok application uses the 5G-ERA Relay and custom ROS messages, custom build of the Relay client and server is required (this will be addressed in the futurue). To build them, follow these steps:

1. Enter the package folder: `cd slam_network_application/era_5g_slam`
2. Build the Relay server docker image: `docker build -f docker/server/Dockerfile -t but5gera/ros2_relay_server_slam:VERSION .`
3. Build the Relay client docker image: `docker build -f docker/client/Dockerfile -t but5gera/ros2_relay_client_slam:VERSION .`

## License

This project is licensed under the [Apache License](LICENSE).
