# Cartographer Network Application

This repository contains the configuration and instructions for setting up the Cartographer network application. It provides a step-by-step guide on how to build Docker images and deploy the application.

## Table of Contents

- [Cartographer Network Application](#cartographer-network-application)
  - [Table of Contents](#table-of-contents)
  - [Build](#build)
  - [Configuration](#configuration)
    - [Default configuration](#default-configuration)
  - [Usage](#usage)
  - [Contributing](#contributing)
  - [License](#license)

## Build

To build the Cartographer network application, follow these steps:

1. Clone the repository: `git clone git@github.com:5G-ERA/slam_network_application.git`
2. Enter the package folder: `cd slam_network_application/era_5g_cartographer_ros2`
3. Build the docker image: `docker build -f docker/cartographer/Dockerfile -t but5gera/era_5g_slam:0.1.0 .`

## Configuration

The Cartographer network application can be configured by modifying the `docker/cartographer/cartographer.lua` file. This file contains various settings such as the odometry and tracking frames and SLAM finetuning parameters. The SLAM Cartographer application requires the 3D laser data (e.g. from Velodyne LIDAR), the robot's odometry and IMU measurement. Besides, it requires a set of TF's. 


### Default configuration

Topics:

* Laser
  * Name: /robot/top_laser/point_cloud
  * Type: sensor_msgs/msg/PointCloud2
* IMU
  * Name: /robot/imu/data
  * Type: sensor_msgs/msg/Imu
* Odometry
  * Name: /robot/odometry/filtered
  * Type: nav_msgs/msg/Odometry

Transformations:

* Odometry
  * Source: robot/base_footprint
  * Target: robot/odom
* IMU
  * Source: robot/imu_link
  * Target: robot/base_footprint
* Laser
  * Source: robot/top_laser_link
  * Target: robot/base_footprint


## Usage

To use the Cartographer network application, follow these steps:

1. Start the application: `docker run --rm -e ROS_DOMAIN_ID=32 but5gera/era_5g_slam:0.1.0`
2. You can adjust the ROS_DOMAIN_ID according to your robot
3. Optionally, you can run the container in host network mode: `docker run --rm -e ROS_DOMAIN_ID=32 --network host but5gera/era_5g_slam:0.1.0`

## Contributing

Contributions are welcome! If you find any issues or have suggestions for improvements, please open an issue or submit a pull request.

## License

This project is licensed under the [Apache License](LICENSE).
