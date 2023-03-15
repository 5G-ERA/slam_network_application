# SLAM Netapp

Central cloud or edge Map Creator, Map Server and 3D localization

## ROS1 GRAPH-SLAM Container

### SLAM mode

#### Inputs

- TF tree

- Topics:
  
  - imu ([sensors_msgs/Imu](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html))
  
  - pointcloud ([draco_point_cloud_transport/CompressedPointCloud2](https://github.com/paplhjak/draco_point_cloud_transport/blob/master/msg/CompressedPointCloud2.msg))
  
  - gps ([sensor_msgs/NavSatFix](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatFix.html))
  
  - odom ([nav_msgs/Odometry](https://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html))

- services:
  
  - stop_mapping_service (WIP)

#### outputs

- files:
  
  - 3d map: pbstream map
  
  - 2d map: yaml + pgm

- topics:
  
  - asset_writer_progress

#### Configuration

### Localization mode

#### Inputs

- files:
  
  - 3d map: pbstream map
  
  - 2d map: yaml + pgm

- Topics:
  
  - imu ([sensors_msgs/Imu](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html))
  
  - pointcloud ([draco_point_cloud_transport/CompressedPointCloud2](https://github.com/paplhjak/draco_point_cloud_transport/blob/master/msg/CompressedPointCloud2.msg))
  
  - gps ([sensor_msgs/NavSatFix](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatFix.html))
  
  - odom ([nav_msgs/Odometry](https://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html))

- services:
  
  - Init_pose

#### Outputs

- TF tree (between odom and map)

- topic:
  
  - reliable_localization

#### Configuration
