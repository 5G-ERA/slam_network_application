# Cartographer container

This repository contains usage a ROS container with Cartographer installed.

## Containers

The following containers are available:

* `cartographer` - ROS Melodic with Cartographer installed
* `octomap` - ROS Melodic with Octomap installed

## Usage

### Run

This repository contains a demo compose file to run 3D slam and mapping.

1. 3D Slam
2. 3D Localization

#### 3D Slam

To run the demo:

1. Download bag file from [google drive](https://drive.google.com/drive/folders/1_RNQyp2eXQUuRVr_x7F_oRjfgFA6VsNA?usp=share_link) into `test/docker/bagfiles` folder.

2. Update bag file service to use the downloaded bag file. Inside service `rosbag` change volume `source` parameter  to the right  bag file by the default`./bags/filtered.bag` 

3. Run the following command:

4. ```bash
   docker compose -f test/docker/localization-octomap.yaml up
   ```

5. Wait until all services are up and running.

6. Open web browser and go to [http://localhost:8080](http://127.0.0.1:8080), you should see a desktop.

7. Wait till the ros bag is finished to play

8. Save the 3d map for cartographer:
   
   ```bash
   docker exec docker-cartographer-1 \
   /ros_entrypoint.sh \
   rosservice call /robot/finish_trajectory "trajectory_id: 0"
   docker exec docker-cartographer-1 \
   /ros_entrypoint.sh \
   rosservice call /robot/write_state \
   "{filename: '/home/robot/new-map.pbstream', include_unfinished_submaps: true}"
   ```

9. Save the octomap:
   
   ```bash
   docker exec docker-octomap-1 \
   /ros_entrypoint.sh \
   rosrun map_server map_saver -f mymap map:=/robot/projected_map
   docker exec docker-octomap-1 \
   /ros_entrypoint.sh \
   rosrun octomap_server octomap_saver \
    -f /home/robot/octomap/mymap_octomap.bt \
   octomap_full:=/robot/octomap_full
   ```

#### 3D Localization

To run the demo:

1. Download bagfile from [google drive](https://drive.google.com/file/d/1SAOnLN9dUt3Se3pcwkREC29W8kW2GZPU/view?usp=sharing) into respectives folders.

2. Update bag file service to use the downloaded bag file. Inside service `rosbag` change volume `source` parameter to the right bag file by the default`./bags/filtered.bag`

3. Change environment variable `LOAD_STATE_FILENAME` in `test/docker/slam.yaml` file, service `cartographer` to the path of the downloaded pbstream.

4. Run the following command:
   
   ```bash
   docker compose -f test/docker/slam.yaml up
   ```

5. Wait until all services are up and running.

6. Open web browser and go to [http://localhost:8080](http://127.0.0.1:8080), you should see a desktop.

7. Wait till the ros bag is finished to play

8. Save the octomap (better performance than in slam):
   
   ```bash
   docker exec docker-octomap-1 \
   /ros_entrypoint.sh \
   rosrun map_server map_saver \
   -f /home/robot/octomap/mymap_octomap \
   map:=/robot/projected_map
   docker exec docker-octomap-1 \
   /ros_entrypoint.sh \
   rosrun octomap_server octomap_saver \
   -f /home/robot/octomap/mymap_octomap.bt \
   octomap_full:=/robot/octomap_full
   ```

## Code of Conduct

This project has adopted the [Robotnik Code of Conduct](https://to.do/code_of_conduct_faq). For more information see the [Code of Conduct FAQ](https://to.do/code_of_conduct_faq) or contact [opencode@robotnik.es](opencode@robotnik.es) with any additional questions or comments.

## License

Copyright (c) 2023, Robotnik Automation S.L.L. All rights reserved.

Licensed under the [BSD 2-Clause](./LICENSE) License.