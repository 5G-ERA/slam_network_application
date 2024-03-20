FROM ros:humble

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Prague
ENV LANG C.UTF-8

RUN apt-get update \
    && apt-get install -y python3-pip ros-humble-cartographer-ros ros-humble-point-cloud-transport-plugins ros-humble-octomap-server ros-humble-nav2-map-server

COPY docker/cartographer/start.sh /root/start.sh
COPY docker/cartographer/DEFAULT_FASTRTPS_PROFILES.xml /root/DEFAULT_FASTRTPS_PROFILES.xml

RUN mkdir -p /root/slam_ws/src 
RUN mkdir /root/shared_dir

COPY era_5g_cartographer /root/slam_ws/src/era_5g_cartographer
COPY era_5g_slam_commander /root/slam_ws/src/era_5g_slam_commander
COPY era_5g_interfaces /root/slam_ws/src/era_5g_interfaces

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /root/slam_ws && colcon build"

ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/DEFAULT_FASTRTPS_PROFILES.xml

RUN chmod +x /root/start.sh
ENTRYPOINT [ "/root/start.sh" ] 