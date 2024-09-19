FROM ros:humble

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Prague
ENV LANG C.UTF-8

RUN apt-get update \
    && apt-get install -y python3-pip ros-humble-cartographer-ros ros-humble-point-cloud-transport-plugins ros-humble-octomap-server ros-humble-nav2-map-server
    
#RUN apt-get update \
#    && apt-get install -y python3-pip ros-foxy-cartographer-ros

RUN mkdir -p /root/map_data/

COPY docker/cartographer_localization_theron/start.sh /root/start.sh

COPY docker/cartographer_localization_theron/map/map.pbstream /root/map_data/map.pbstream
COPY docker/cartographer_localization_theron/map/map.yaml /root/map_data/map.yaml
COPY docker/cartographer_localization_theron/map/map.pgm /root/map_data/map.pgm

COPY docker/cartographer_localization/DEFAULT_FASTRTPS_PROFILES.xml /root/DEFAULT_FASTRTPS_PROFILES.xml

RUN mkdir -p /root/slam_ws/src 

COPY era_5g_cartographer /root/slam_ws/src/era_5g_cartographer
COPY era_5g_slam_commander /root/slam_ws/src/era_5g_slam_commander
COPY era_5g_interfaces /root/slam_ws/src/era_5g_interfaces

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /root/slam_ws && colcon build"

ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/DEFAULT_FASTRTPS_PROFILES.xml

RUN chmod +x /root/start.sh
ENTRYPOINT [ "/root/start.sh" ] 
