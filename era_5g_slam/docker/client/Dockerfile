FROM but5gera/ros2_relay_client:1.2.2

RUN mkdir -p /root/slam_ws/src 
COPY era_5g_interfaces /root/slam_ws/src/era_5g_interfaces

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /root/slam_ws && colcon build"

COPY docker/client/start.sh /root/start.sh
RUN chmod +x /root/start.sh