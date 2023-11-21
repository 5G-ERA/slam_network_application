FROM ros:humble

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Prague
ENV LANG C.UTF-8

RUN apt-get update \
    && apt-get install -y python3-pip ros-humble-cartographer-ros

COPY docker/cartographer/start.sh /root/start.sh
COPY docker/cartographer/DEFAULT_FASTRTPS_PROFILES.xml /root/DEFAULT_FASTRTPS_PROFILES.xml
COPY docker/cartographer/cartographer.lua /root/cartographer.lua
COPY docker/cartographer/cartographer.launch /root/cartographer.launch

ENV FASTRTPS_DEFAULT_PROFILES_FILE=/root/DEFAULT_FASTRTPS_PROFILES.xml

RUN chmod +x /root/start.sh
ENTRYPOINT [ "/root/start.sh" ] 