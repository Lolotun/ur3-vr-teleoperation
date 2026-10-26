FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-colcon-common-extensions \
    python3-yaml \
    python3-pip \
    ros-humble-geometry-msgs \
    ros-humble-moveit-servo \
    ros-humble-moveit-kinematics \
    ros-humble-rcl-interfaces \
    ros-humble-robot-state-publisher \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-std-srvs \
    ros-humble-tf2-ros \
    ros-humble-trajectory-msgs \
    ros-humble-ur-description \
    ros-humble-ur-robot-driver \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/teleop-requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/teleop-requirements.txt

# Revision read from the last working local workshop image.
ARG ROS_TCP_ENDPOINT_REV=05a3b25a11d9639549ae128e471bd696430b91f7
WORKDIR /ws
RUN mkdir -p src && \
    git clone -b main-ros2 https://github.com/leggedrobotics/ROS-TCP-Endpoint.git src/ROS-TCP-Endpoint && \
    git -C src/ROS-TCP-Endpoint checkout --detach ${ROS_TCP_ENDPOINT_REV}

COPY ros2_ws/src/teleop_core src/teleop_core
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENV ROS_DOMAIN_ID=11
ENV ROS_LOCALHOST_ONLY=1
EXPOSE 10000
ENTRYPOINT ["/entrypoint.sh"]
CMD ["wifi_speedl"]
