FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rqt-robot-steering \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

COPY . ./src

SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]