# 1. Mimari bağımsız temel imaj kullanımı
# Bu imaj hem x86 hem de arm64 (Jetson) destekler
FROM ros:humble-ros-base

# 2. Çevresel değişkeni sabitleyelim
ENV ROS_DISTRO=humble

# 3. Sistem paketlerini yükle
# 'ros-humble-desktop' yerine sadece ihtiyacın olanları ekleyerek imajı hafifletiyoruz
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

# 4. Çalışma alanı
WORKDIR /ros2_ws

# 5. Kodları kopyala
COPY . ./src

# 6. Derleme (Build)
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install

# 7. Otomatik source ayarları
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]