# Basis: Offizielles ROS 2 Humble Image
FROM ros:humble-ros-base

# 1. System-Tools & Build-Tools installieren
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

# 2. Rosdep initialisieren
RUN rosdep init && rosdep update

# 3. Arbeitsverzeichnis setzen
WORKDIR /root/ros2_ws

# 4. Entrypoint Skript (lädt ROS Umgebung automatisch)
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
