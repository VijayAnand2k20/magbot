# Stage 1: Build environment
FROM ros:humble AS builder

# Set shell
SHELL ["/bin/bash", "-c"]

# Set working directory
WORKDIR /ws

# Install system dependencies (shared between build and runtime)
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-xacro \
    ros-humble-joy \
    python3-transforms3d \
    ros-humble-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install numpy transforms3d evdev

# Copy source code
COPY . .

RUN mkdir -p /root/.gazebo/models/magbot_gazebo/description
COPY src/magbot_gazebo/description/urdf/model* /root/.gazebo/models/magbot_gazebo
COPY src/magbot_gazebo/description/meshes /root/.gazebo/models/magbot_gazebo/description/meshes


# Build ROS2 packages
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select \
    interfaces_pkg \
    magbot_control \
    magbot_input_interfacing \
    magbot_peripheral_interfacing \
    magbot_servo_interfacing \
    magbot_utilities \
    magbot \
    magbot_gazebo

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    build-essential \
    gcc \
    python3-dev

# Install Python dependencies
RUN pip3 install numpy transforms3d evdev


# Set working directory
WORKDIR /ws

# Create entrypoint script
RUN printf '#!/bin/bash\nsource /opt/ros/humble/setup.bash\nsource /ws/install/setup.bash\nexec "$@"\n' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "magbot", "magbot.launch.py"]
