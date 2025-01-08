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

# Stage 2: Runtime environment
FROM ros:humble-ros-core

# Copy system dependencies from builder stage
COPY --from=builder /var/lib/apt/lists /var/lib/apt/lists

# Install runtime dependencies (minimal setup)
RUN apt-get update && apt-get install -y --no-install-recommends \
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

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    gcc \
    python3-dev

# Install Python dependencies
RUN pip3 install numpy transforms3d evdev

# Copy built packages from builder
COPY --from=builder /ws/install /ws/install

# Set working directory
WORKDIR /ws

# Create entrypoint script
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
source /ws/install/setup.bash\n\
exec "$@"' > /entrypoint.sh && \
chmod +x /entrypoint.sh

# Set entrypoint and default command
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "magbot", "magbot_service.launch.py"]
