# Use the ROS Humble desktop image as a base
FROM osrf/ros:humble-desktop

# Arguments for creating a non-root user
# ARG USERNAME=ubuntu
# ARG USER_UID=1000
# ARG USER_GID=$USER_UID

# # Create non-root user and add sudo privileges if they don't exist
# RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
#         groupadd --gid $USER_GID $USERNAME && \
#         useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
#     fi && \
#     apt-get update && \
#     apt-get install -y sudo && \
#     echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
#     chmod 0440 /etc/sudoers.d/$USERNAME

# # Add user to video group for webcam access
# RUN usermod -aG video $USERNAME

# Update all packages and install necessary tools
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
        git \
        python3-pip \
        joystick \
        jstest-gtk \
        htop \
        evtest && \
    pip3 install \
        transforms3d \
        Adafruit-Blinka \
        adafruit-circuitpython-servokit \
        adafruit-circuitpython-bno055 \
        RPi.GPIO

# Install ROS and Gazebo dependencies
RUN apt-get install -y \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-gazebo-ros2-control \
        python3-colcon-common-extensions \
        ros-humble-xacro

# Download and install Gazebo
RUN curl -sSL http://get.gazebosim.org | sh

# Initialize rosdep and update
# Initialize rosdep and update (with a check for existing sources list file)
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update

# Source the ROS environment in all shells
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

# Switch to the non-root user for future commands
USER $USERNAME

# Set the working directory
WORKDIR /magbot_ws

# Set ROS environment variables
ENV ROS_DISTRO=humble

# Build command entry point (useful for interactive shells)
CMD ["/bin/bash"]