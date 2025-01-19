# Specify the base image
FROM osrf/ros:humble-desktop-full

# Avoid user interaction with tzdata
ENV DEBIAN_FRONTEND=noninteractive

# Setup the user to match your host's UID and GID to manage file permissions
ARG UNAME=user
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} ${UNAME} \
    && useradd -m -u ${UID} -g ${GID} -s /bin/bash ${UNAME} \
    && usermod -aG sudo ${UNAME} \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Switch to the user
USER ${UNAME}

# Set the environment variable to non-interactive (again, just to make sure)
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary tools
RUN sudo apt-get update && sudo apt-get install -y wget gnupg2 lsb-release

# Install Gazebo Fortress
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN sudo apt-get update
RUN sudo apt-get install ignition-fortress -y

# Install dependencies
RUN sudo apt-get update && \
    sudo apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-xacro \
    && sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

# Setup the workspace
ENV ROS2_WS=/home/${UNAME}/ros2_ws
RUN mkdir -p ${ROS2_WS}/src

# Source ROS setup on every new shell
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/${UNAME}/.bashrc

# Copy the entrypoint script
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]