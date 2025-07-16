# Start from ROS Noetic Perception
FROM ros:noetic-perception

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Update and install dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        gazebo11 \
        ros-noetic-gazebo-ros-pkgs \
        ros-noetic-navigation \
        ros-noetic-rviz \
        ros-noetic-tf \
        ros-noetic-map-server \
        ros-noetic-amcl \
        ros-noetic-move-base \
        wget \
        git \
        python3-pip \
        lsb-release \
        sudo && \
    rm -rf /var/lib/apt/lists/*

# Optionally install additional Python tools (e.g. colcon, etc.)
RUN pip3 install -U rosdep

# Initialize rosdep
RUN rosdep update

# Create a workspace (optional but recommended)
RUN mkdir -p /workspace/src

# Set working directory
WORKDIR /workspace

# Source ROS and workspace setup when a container is launched
SHELL ["/bin/bash", "-c"]

# Build workspace if you’re going to add custom packages
# Uncomment these if adding your own packages later:
# RUN cd /workspace && \
#     catkin_make

# Source ROS and workspace on start
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
