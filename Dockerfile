# Start with the official ROS Humble ROS-Core image based on Ubuntu Jammy
FROM ros:humble-ros-core-jammy

SHELL ["/bin/bash", "-c"]

# Install basic build tools and ROS2 development dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Set the ROS distro to Humble
ENV ROS_DISTRO=humble

# Initialize rosdep and update for the current ROS distro
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# Add and update colcon mixins and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# Install ROS2 base packages specific to Humble distribution
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# Set environment variables and source ROS2 Humble setup
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# Source the ROS2 environment automatically when a new shell is created
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /root/hogsmeade/install/setup.bash" >> ~/.bashrc

COPY . /root/hogsmeade/src

RUN mkdir -p ~/hogsmeade/src

WORKDIR /root/hogsmeade

RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && cd ~/hogsmeade/src \
    && rosdep update --include-eol-distros \
    && rosdep install --from-paths . -i -y --include-eol-distros

RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && cd ~/hogsmeade \
    && colcon build

EXPOSE 4567

# Copy entrypoint script
COPY ./entrypoint.sh /entrypoint.sh

# Make entrypoint.sh executable
RUN chmod +x /entrypoint.sh

# Run entrypoint.sh on start
CMD ["/bin/bash", "/entrypoint.sh"]