# syntax=docker/dockerfile:1.7-labs
FROM ros:humble

SHELL ["/bin/bash", "-c"]

# Arguments
ARG FOLDER_NAME=rc-brain
ARG WORKSPACE_DIR=/home/cev

# Set working directory
WORKDIR $WORKSPACE_DIR

# Source ROS2 environment automatically in every shell
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source $WORKSPACE_DIR/install/setup.bash" >> ~/.bashrc

# Install dependencies
RUN apt-get update && apt-get install -y \
    curl \
    python3-colcon-common-extensions \
    ros-$ROS_DISTRO-ros-base \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install clang-format
RUN curl -fsSL https://apt.llvm.org/llvm-snapshot.gpg.key | tee /etc/apt/trusted.gpg.d/apt.llvm.org.asc && \
    echo $'deb http://apt.llvm.org/jammy/ llvm-toolchain-jammy main\n' \
         $'deb http://apt.llvm.org/jammy/ llvm-toolchain-jammy-18 main\n' \
         $'deb http://apt.llvm.org/jammy/ llvm-toolchain-jammy-19 main\n' \
    | tee -a /etc/apt/sources.list && \
    apt-get update && apt-get install -y clang-format clang-format-19 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Fix libc-bin issue
RUN rm /var/lib/dpkg/info/libc-bin.* || true && apt-get update && apt-get install -y libc-bin

# Initialize rosdep
RUN rosdep update --rosdistro $ROS_DISTRO

# Create workspace directory
RUN mkdir -p src/$FOLDER_NAME

# Copy the full package source code
COPY --parents ./* src/$FOLDER_NAME/
RUN chmod +x src/$FOLDER_NAME/install.sh && src/$FOLDER_NAME/install.sh

# Install package dependencies
RUN source /opt/ros/$ROS_DISTRO/setup.bash && rosdep install --from-paths src -r -y

# Build the ROS2 workspace
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install

# Set up entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
