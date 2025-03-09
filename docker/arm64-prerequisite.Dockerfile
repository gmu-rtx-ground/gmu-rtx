# Use the official ROS Noetic base image
FROM ros:noetic AS ros-noetic-base

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

FROM ros-noetic-base AS dependencies-realsense2

# Make Ubuntu up-to-date including the latest stable kernel
RUN sudo apt-get update && \
    sudo apt-get upgrade && \
    sudo apt-get dist-upgrade

# Install Install the core packages required to build 
# librealsense binaries and the affected kernel modules
RUN sudo apt-get install -y \
        libssl-dev \
        libusb-1.0-0-dev \
        libudev-dev \
        pkg-config \
        libgtk-3-dev
        
# Install build tools
RUN sudo apt-get install -y \
        git \
        wget \
        cmake \
        build-essential

# Prepare Linux Backend and the Dev. Environment
RUN sudo apt-get install -y \
        libglfw3-dev \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        at \
        libudev-dev

# Install additional dependencies
RUN sudo apt-get install -y \
        v4l-utils \
        udev

# Clone RealSense repo and run initial dependency scripts
RUN git clone https://github.com/IntelRealSense/librealsense.git /home/${USERNAME}/librealsense

# Run Intel Realsense permissions script from librealsense2 root directory
WORKDIR /home/${USERNAME}/librealsense
RUN /bin/bash -c "./scripts/setup_udev_rules.sh"

# Build RealSense library from source
RUN mkdir /home/${USERNAME}/librealsense/build
WORKDIR /home/${USERNAME}/librealsense/build

# Run cmake configure step
RUN cmake ../.

# Recompile and install librealsense2 binaries
RUN sudo make uninstall -j$(($(nproc)-1))
RUN make clean -j$(($(nproc)-1))
RUN make -j$(($(nproc)-1))
RUN sudo make install -j$(($(nproc)-1))
