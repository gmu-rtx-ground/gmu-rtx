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
RUN apt update && \
    apt install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    usermod --append --groups video ${USERNAME}

# Set environment variables
ENV QT_X11_NO-MITSHM=1 \
    CPLUS_INCLUDE_PATH=/opt/ros/noetic/include \
    LD_LIBRARY_PATH=/opt/ros/noetic/lib \
    CATKIN_WS=/home/${USERNAME}/catkin_ws \
    USR_HOME=/home/${USERNAME} \
    GMU_RTX=/home/${USERNAME}/gmu-rtx

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "${USR_HOME}/.bashrc"

FROM ros-noetic-base AS dependencies-realsense2

# Copy apt package manifest into container
COPY ./docker/apt-arm-realsense.txt /tmp/apt-arm-realsense.txt

# Make Ubuntu up-to-date including the latest stable kernel
RUN apt update && \
    apt upgrade -y --no-install-recommends && \
    apt dist-upgrade -y --no-install-recommends && \
    xargs -a /tmp/apt-arm-realsense.txt apt install --no-install-recommends -y && \
    apt autoremove --purge -y && \
    apt autoclean && \
    rm -rf /var/lib/apt/lists/*

# Clone RealSense repo and run initial dependency scripts
RUN git clone https://github.com/IntelRealSense/librealsense.git ${USR_HOME}/librealsense

# Run Intel Realsense permissions script from librealsense2 root directory
WORKDIR ${USR_HOME}/librealsense
RUN /bin/bash -c "./scripts/setup_udev_rules.sh"

# Build RealSense library from source
RUN mkdir -p ${USR_HOME}/librealsense/build
WORKDIR ${USR_HOME}/librealsense/build

# Run cmake configure step
RUN cmake ../.

# Recompile and install librealsense2 binaries
RUN make uninstall -j$(nproc)
RUN make clean -j$(nproc)
RUN make -j$(nproc)
RUN make install -j$(nproc)

