# Use the official ROS Noetic base image
FROM ros:noetic

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

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################

ENV QT_X11_NO-MITSHM=1

# Environment Variables for ROS
ENV CPLUS_INCLUDE_PATH=/opt/ros/noetic/include
ENV LD_LIBRARY_PATH=/opt/ros/noetic/lib

# Install system dependencies and useful tools
RUN sudo apt-get update && sudo apt-get install -y \
    python3-pip \
    python3-venv \
    build-essential \
    curl \
    nano \
    htop \
    net-tools \
    iputils-ping \
    unzip \
    ros-noetic-common-tutorials \
    ros-noetic-geometry-tutorials \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-rqt-graph \
    ros-noetic-roscpp \
    ros-noetic-rosserial \
    ros-noetic-rospy \
    ros-noetic-rospy-tutorials 


# Switch back to the root user to install the Arduino CLI
USER root

# Create a directory for the Arduino CLI binary
RUN mkdir -p /usr/local/bin

# Check if the binary exists
RUN ls -lah /usr/local/bin

# Install Arduino CLI
# RUN curl to download the latest version of the Arduino CLI binary and extract to the /usr/local/bin directory
RUN curl -fsSL --retry 5 --fail https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Linux_64bit.tar.gz -o /home/${USERNAME}/arduino-cli.tar.gz  && \
        ls -lah /home/${USERNAME}/arduino-cli.tar.gz && \
        tar -xzf /home/${USERNAME}/arduino-cli.tar.gz -C /usr/local/bin && \
        rm -rf /home/${USERNAME}/arduino-cli.tar.gz

# Check if the binary exists
RUN ls -lah /usr/local/bin/arduino-cli

# Make the Arduino CLI binary executable
RUN chmod +x /usr/local/bin/arduino-cli

# Switch back to the non-root user
USER $USERNAME

# Configure Arduino CLI (creates required directories)
RUN arduino-cli config init

# Install common Arduino cores (for example, AVR for Arduino Uno)
RUN arduino-cli core update-index && \
    arduino-cli core install arduino:avr

# Install example Arduino libraries
RUN arduino-cli lib install "Adafruit GFX Library" "Adafruit BusIO"

# Upgrade pip
RUN pip3 install --upgrade pip

# Install lightweight Python libraries for robotics
RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    transforms3d \
    pyquaternion \
    pyserial \
    opencv-python \
    rospkg \
    catkin-tools \
    networkx \
    paho-mqtt \
    pymavlink

# Mount common Ubuntu Linux paths as volumes
VOLUME ["/home", "/mnt", "/opt"]
