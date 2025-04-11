#!/bin/bash

# Ensure submodules are properly initialized
git submodule update --init --recursive

{
  [[ -n "$CPLUS_INCLUDE_PATH" ]] || echo "export CPLUS_INCLUDE_PATH=${CPLUS_INCLUDE_PATH:-/opt/ros/noetic/include}"
  [[ -n "$LD_LIBRARY_PATH" ]] || echo "export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-/opt/ros/noetic/lib}"
  [[ -n "$CATKIN_WS" ]] || echo "export CATKIN_WS=${CATKIN_WS:-${HOME}/catkin_ws}"  
} >> "${HOME}/.bashrc"

if [[ ":$PATH:" != *":$HOME/.local/bin:"* ]]; then
  echo "export PATH=${HOME}/.local/bin:\$PATH" >> "${HOME}/.bashrc"
fi


grep -qxF "source ${CATKIN_WS}/devel/setup.bash" "${HOME}/.bashrc" || \
  echo "source ${CATKIN_WS}/devel/setup.bash" >> "${HOME}/.bashrc"

source "${HOME}/.bashrc"

# Update and install packages
export DEBIAN_FRONTEND=noninteractive
sudo apt update
sudo apt upgrade -y
sudo apt install -y $(< ./apt-packages.txt)
sudo apt autoremove --purge -y
sudo apt autoclean

# Install Arduino dependencies
sudo mkdir -p /usr/local/bin

if ! command -v arduino-cli > /dev/null 2>&1; then
  echo "arduino-cli is not installed. Downloading from source..."
  curl -fsSL --retry 5 --fail https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Linux_ARM64.tar.gz \
          -o "${HOME}/arduino-cli.tar.gz" && \
          sudo tar -xzf "${HOME}/arduino-cli.tar.gz" -C "/usr/local/bin" && \
          rm -rf "${HOME}/arduino-cli.tar.gz" && \
          sudo chmod +x /usr/local/bin/arduino-cli
fi

if [ ! -f "${HOME}/.arduino15/arduino-cli.yaml" ]; then
  arduino-cli config init
else
  echo "arduino-cli configuration already exists; skipping initialization."
fi

arduino-cli core update-index
arduino-cli core install arduino:avr

ARDUINO_LIB="${HOME}/Arduino/libraries"
ROS_LIB="${ARDUINO_LIB}/ros_lib"

source "/opt/ros/${ROS_DISTRO:-noetic}/setup.bash"

if [ ! -d "${ROS_LIB}" ]; then
  rosrun rosserial_arduino make_libraries.py "${ARDUINO_LIB}"
else
  echo "ros_lib already exists; reinstalling with current Arduino core..."
  rm -rf "${ROS_LIB}" && \
  rosrun rosserial_arduino make_libraries.py "${ARDUINO_LIB}"
fi

# Install Python libraries

pip3 install --upgrade pip --no-cache-dir
pip3 install --no-cache-dir -r ./requirements.txt

# Add udev rules for modules from gmu-rtx repo
sudo cp ~/gmu-rtx/ROS/udev_rules/* /etc/udev/rules.d/ && \
sudo udevadm control --reload-rules && \
sudo udevadm trigger;

# Create catkin workspace
mkdir -p "${CATKIN_WS}/src"

# Check for the CATKIN_WS env var
if [ -z "${CATKIN_WS}" ]; then
  echo "CATKIN_WS is not defined. Exiting."
  exit 1
fi

# Create sym links for ROS packages
ROS_SOURCE_DIR="${HOME}/gmu-rtx/ROS"
DESTINATION_DIR="${CATKIN_WS}/src"

# Exclusion Directories
EXCLUDE_DIRS=(
  "Roscordings"
  "udev_rules"
)

# Check if the source directory exists
if [ ! -d "${ROS_SOURCE_DIR}" ]; then
  echo "ROS source directory does not exist. Exiting."
  exit 1
fi

# Check if the destination directory already exists
mkdir -p "${DESTINATION_DIR}"

# Function to check if a directory is in the exclusion list
is_excluded() {
  local dir="$1"
  for exclude in "${EXCLUDE_DIRS[@]}"; do
    if [[ "$dir" == *"$exclude"* ]]; then
      return 0
    fi
  done
  return 1
}

# Process each directory in the repo source
for dir in "$ROS_SOURCE_DIR"/*; do
  # Get the basename
  base=$(basename "$dir")
  
  # Skip if in exclusion list
  if is_excluded "$base"; then
    echo "Skipping excluded dir: $base"
    continue
  fi
  
  # Destination path for the symlink
  dest_link="$DESTINATION_DIR/$base"
  
  # Create symlink if it doesn't already exist
  if [ ! -e "$dest_link" ]; then
    ln -s "$dir" "$dest_link"
    echo "Created symlink: $dest_link -> $dir"
  else
    echo "Symlink already exists: $dest_link"
  fi
done

# Build catkin packages
cd "${CATKIN_WS}"

rosdep update
rosdep install --from-paths src --ignore-src  -r -y
caktin clean -y
catkin build
catkin test

cd "${HOME}/gmu-rtx"
