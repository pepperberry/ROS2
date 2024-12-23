#!/bin/bash

'''
@Author Ezekiel A. Mitchell
@Date December 21, 2024
@Orgnization Endr, Seattle University

This script contains the following executables for install ROS2 Jazzy on a Raspberry Pi 5 with Ubuntu 22.04 LTS OS.

'''
# Stop on any error
set -e
echo "Starting ROS 2 Jazzy installation on Raspberry Pi with Ubuntu 22.04 LTS..."

# 1. Update and upgrade system packages
echo "Updating and upgrading system packages..."
sudo apt update && sudo apt upgrade -y

# 2. Add ROS 2 Jazzy repository and keys
echo "Adding ROS 2 repository..."
sudo apt install -y software-properties-common curl gnupg
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo apt-add-repository universe
sudo apt-add-repository restricted
sudo apt-add-repository multiverse

# Add the ROS 2 apt repository
sudo sh -c 'echo "deb [arch=arm64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# 3. Install ROS 2 Jazzy
echo "Installing ROS 2 Jazzy..."
sudo apt update
sudo apt install -y ros-jazzy-desktop

# 4. Source the ROS 2 setup script
echo "Configuring ROS 2 environment..."
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 5. Install additional ROS 2 tools and dependencies
echo "Installing additional ROS 2 tools..."
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-argcomplete build-essential git

# 6. Initialize rosdep
echo "Initializing rosdep..."
sudo rosdep init
rosdep update

# 7. Install ROS 2 build tools (optional, for building from source)
echo "Installing build tools for ROS 2..."
sudo apt install -y python3-vcstool

# 8. Optional: Install RMW implementation for DDS (Cyclone DDS)
echo "Installing Cyclone DDS RMW implementation..."
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp

# 9. Set up permissions for GPIO, I2C, and other hardware interfaces (for Raspberry Pi)
echo "Setting up hardware interface permissions..."
sudo usermod -aG dialout $USER
sudo groupadd gpio
sudo usermod -aG gpio $USER
sudo groupadd i2c
sudo usermod -aG i2c $USER

# 10. Reboot to apply all changes
echo "Installation complete. Rebooting to apply changes..."
sudo reboot