# Introduction to ROS2 on RPi 5 🖱

<p align="center">
    <br>
    <img src="etc/gif/ROS2.gif" alt="GIF Image" width="300"/>
</p>

This repository acts as a guide for installing and configuring ROS2 on a Raspberry Pi 5 device running the Ubuntu 22.04+ LTS operating system.

## Setup

The setup.sh script installs ROS2 and all of its required dependencies onto the local Raspberry Pi 5 device.

``` bash
# From the ROS2 directory, migrate to the setup script
cd "config"

# Ensure that the setup.sh script is executable then execute it
chmod +rwx setup.sh
./setup.sh
```