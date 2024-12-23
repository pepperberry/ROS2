#!/bin/bash

# Path to your virtual environment
VENV_PATH="/home/test/ROS2/config/ros2_venv"

# Activate the virtual environment
source "$VENV_PATH/bin/activate"

# Now you can run Python commands within the virtual environment
python -m pip install --upgrade pip
python setup.py

# Deactivate the virtual environment when done
deactivate