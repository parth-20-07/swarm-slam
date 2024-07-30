#!/bin/bash

# Prompt the user for the number of robots
read -p "Enter the number of robots: " NUM_OF_ROBOTS

# Set the environment variables
export TURTLEBOT3_MODEL=burger  # You can also prompt the user for this if needed
export NUM_OF_ROBOTS

# Generate the launch files
python3 $(rospack find turtlebot3_gazebo)/launch/generate_turtlebot3_launch_file.py
python3 $(rospack find slam)/launch/generate_random_movement_launch_file.py

# Start the main launch file
roslaunch slam main.launch
