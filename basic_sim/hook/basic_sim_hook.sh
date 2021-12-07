#!/bin/bash

# Generate prefix from 4 levels up from env-hook directory (colcon ws install folder)
AMENT_CURRENT_PREFIX=$(ros2 pkg prefix basic_sim)

# Use ament functionality to prepend Gazebo env vars with custom package resources
ament_prepend_unique_value GAZEBO_MODEL_PATH "$AMENT_CURRENT_PREFIX/share/basic_sim/models"
ament_prepend_unique_value GAZEBO_RESOURCE_PATH "$AMENT_CURRENT_PREFIX/share/basic_sim/worlds"