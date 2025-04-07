#! /usr/bin/env bash

# We remove a folder that otherwise gives issues in ROS2 launches
sudo rm -r /home/user/.ros

# Check if the first argument is 'debug'
if [ "$1" = "debug" ]; then
    export ROS2_WS_PATH=/home/user/ros2_ws
else
    export ROS2_WS_PATH=/home/simulations/ros2_sims_ws
fi

# We set up the environment for ROS2
. /usr/share/gazebo/setup.sh
. ${ROS2_WS_PATH}/install/setup.bash

export GAZEBO_RESOURCE_PATH=${ROS2_WS_PATH}/src/leo_rover_simulation/leo_description:${GAZEBO_RESOURCE_PATH}
export GAZEBO_MODEL_PATH=${ROS2_WS_PATH}/src/leo_rover_simulation/leo_description:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_DATABASE_URI=""
ros2 launch leo_description main_simple.launch.py
