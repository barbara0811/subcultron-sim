#!/bin/bash
# ROS Environment
#USER=stdops
#VEHICLE=amussel
#ROS_HOME=/home/${USER}/ros
#source ${ROS_HOME}/devel/setup.bash
#export ROS_MASTER_URI=http://localhost:11311

# Vehicle configuration environment
source `rospack find ${VEHICLE}`/data/devices/device_config.bash
source `rospack find ${VEHICLE}`/data/control/control_config.bash
source `rospack find ${VEHICLE}`/data/navigation/navigation_config.bash

# Launch configuration
export LAUNCH_PKG=amussel
export LAUNCH_FILE=amussel_standard_simulation.launch

# Configure logging
#export LOG_PATH=/home/${USER}/logs/launcher
#export ROS_LOG_DIR=/home/${USER}/logs/ros
export LOGGING=false

# Configure simulation
export XML_SAVE_PATH=`rospack find amussel`/data/mission.xml
export USE_NOISE=false

#Location for simulation or predefined position
export LOCATION=labos
export USE_LOCAL_FIX=1
