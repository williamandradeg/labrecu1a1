#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source "$HOME/lrrecupera/install/setup.bash"
export ROS_DOMAIN_ID=81
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=lrrecupera_celda
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib:$HOME/lrrecupera/install/celda_robotica/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share:$HOME/lrrecupera/src/celda_robotica:${GZ_SIM_RESOURCE_PATH:-}"
