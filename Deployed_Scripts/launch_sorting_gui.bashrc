#!/bin/bash

cd ros_ws1
./intera.sh sim &
source devel/setup.bash
export ROS_PACKAGE_PATH=~/ros_ws1/src/intera_sdk/intera_examples:$ROS_PACKAGE_PATH
roslaunch intera_examples test_launch.launch
