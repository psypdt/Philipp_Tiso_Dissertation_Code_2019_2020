#!/bin/bash

cd ros_ws1
source /opt/ros/kinetic/setup.bash && source devel/setup.bash
roslaunch intera_examples test_launch.launch
