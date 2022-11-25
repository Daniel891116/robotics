#!/bin/bash

cd /home/robot/workspace2/team3_ws/HW4
colcon build
source install/setup.bash

ros2 run send_script $1
