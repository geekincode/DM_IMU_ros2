#!/bin/bash

source install/setup.bash

password="123456"

echo $password | sudo -S chmod 777 /dev/ttyACM0

ros2 launch dm_imu dm_rviz.launch.py
