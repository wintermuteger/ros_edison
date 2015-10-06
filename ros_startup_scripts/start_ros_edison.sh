#!/bin/sh

/opt/ros/indigo/lib/ros_edison/motor_shield &
sleep 3
/opt/ros/indigo/lib/ros_edison/sys_mon &
sleep 3
/opt/ros/indigo/lib/ros_edison/acc_mpu6050 &
sleep 3
