#ifndef ACC_MPU6050_H
#define ACC_MPU6050_H

#include <stdio.h>
#include <sstream>

#include <signal.h>

#include <time.h>
#include <unistd.h>

#include <stdint.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "acc_ctrl.h"

//Version string
#define VERSION_STRING "0.1"

//Node name in ROS environment
#define NODE_NAME "acc_mpu6050"

#endif
