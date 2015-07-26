#ifndef MOTORSHIELD_H
#define MOTORSHIELD_H

#include <stdio.h>
#include <sstream>

#include <signal.h>

#include <time.h>
#include <unistd.h>

#include <stdint.h>

#include "motor_ctrl.h"

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "ros_edison/MotorShield.h"

//Version string
#define VERSION_STRING "0.1"

//Node name in ROS environment
#define NODE_NAME "motor_shield"

#endif
