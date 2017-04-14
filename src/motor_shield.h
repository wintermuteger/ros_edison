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
#include "std_msgs/Int16.h"
#include "ros_edison/MotorShield.h"

//Version string
#define VERSION_STRING "0.2"

//Node name in ROS environment
#define NODE_NAME "motor_shield"

//Control commands
#define CMD_REQUEST_TIMER 1
#define CMD_RESET_TIMER 2

#endif
