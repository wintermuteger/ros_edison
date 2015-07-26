#ifndef SYSMON_H
#define SYSMON_H

#include <stdio.h>
#include <sstream>

#include <signal.h>

#include <time.h>
#include <unistd.h>

#include <stdint.h>

#include "gpio_ctrl.h"

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"

//Version string
#define VERSION_STRING "0.1"

//Node name in ROS environment
#define NODE_NAME "sys_mon"

#endif
