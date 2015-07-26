#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include <sys/ioctl.h>

#include <stdio.h>
#include <sstream>

#include <errno.h>
#include <fcntl.h>

#include <time.h>
#include <unistd.h>

#include <stdlib.h>

#include <stdint.h>

#include "ros/ros.h"

#include "gpio_ctrl.h"

#define TRISTATE_ALL 214

#define BRAKE_A_LS 256
#define BRAKE_A_IO 13
#define BRAKE_B_LS 257
#define BRAKE_B_IO 49
#define DIR_A_LS 260
#define DIR_A_MUX 242
#define DIR_A_IO 42
#define DIR_B_LS 261
#define DIR_B_MUX 243
#define DIR_B_IO 40
#define PWM_A_LS 251
#define PWM_A_IO 12
#define PWM_A 0
#define PWM_B_LS 259
#define PWM_B_IO 183
#define PWM_B 3

void activateMotorShield(void);
void deactivateMotorShield(void);

int motor_init(void);
int motor_shutdown(void);

int motors_control(uint8_t,uint8_t,const char);

#endif
