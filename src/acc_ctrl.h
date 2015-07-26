#ifndef ACC_CTRL_H
#define ACC_CTRL_H

#include <sys/ioctl.h>

#include <stdio.h>
#include <sstream>

#include <errno.h>
#include <fcntl.h>

#include <time.h>
#include <unistd.h>

#include <stdlib.h>

#include "linux/i2c-dev.h"

#include "gpio_ctrl.h"

#define I2C_ID 6

int acc_init(void);
int acc_shutdown(void);
int open_i2c_dev(int i2cbus, char *filename, size_t size, int quiet);
int set_slave_addr(int file, int address, int force);
int close_i2c_dev();
int16_t read_Y_acc();
int16_t read_Z_acc();
int16_t read_X_acc();
int16_t read_Y_gyro();
int16_t read_Z_gyro();
int16_t read_X_gyro();


#endif
