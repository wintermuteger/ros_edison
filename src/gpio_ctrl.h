#ifndef GPIO_CTRL_H
#define GPIO_CTRL_H

#include <stdio.h>
#include <sstream>

#include <time.h>
#include <unistd.h>

#include <stdint.h>

int sys_write(const char* descr,uint16_t descrnum , const char* msg);
int sys_write(const char* descr,int16_t msg_num);
int sys_write(const char* descr, const char* msg);
int set_gpio(unsigned int gpionum, unsigned int val);
int set_SoC_mode(unsigned int gpionum, const char* mode_str);

#endif
