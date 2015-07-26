#include "gpio_ctrl.h"

int sys_write(const char* descr,uint16_t descrnum ,const char* msg)
{
  const unsigned int DESCR_LEN = 64;

  char buffer_descr[DESCR_LEN];
  snprintf(buffer_descr, DESCR_LEN, descr, descrnum);


  FILE* fdescr = NULL;
  fdescr = fopen(buffer_descr, "w");
  if(fdescr == NULL)
    {
      printf("File %s could not be opened!\n", buffer_descr);
      return 0;
    }
  fprintf(fdescr,"%s", msg);
  fclose(fdescr);
}

int sys_write(const char* descr,int16_t msg_num)
{
  const unsigned int MSG_LEN = 16;

  char buffer_msg[MSG_LEN];

  //Export GPIO
  snprintf(buffer_msg, MSG_LEN, "%d", msg_num);

  return sys_write(descr, buffer_msg);
}

int sys_write(const char* descr,const char* msg)
{
  FILE* fdescr = NULL;
  fdescr = fopen(descr, "w");
  if(fdescr == NULL)
    {
      printf("File %s could not be opened!\n", descr);
      return 0;
    }
  fprintf(fdescr,"%s", msg);
  fclose(fdescr);
}

int set_gpio(unsigned int gpionum, unsigned int val)
{
  const unsigned int MSG_LEN = 16;

  char buffer_msg[MSG_LEN];

  //Export GPIO
  sys_write("/sys/class/gpio/export", (int16_t)gpionum);

  //Set out direction
  sys_write( "/sys/class/gpio/gpio%d/direction", gpionum, "out");

  //Set Value
  snprintf(buffer_msg, MSG_LEN, "%d", val);
  sys_write("/sys/class/gpio/gpio%d/value", gpionum, (const char*) buffer_msg);

  //Unexport
  sys_write("/sys/class/gpio/unexport", (int16_t)gpionum);
}

int set_SoC_mode(unsigned int gpionum, const char* mode_str)
{

  const unsigned int MSG_LEN = 16;

  char buffer_msg[MSG_LEN];

  //Set Value
  snprintf(buffer_msg, MSG_LEN, "%s", mode_str);
  sys_write("/sys/kernel/debug/gpio_debug/gpio%d/current_pinmux", gpionum, (const char*) buffer_msg);
}
