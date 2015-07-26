#include "motor_ctrl.h"

#define PWM_MIN 5000000 
#define PWM_MAX 10000000
#define PWM_SPAN (PWM_MAX-PWM_MIN) 
#define PWM_PERIOD 10000000

//Monitor if pwm is registered
int pwm_is_registered = 0;

int motors_control(uint8_t dir,uint8_t pow,const char engine)
{
  const uint8_t BUFFER_LEN = 32;
  char buffer[BUFFER_LEN];

  if(engine == 'A')
    { 
      //Set direction
      if(dir != 0)
	{
	  sys_write("/sys/class/gpio/gpio%d/value",DIR_A_IO, "1"); 
	}
      else
	{
	  sys_write("/sys/class/gpio/gpio%d/value",DIR_A_IO, "0"); 
	}
      //Set Brakes
      if(pow == 0)
	{
	  sys_write("/sys/class/gpio/gpio%d/value",BRAKE_A_IO, "1");
	}
      else
	{
	  sys_write("/sys/class/gpio/gpio%d/value",BRAKE_A_IO, "0");
	}
      //Set Duty-Cycle

      const float duty_max = (float)PWM_MAX;
      const float duty_min = (float)PWM_MIN;
      const float duty_span = (float)PWM_SPAN;
      float dutyA = (float)pow/255.0f*duty_span+duty_min;
      
      snprintf(buffer,BUFFER_LEN, "%d",(uint32_t)dutyA); 
      sys_write("/sys/class/pwm/pwmchip0/pwm%d/duty_cycle", PWM_A, buffer);
    }
  else if(engine == 'B')
    {
      //Set direction
      if(dir != 0)
	{
	  sys_write("/sys/class/gpio/gpio%d/value",DIR_B_IO, "0"); 
	}
      else
	{
	  sys_write("/sys/class/gpio/gpio%d/value",DIR_B_IO, "1"); 
	}
      //Set Brakes
      if(pow == 0)
	{
	  sys_write("/sys/class/gpio/gpio%d/value",BRAKE_B_IO, "1");
	}
      else
	{
	  sys_write("/sys/class/gpio/gpio%d/value",BRAKE_B_IO, "0");
	}
      //Set Duty-Cycle
      const float duty_max = (float)PWM_MAX;
      const float duty_min = (float)PWM_MIN;
      const float duty_span = (float)PWM_SPAN;
      float dutyB = (float)pow/255.0f*duty_span+duty_min;
  
      snprintf(buffer,BUFFER_LEN, "%d",(uint32_t)dutyB); 
      sys_write("/sys/class/pwm/pwmchip0/pwm%d/duty_cycle", PWM_B, buffer);
    }
}

int motor_init()
{
  ROS_INFO("Initializing motor control...");
  activateMotorShield();
  pwm_is_registered = 1;

  return 1;
}

int motor_shutdown()
{
  ROS_INFO("Shuttind down motor control...");
  if(pwm_is_registered != 0)
    {
      deactivateMotorShield();
      pwm_is_registered = 0;
    }
  return 1;
}

void activateMotorShield()
{
  const uint8_t BUFFER_LEN = 32;
  char buffer[BUFFER_LEN];
 
  //Deactivate VDD_SHIELD_SW by asserting TRISTATE_ALL
  snprintf(buffer,BUFFER_LEN, "%d",TRISTATE_ALL); 
  sys_write("/sys/class/gpio/export", buffer);
  sys_write("/sys/class/gpio/gpio%d/direction",TRISTATE_ALL ,"out");  
  sys_write("/sys/class/gpio/gpio%d/value",TRISTATE_ALL, "0"); 
  sys_write("/sys/class/gpio/unexport",buffer);

  //Configure Brake A
  snprintf(buffer,BUFFER_LEN, "%d",BRAKE_A_LS); 
  sys_write("/sys/class/gpio/export", buffer);
  sys_write("/sys/class/gpio/gpio%d/direction",BRAKE_A_LS ,"out");  
  sys_write("/sys/class/gpio/gpio%d/value",BRAKE_A_LS, "1"); 
  sys_write("/sys/class/gpio/unexport",buffer);

  snprintf(buffer,BUFFER_LEN, "%d",BRAKE_A_IO); 
  sys_write("/sys/class/gpio/export", buffer);
  sys_write("/sys/class/gpio/gpio%d/direction",BRAKE_A_IO ,"out");  
  sys_write("/sys/class/gpio/gpio%d/value",BRAKE_A_IO, "1"); 

  //Configure Brake B
  snprintf(buffer,BUFFER_LEN, "%d",BRAKE_B_LS); 
  sys_write("/sys/class/gpio/export", buffer);
  sys_write("/sys/class/gpio/gpio%d/direction",BRAKE_B_LS ,"out");  
  sys_write("/sys/class/gpio/gpio%d/value",BRAKE_B_LS, "1"); 
  sys_write("/sys/class/gpio/unexport",buffer);

  snprintf(buffer,BUFFER_LEN, "%d",BRAKE_B_IO); 
  sys_write("/sys/class/gpio/export", buffer);
  sys_write("/sys/class/gpio/gpio%d/direction",BRAKE_B_IO ,"out");  
  sys_write("/sys/class/gpio/gpio%d/value",BRAKE_B_IO, "1"); 
  

  //Configure Dir A
  snprintf(buffer,BUFFER_LEN, "%d",DIR_A_LS); 
  sys_write("/sys/class/gpio/export", buffer);
  sys_write("/sys/class/gpio/gpio%d/direction",DIR_A_LS ,"out");  
  sys_write("/sys/class/gpio/gpio%d/value",DIR_A_LS, "1"); 
  sys_write("/sys/class/gpio/unexport",buffer);

  sys_write("/sys/class/gpio/export",DIR_A_MUX);
  sys_write("/sys/class/gpio/gpio%d/direction",DIR_A_MUX ,"out");  
  sys_write("/sys/class/gpio/gpio%d/value",DIR_A_MUX, "0");
  sys_write("/sys/class/gpio/unexport", DIR_A_MUX);

  sys_write("/sys/kernel/debug/gpio_debug/gpio%d/current_pinmux", DIR_A_IO, "mode0");

  snprintf(buffer,BUFFER_LEN, "%d",DIR_A_IO); 
  sys_write("/sys/class/gpio/export", buffer);
  sys_write("/sys/class/gpio/gpio%d/direction",DIR_A_IO ,"out");  
  sys_write("/sys/class/gpio/gpio%d/value",DIR_A_IO, "0"); 

  //Configure Dir B
  snprintf(buffer,BUFFER_LEN, "%d",DIR_B_LS); 
  sys_write("/sys/class/gpio/export", buffer);
  sys_write("/sys/class/gpio/gpio%d/direction",DIR_B_LS ,"out");  
  sys_write("/sys/class/gpio/gpio%d/value",DIR_B_LS, "1"); 
  sys_write("/sys/class/gpio/unexport",buffer);

  sys_write("/sys/class/gpio/export",DIR_B_MUX);
  sys_write("/sys/class/gpio/gpio%d/direction",DIR_B_MUX ,"out");  
  sys_write("/sys/class/gpio/gpio%d/value",DIR_B_MUX, "0");
  sys_write("/sys/class/gpio/unexport", DIR_B_MUX);

  sys_write("/sys/kernel/debug/gpio_debug/gpio%d/current_pinmux", DIR_B_IO, "mode0");

  snprintf(buffer,BUFFER_LEN, "%d",DIR_B_IO); 
  sys_write("/sys/class/gpio/export", buffer);
  sys_write("/sys/class/gpio/gpio%d/direction",DIR_B_IO ,"out");  
  sys_write("/sys/class/gpio/gpio%d/value",DIR_B_IO, "0"); 

  //Activate PWM A
  snprintf(buffer,BUFFER_LEN, "%d",PWM_A_LS); 
  sys_write("/sys/class/gpio/export", buffer);
  sys_write("/sys/class/gpio/gpio%d/direction",PWM_A_LS ,"out");  
  sys_write("/sys/class/gpio/gpio%d/value",PWM_A_LS, "1"); 
  sys_write("/sys/class/gpio/unexport",buffer);

  snprintf(buffer,BUFFER_LEN, "%d",PWM_A_IO); 
  sys_write("/sys/class/gpio/export", buffer);
  sys_write("/sys/class/gpio/gpio%d/direction",PWM_A_IO ,"out");  
  sys_write("/sys/class/gpio/gpio%d/value",PWM_A_IO, "0"); 
  sys_write("/sys/class/gpio/unexport",buffer);

  snprintf(buffer,BUFFER_LEN, "%d",PWM_A); 
  sys_write("/sys/class/pwm/pwmchip0/export", buffer);
  sys_write("/sys/class/pwm/pwmchip0/pwm%d/enable",PWM_A ,"1");  
  snprintf(buffer,BUFFER_LEN, "%d",PWM_PERIOD); 
  sys_write("/sys/class/pwm/pwmchip0/pwm%d/period",PWM_A, buffer); 
  sys_write("/sys/class/pwm/pwmchip0/pwm%d/duty_cycle", PWM_A, "0");

  sys_write("/sys/kernel/debug/gpio_debug/gpio%d/current_pinmux", PWM_A_IO, "mode1");

  //Activate PWM B
  snprintf(buffer,BUFFER_LEN, "%d",PWM_B_LS); 
  sys_write("/sys/class/gpio/export", buffer);
  sys_write("/sys/class/gpio/gpio%d/direction",PWM_B_LS ,"out");  
  sys_write("/sys/class/gpio/gpio%d/value",PWM_B_LS, "1"); 
  sys_write("/sys/class/gpio/unexport",buffer);

  snprintf(buffer,BUFFER_LEN, "%d",PWM_B_IO); 
  sys_write("/sys/class/gpio/export", buffer);
  sys_write("/sys/class/gpio/gpio%d/direction",PWM_B_IO ,"out");  
  sys_write("/sys/class/gpio/gpio%d/value",PWM_B_IO, "0"); 
  sys_write("/sys/class/gpio/unexport",buffer);

  snprintf(buffer,BUFFER_LEN, "%d",PWM_B); 
  sys_write("/sys/class/pwm/pwmchip0/export", buffer);
  sys_write("/sys/class/pwm/pwmchip0/pwm%d/enable",PWM_B ,"1");  
  snprintf(buffer,BUFFER_LEN, "%d",PWM_PERIOD); 
  sys_write("/sys/class/pwm/pwmchip0/pwm%d/period",PWM_B, buffer); 
  sys_write("/sys/class/pwm/pwmchip0/pwm%d/duty_cycle", PWM_B, "0");

  sys_write("/sys/kernel/debug/gpio_debug/gpio%d/current_pinmux", PWM_B_IO, "mode1");

  //Activate VDD_SHIELD_SW by asserting TRISTATE_ALL
  snprintf(buffer,BUFFER_LEN, "%d",TRISTATE_ALL); 
  sys_write("/sys/class/gpio/export", buffer);
  sys_write("/sys/class/gpio/gpio%d/direction",TRISTATE_ALL ,"out");  
  sys_write("/sys/class/gpio/gpio%d/value",TRISTATE_ALL, "1"); 
  sys_write("/sys/class/gpio/unexport",buffer);


  ROS_INFO("All GPIOs and PWMs set...");

  motors_control(0,0,'A');
  motors_control(0,0,'B');
}

void deactivateMotorShield() 
{
  const uint8_t BUFFER_LEN = 32;
  char buffer[BUFFER_LEN];
 
  ROS_INFO("Shutting down motor-shield...\n");
  
  motors_control(0,0,'A');
  motors_control(0,0,'B');

  //Unregsiter PWM module
  snprintf(buffer,BUFFER_LEN, "%d",BRAKE_A_IO); 
  sys_write("/sys/class/gpio/unexport", buffer);

  snprintf(buffer,BUFFER_LEN, "%d",BRAKE_B_IO); 
  sys_write("/sys/class/gpio/unexport", buffer);

  snprintf(buffer,BUFFER_LEN, "%d",DIR_A_IO); 
  sys_write("/sys/class/gpio/unexport", buffer);

  snprintf(buffer,BUFFER_LEN, "%d",DIR_B_IO); 
  sys_write("/sys/class/gpio/unexport", buffer);

  snprintf(buffer,BUFFER_LEN, "%d",PWM_A); 
  sys_write("/sys/class/pwm/pwmchip0/unexport", buffer);

  snprintf(buffer,BUFFER_LEN, "%d",PWM_B); 
  sys_write("/sys/class/pwm/pwmchip0/unexport", buffer);
  //return 1;
}
