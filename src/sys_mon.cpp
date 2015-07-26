#include "sys_mon.h"

//CPU times
int user_c = 0;
int nice_c = 0;
int system_c = 0;
int idle_c = 0;

//ROS counter
int ros_cycles = 0;

float getCPUload()
{
  int latest_user = 0;
  int latest_nice = 0;
  int latest_system = 0;
  int latest_idle = 0;
  FILE* fdescr = NULL;
  fdescr = fopen("/proc/stat", "r");
  if(fdescr == NULL)
    {
      printf("File %s could not be opened!\n", "/proc/stat");
      return 0.0;
    }
  fscanf(fdescr,"cpu %i %i %i %i",&latest_user, &latest_nice, &latest_system, &latest_idle);
  fclose(fdescr);

  float load;
  float total;
  load = (float)((latest_user-user_c)+(latest_nice-nice_c)+(latest_system-system_c));
  total = load+(float)(latest_idle-idle_c);
  load = load/total;
  

  user_c = latest_user;
  nice_c = latest_nice;
  system_c = latest_system;
  idle_c = latest_idle;

  return load;
  
}

float getBatVolt()
{
  const char* FILE_NAME = "/sys/bus/iio/devices/iio:device1/in_voltage2_raw";
  int adc_val = 0;
  FILE* fdescr = NULL;
  fdescr = fopen(FILE_NAME, "r");
  if(fdescr == NULL)
    {
      printf("File %s could not be opened!\n", FILE_NAME);
      return 0.0;
    }
  fscanf(fdescr,"%i",&adc_val);
  fclose(fdescr);
  

  return ((float)adc_val)/4095.0f*10.0f;
}

void registerADC2Pins()
{
  //Enable TriStateAll to protect FETs
  set_gpio(214,0);

  //Set Muxes, Pullup and direction
  set_gpio(202,1);
  set_gpio(234,0);
  
  //Deactivate Pull-Up
  sys_write("/sys/class/gpio/export", 210);
  sys_write("/sys/class/gpio/gpio%d/direction", 210, "in");
  sys_write("/sys/class/gpio/unexport",210);

  //Disable TriStateAll
  set_gpio(214,1);
}


//
//Main function - setting up all services, puglishers and subscribers of this node
//

int main(int argc, char** argv)
{
  printf("[ros_edison] Sys-Mon Node V.%s\n", VERSION_STRING);
  printf("Copyright 2015, Christoph Schultz\n");

  ros::init(argc, argv, NODE_NAME);

  ROS_INFO("Setting up node %s...", NODE_NAME);

  ros::NodeHandle n;

  ROS_INFO("Initializing sys-mon...");

  ROS_INFO("Registering CPU publisher...");
  ros::Publisher cpu_pub = n.advertise<std_msgs::Float32>("cpu_load", 100);
  
  ROS_INFO("Registering ROS counter publisher...");
  ros::Publisher cnt_pub = n.advertise<std_msgs::Int32>("sys_count", 100);

  ROS_INFO("Registering Battery Voltage publisher...");
  ros::Publisher bat_pub = n.advertise<std_msgs::Float32>("bat_volt", 100);

  ros::Rate loop_rate(100);
 

  ROS_INFO("Sys-mon starts publishing");

  std_msgs::Float32 cpu_load_msg;
  std_msgs::Int32 cnt_msg;
  std_msgs::Float32 bat_volt_msg;

  registerADC2Pins();

  while(ros::ok())
    {
      
      if((ros_cycles/100)*100==ros_cycles)
	{
           cpu_load_msg.data = getCPUload();
           cpu_pub.publish(cpu_load_msg);
	   
	   bat_volt_msg.data = getBatVolt();
	   bat_pub.publish(bat_volt_msg);
	}
      cnt_msg.data = ros_cycles;
      cnt_pub.publish(cnt_msg);
      ros::spinOnce();
      loop_rate.sleep();
      ros_cycles ++;
    }
  
  return 0;
}
