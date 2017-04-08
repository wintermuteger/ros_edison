#include "motor_shield.h"

//ATTENTION: The code right now does basically nothing. The mechanism is missing that waits for engineA and engineB being programmed if timerRequested is set. I am in fear of race conditions here. In this case timerRunning basically will never become active.

//Global tracking the timer status for timed engine-control
bool timer_engA_set = false;
bool timer_engB_set = false;
bool timerRequested = false;
bool timerRunning = false;
unsigned int timerVal = 0;

//
//Callback for "activateEngine" Service
//
void ctrlEngineA(const std_msgs::Float32& msg)
{
  //
  //Determine the control values for the motor control command!
  //
  uint8_t dirA = 0;
  uint8_t powA = 0;

  if(msg.data >= 0.0f)
    {
      dirA = 0;
      powA = (uint8_t)(msg.data*255.0f);
    }
  else
    {
      dirA = 1;
      powA = (uint8_t)(-msg.data*255.0f); 
    }  

  //
  //Call engines control
  //
  motors_control(dirA,powA,'A');
}

void ctrlEngineB(const std_msgs::Float32& msg)
{

  //
  //Determine the control values for the motor control command!
  //
  uint8_t dirB = 0;
  uint8_t powB = 0;
 
  if(msg.data >= 0.0f)
    {
      dirB = 0;
      powB = (uint8_t)(msg.data*255.0f); 
    }
  else
    {
      dirB = 1;
      powB = (uint8_t)(-msg.data*255.0f); 
    }
  
  //
  //Call engines control
  //
  motors_control(dirB,powB,'B');
}

//Function to set the tick timer
void setTimer(const std_msgs::Int16& msg)
{
  if(msg.data > 0)
  {
    timerVal = msg.data;
    if(timer_engA_set && timer_engB_set)
    {
       timerRunning = true;
    }
    else
    {
       timerRequested = true;
    } 
  }
}

//
//Main function - setting up all services, puglishers and subscribers of this node
//

int main(int argc, char** argv)
{
  printf("[ros_edison] Motor-Shield Node V.%s\n", VERSION_STRING);
  printf("Copyright 2017, Christoph Schultz\n");

  ros::init(argc, argv, NODE_NAME);

  ROS_INFO("Setting up node %s...", NODE_NAME);

  ros::NodeHandle n;

  ROS_INFO("Initializing motor-shield...");
  motor_init();

  ROS_INFO("Registering motor-control subscribers...");
  ros::Subscriber subA = n.subscribe("engineA", 1000, ctrlEngineA);
  ros::Subscriber subB = n.subscribe("engineB", 1000, ctrlEngineB);
  ros::Subscriber subT = n.subscribe("timer", 1000, setTimer);

  ROS_INFO("Motor control is ready");

  ros::Rate r(10);

  timer_engA_set = false;
  timer_engB_set = false;
  timerRequested = false;
  timerRunning = false;
  timerVal = 0;

  while(true)
  {
    if(timerRunning)
    {
      if(timerVal == 0)
      {
	motors_control(0,0,'A');
	motors_control(0,0,'B');
	timer_engA_set = false;
	timer_engB_set = false;
	timerRequested = false;
	timerRunning = false;
      }
      else //timerVal != 0
      {
	timerVal = timerVal - 1;
      }	
    }

    ros::spinOnce();
    r.sleep();
  }
  motor_shutdown();

  return 0;
}
