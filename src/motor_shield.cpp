#include "motor_shield.h"

//Global tracking the timer status for timed engine-control
int16_t dirA_buff = -1;
int16_t powA_buff = -1;
int16_t dirB_buff = -1;
int16_t powB_buff = -1;
bool timerRequested = false;
bool timerRunning = false;
int16_t timerVal = -1;

//
//Callback for "engineA" Service
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
  if(timerRequested == false)
    {  
      //
      //Call engines control
      //
      motors_control(dirA,powA,'A');
    }
  else //timerRequested == true
    {
      dirA_buff = (int16_t)dirA;
      powA_buff = (int16_t)powA;
      ROS_INFO("TimerRequested - Parameters set to dirA: %d, powA: %d", dirA_buff, powA_buff);
    }
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
  if(timerRequested == false)
    { 
      //
      //Call engines control
      //
      motors_control(dirB,powB,'B');
    }
  else //timerRequested == true;
    {
      dirB_buff = (int16_t)dirB;
      powB_buff = (int16_t)powB;
      ROS_INFO("TimerRequested - Parameters set to dirB: %d, powB: %d", dirB_buff, powB_buff);
    }
}

//Function to set the tick timer
void setTimer(const std_msgs::Int16& msg)
{
  if(msg.data >= 0)
  {
    timerVal = msg.data;
    ROS_INFO("TimerRequested - Parameters set to timerVal %d", timerVal);
  }
}

//Service to handle state control commands
bool controlEngine(ros_edison::MotorShield::Request & req,ros_edison::MotorShield::Response &res)
{
  switch(req.ctrlCmd)
    {
    case CMD_REQUEST_TIMER:
      res.success = true;
      ROS_INFO("Timer requested");
      powA_buff = -1;
      dirA_buff = -1;
      powB_buff = -1;
      dirB_buff = -1;
      timerRunning = false;
      timerVal = -1;
      timerRequested = true;
      break;
    case CMD_RESET_TIMER:
      res.success = true;
      ROS_INFO("Timer reset");
      timerRequested = false;
      timerRunning = false;
      powA_buff = -1;
      dirA_buff = -1;
      powB_buff = -1;
      dirB_buff = -1;
      timerVal = -1;
      break;
    default:
      res.success = false;
      ROS_ERROR("Motor-Shield received unknown control command with code %d", req.ctrlCmd);
      break;
    };

  return res.success;
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
  ros::ServiceServer service = n.advertiseService("MotorShield", controlEngine);

  ROS_INFO("Motor control is ready");

  ros::Rate r(10);
  powA_buff = -1;
  dirA_buff = -1;
  powB_buff = -1;
  dirB_buff = -1;
  timerRequested = false;
  timerRunning = false;
  timerVal = -1;

  bool lastTimerRequested = false;

  while(true)
  {
    
    if(lastTimerRequested != timerRequested)
      {
	ROS_INFO("Status of timerRequested changed to %d", timerRequested);
      }
    lastTimerRequested = timerRequested;

    if(timerRequested) //Timer has been requested, check if all values (timer, engineA and engineB) have been received
      {
	if(powA_buff >= 0 && powB_buff >= 0 && timerVal >= 0)
	  {
	    timerRequested = false;
	    timerRunning = true;
	    //Start motors
	    motors_control((uint8_t)dirA_buff,(uint8_t)powA_buff,'A');
	    motors_control((uint8_t)dirB_buff,(uint8_t)powB_buff,'B');

	    ROS_INFO("Timed operation started - Parameters set to dirA: %d, powA: %d, dirB: %d, powB: %d, timer %d", dirA_buff, powA_buff, dirB_buff, powB_buff, timerVal);
	  }
      }
    else if(timerRunning) //Timer was first requested and is now running
    {
      if(timerVal == 0)
      {
	//Reset all timer globals and stop control
	motors_control(0,0,'A');
	motors_control(0,0,'B');
	powA_buff = -1;
	dirA_buff = -1;
	powB_buff = -1;
	dirB_buff = -1;
	timerVal = -1;
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
