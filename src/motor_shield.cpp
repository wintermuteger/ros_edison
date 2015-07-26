#include "motor_shield.h"

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

//
//Main function - setting up all services, puglishers and subscribers of this node
//

int main(int argc, char** argv)
{
  printf("[ros_edison] Motor-Shield Node V.%s\n", VERSION_STRING);
  printf("Copyright 2015, Christoph Schultz\n");

  ros::init(argc, argv, NODE_NAME);

  ROS_INFO("Setting up node %s...", NODE_NAME);

  ros::NodeHandle n;

  ROS_INFO("Initializing motor-shield...");
  motor_init();

  ROS_INFO("Registering motor-control subscribers...");
  ros::Subscriber subA = n.subscribe("engineA", 1000, ctrlEngineA);
  ros::Subscriber subB = n.subscribe("engineB", 1000, ctrlEngineB);
 

  ROS_INFO("Motor control is ready");

  ros::spin();

  motor_shutdown();

  return 0;
}
