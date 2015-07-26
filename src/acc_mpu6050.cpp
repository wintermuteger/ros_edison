#include "acc_mpu6050.h"


//
//Main function - setting up all services, puglishers and subscribers of this node
//

int main(int argc, char** argv)
{
  printf("[ros_edison] Acceloremeter MPU6050 Node V.%s\n", VERSION_STRING);
  printf("Copyright 2015, Christoph Schultz\n");

  ros::init(argc, argv, NODE_NAME);

  ROS_INFO("Setting up node %s...", NODE_NAME);

  ros::NodeHandle n;

  ROS_INFO("Initializing I2C/accelerometer...");

  acc_init();

  ROS_INFO("Registering Acc publisher...");
  ros::Publisher acc_pub = n.advertise<geometry_msgs::Twist>("accelerometer", 100);

  ros::Rate loop_rate(100);
 

  ROS_INFO("Sys-mon starts publishing");
  geometry_msgs::Twist acc_msg;

  while(ros::ok())
    {
      acc_msg.linear.x = (float)read_X_acc();
      acc_msg.linear.y = (float)read_Y_acc();
      acc_msg.linear.z = (float)read_Z_acc();
      acc_msg.angular.x = (float)read_X_gyro();
      acc_msg.angular.y = (float)read_Y_gyro();
      acc_msg.angular.z = (float)read_Z_gyro();
      acc_pub.publish(acc_msg);
      ros::spinOnce();
      loop_rate.sleep();
    }

  acc_shutdown();
  
  return 0;
}
