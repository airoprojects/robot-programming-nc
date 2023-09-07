// Author: Leonardo Colosi

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// Custom include 
#include "types.h"

int main(int argc, char** argv) {

  // This is a standard procedure to initialize a ROS node and a node handler
  // also a publisher object is created and it will publish on "cmd_vel" topic 
  // msg about the robot commanded velocity
  ros::init(argc, argv, "robot_node");
  ros::NodeHandle nh;
  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ROS_INFO("A new robot has spawned");

  // Set the loop rate in Hz
  ros::Rate loop_rate(15);

  while (ros::ok()) {
    // Call to custom keyboard handler
    geometry_msgs::Twist velocity_command = handleKeyboardInput();
    ROS_INFO("CMD Velocity: %f", velocity_command.linear.x);
    cmd_vel_pub.publish(velocity_command);
    ros::spinOnce();

    // Sleep for the remainder of the loop
    loop_rate.sleep();
  }

  endwin(); // End the ncurses session
  return 0;
}