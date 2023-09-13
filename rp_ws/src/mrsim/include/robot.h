#pragma once

#include "types.h"
#include "world.h"

#include "ros/ros.h"
#include "mrsim/rodom.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

// Definitiopn of robot struct that extend world item
struct Robot : public WorldItem {
  
  // Robot constructor that takes as input a word to which the robot belong
  Robot(float radius_, std::shared_ptr<World> w_,
        std::string namespace_, const Pose& pose_ = Pose::Identity());
  
  // Robot contructor that takes as input a world item to which the robot belong
  Robot(float radius_, std::shared_ptr<WorldItem> parent_, 
        std::string namespace_, const Pose& pose_ = Pose::Identity());

  // Method to draw the robot on the map with open cv
  void draw() override;

  // Method to update the robot pose in function of the velocity
  void timeTick(float dt) override;

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

  // Robot dadious and initial translational and rotational velocity
  float radius;
  float tv = 0, rv = 0;

  // Node fields
  ros::NodeHandle nh;  // ROS Node Handle
  ros::Publisher odom_pub;  // Publisher to send odometry data
  ros::Subscriber cmd_vel_sub;  // Subscriber to receive velocity commands
};    