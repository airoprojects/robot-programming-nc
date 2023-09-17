#pragma once

#include "types.h"
#include "world.h"

#include "ros/ros.h"
#include "mrsim/rodom.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h> 

// Definitiopn of robot struct that extend world item
struct Robot : public WorldItem {
  
  // Robot constructor that takes as input a word to which the robot belong
  Robot(std::shared_ptr<World> w_,
        string frame_id_, 
        string namespace_, 
        float radius_, 
        float max_rv_,
        float max_tv_,
        const Pose& pose_ = Pose::Identity(), 
        int id_p = -1);
  
  // Robot contructor that takes as input a world item to which the robot belong
  Robot(std::shared_ptr<WorldItem> parent_, 
        string frame_id_, 
        string namespace_,
        float radius_, 
        float max_rv_,
        float max_tv_,
        const Pose& pose_ = Pose::Identity(), 
        int id_p = -1);

  // Method to draw the robot on the map with open cv
  void draw() override;

  // Method to update the robot pose in function of the velocity
  void timeTick(float dt) override;

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

  void customTf2();

  // Robot dadious and initial translational and rotational velocity
  string frame_id;
  float radius;
  float tv = 0, rv = 0, max_rv, max_tv;
  int id_p = -1;

  // ROS
  ros::NodeHandle nh;  // ROS Node Handle
  ros::Publisher odom_pub;  // Publisher to send odometry data
  ros::Subscriber cmd_vel_sub;  // Subscriber to receive velocity commands
};    