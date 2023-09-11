#include <opencv2/imgproc.hpp>

#include "robot.h"

Robot::Robot(float radius_, std::shared_ptr<World> w_, const Pose& pose_)
    : radius(radius_), WorldItem(w_, pose_), tv(0.0), rv(0.0), nh("~"), 
      odom_pub(nh.advertise<nav_msgs::Odometry>("odom", 10)),
      cmd_vel_sub(nh.subscribe("cmd_vel", 10, &Robot::cmdVelCallback, this)) {}

Robot::Robot(float radius_, std::shared_ptr<WorldItem> parent_,
             const Pose& pose_)
    : radius(radius_), WorldItem(parent_, pose_), tv(0.0), rv(0.0), nh("~"), 
      odom_pub(nh.advertise<nav_msgs::Odometry>("odom", 10)),
      cmd_vel_sub(nh.subscribe("cmd_vel", 10, &Robot::cmdVelCallback, this)) {}


void Robot::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // Update the robot's velocity based on received commands
    tv = msg->linear.x;
    rv = msg->angular.z;
}


// To edit in world
void Robot::draw() {
  int int_radius = radius * world->i_res;
  IntPoint p = world->world2grid(poseInWorld().translation());
  cv::circle(world->display_image, cv::Point(p.y(), p.x()), int_radius,
             cv::Scalar::all(0), -1);
}

void Robot::timeTick(float dt) {
  //initializing
  Pose motion = Pose::Identity();
  motion.translation() << tv * dt, 0;
  motion.rotate(rv * dt);

  //apply trasformation
  Pose next_pose = pose_in_parent * motion;
  IntPoint ip = world->world2grid(next_pose.translation());
  int int_radius = radius * world->i_res;
  if (!world->collides(ip, int_radius)) pose_in_parent = next_pose;

  // Publish odometry data
  nav_msgs::Odometry odom;
  // ... (fill in odometry data)
  odom_pub.publish(odom);
}
