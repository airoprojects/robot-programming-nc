#include "robot.h"

Robot::Robot(float radius_, std::shared_ptr<World> w_,
            std::string namespace_, const Pose& pose_, int id_p_)
    : radius(radius_), WorldItem(w_, namespace_,  pose_), tv(0.0), rv(0.0), nh("~"), id_p(id_p_),
      odom_pub(nh.advertise<nav_msgs::Odometry>("/" + namespace_ + "/odom", 10)),
      cmd_vel_sub(nh.subscribe("/" + namespace_ + "/cmd_vel", 10, &Robot::cmdVelCallback, this)) {}

Robot::Robot(float radius_, std::shared_ptr<WorldItem> parent_,
            std::string namespace_, const Pose& pose_, int id_p_)
    : radius(radius_), WorldItem(parent_, namespace_, pose_), tv(0.0), rv(0.0), nh("~"), id_p(id_p_),
      odom_pub(nh.advertise<nav_msgs::Odometry>("/" + namespace_ + "/odom", 10)),
      cmd_vel_sub(nh.subscribe("/" + namespace_ + "/cmd_vel", 10, &Robot::cmdVelCallback, this)) {}


// Update the robot's velocity based on received commands
void Robot::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    tv = msg->linear.x;
    rv = msg->angular.z;
}

void Robot::draw() {

  int int_radius = radius * world->i_res;
  
  IntPoint p = world->world2grid(poseInWorld().translation());

  // cv::circle(world->display_image, cv::Point(p.y(), p.x()), int_radius,
  //            cv::Scalar::all(0), -1);
  cv::circle(world->display_image, cv::Point(p.y(), p.x()), int_radius,
             cv::Scalar(0,0, 0, 1), -1);

}

void Robot::timeTick(float dt) {
  // debug
  // cout << "tv " << tv << endl;
  // cout << "rv " << rv << endl;
  //

  Pose motion = Pose::Identity();
  motion.translation() << tv * dt, 0;
  motion.rotate(rv * dt);

  // std::cout << "pose in parent:\n" << pose_in_parent.matrix() << "\n";

  Pose next_pose = pose_in_parent * motion;
  IntPoint ip = world->world2grid(next_pose.translation());
  int int_radius = radius * world->i_res;
  if (!world->collides(ip, int_radius)) pose_in_parent = next_pose;

  // std::cout << "pose in parent robot:\n" << pose_in_parent.matrix() << "\n";
  // std::cout << "motion representation:\n" << motion.matrix() << "\n";
  // std::cout << "next pose representation:\n" << next_pose.matrix() << "\n";

  // // Publish odometry data
  mrsim::rodom odom;

  // Translational component extraction
  Eigen::Vector2f msg_translation = pose_in_parent.translation();
  float msg_x = msg_translation.x();
  float msg_y = msg_translation.y();

  // Rotational component extraction
  Eigen::Rotation2Df msg_rotation(pose_in_parent.linear());
  float msg_theta = msg_rotation.angle();
  
  odom.x = msg_x;
  odom.y = msg_y;
  odom.theta = msg_theta;
  odom_pub.publish(odom);

}
