#include "robot.h"

Robot::Robot( std::shared_ptr<World> w_,
              string frame_id_, 
              string namespace_,
              float radius_,
              float max_rv_,
              float max_tv_,
              const Pose& pose_, 
              int id_p_)
      : WorldItem(w_, namespace_, frame_id_, pose_), frame_id(frame_id_), 
        radius(radius_), max_rv(max_rv_), max_tv(max_tv_), nh("~"), id_p(id_p_), parent_frame_id(w_->world_frame_id),
        odom_pub(nh.advertise<nav_msgs::Odometry>("/" + namespace_ + "/odom", 10)),
        cmd_vel_sub(nh.subscribe("/" + namespace_ + "/cmd_vel", 10, &Robot::cmdVelCallback, this)) {}

Robot::Robot( std::shared_ptr<WorldItem> parent_,
              string frame_id_,
              string namespace_,
              float radius_,
              float max_rv_,
              float max_tv_,
              const Pose& pose_, 
              int id_p_)
      : WorldItem(parent_, namespace_, frame_id_, pose_), frame_id(frame_id_), 
        radius(radius_), max_rv(max_rv_), max_tv(max_tv_), nh("~"), id_p(id_p_), parent_frame_id(parent_->item_frame_id),
        odom_pub(nh.advertise<nav_msgs::Odometry>("/" + namespace_ + "/odom", 10)),
        cmd_vel_sub(nh.subscribe("/" + namespace_ + "/cmd_vel", 10, &Robot::cmdVelCallback, this)) {}


void Robot::draw() {

  int int_radius = radius * world->i_res;
  IntPoint p = world->world2grid(poseInWorld().translation());
  cv::circle(world->display_image, cv::Point(p.y(), p.x()), int_radius,
             cv::Scalar::all(0), -1);

}

void Robot::timeTick(float dt) {
  Pose motion = Pose::Identity();
  motion.translation() << tv * dt, 0;
  motion.rotate(rv * dt);

  Pose next_pose = pose_in_parent * motion;
  IntPoint ip = world->world2grid(next_pose.translation());
  int int_radius = radius * world->i_res;
  if (!world->collides(ip, int_radius)) pose_in_parent = next_pose;

  // Translational component extraction
  Eigen::Vector2f msg_translation = pose_in_parent.translation();
  float msg_x = msg_translation.x();
  float msg_y = msg_translation.y();

  // Rotational component extraction
  Eigen::Rotation2Df msg_rotation(pose_in_parent.linear());
  float msg_theta = msg_rotation.angle();

  // Publish odometry data
  mrsim::rodom odom;
  odom.x = msg_x;
  odom.y = msg_y;
  odom.theta = msg_theta;
  odom_pub.publish(odom);

  // Call tf to update transformation from robot to world
  // tf2Robot();
}

// Update the robot's velocity based on received commands
void Robot::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  float new_tv = msg->linear.x;
  float new_rv = msg->angular.z; 
  if (new_tv > max_tv) tv = max_tv;
  else tv = new_tv;
  if (new_rv > max_rv) rv = max_rv;
  else rv = new_rv;
}

void Robot::tf2Robot() {

  Pose trasformation = poseInWorld();
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.stamp = ros::Time::now();
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = parent_frame_id;

  tf2::Transform tf_transform;
  tf_transform.setOrigin(tf2::Vector3(trasformation.translation().x(), trasformation.translation().y(), 0.0));

  tf2::Quaternion q;
  q.setRPY(0, 0, atan2(trasformation.rotation()(1,0), trasformation.rotation()(0,0)));  // Imposta roll, pitch e yaw
  tf_transform.setRotation(q);

  // Translation
  transform_stamped.transform.translation.x = tf_transform.getOrigin().x();
  transform_stamped.transform.translation.y = tf_transform.getOrigin().y();
  transform_stamped.transform.translation.z = tf_transform.getOrigin().z();

  // Rotation
  transform_stamped.transform.rotation.x = tf_transform.getRotation().x();
  transform_stamped.transform.rotation.y = tf_transform.getRotation().y();
  transform_stamped.transform.rotation.z = tf_transform.getRotation().z();
  transform_stamped.transform.rotation.w = tf_transform.getRotation().w();

  static tf2_ros::TransformBroadcaster br;
  br.sendTransform(transform_stamped);

}