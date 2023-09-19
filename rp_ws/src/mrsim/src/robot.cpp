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
  tf2Robot();
  staticTransform();
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
  transform_stamped.header.frame_id = parent_frame_id;
  transform_stamped.child_frame_id = frame_id;

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

void Robot::staticTransform() {
  geometry_msgs::TransformStamped static_transformStamped1;
  static_transformStamped1.header.stamp = ros::Time::now();
  static_transformStamped1.header.frame_id = frame_id;
  static_transformStamped1.child_frame_id = "base_link";
  static_transformStamped1.transform.translation.x = 0.0;
  static_transformStamped1.transform.translation.y = 0.0;
  static_transformStamped1.transform.translation.z = 0.0;
  tf2::Quaternion quat1;
  quat1.setRPY(0, 0, 0);
  static_transformStamped1.transform.rotation.x = quat1.x();
  static_transformStamped1.transform.rotation.y = quat1.y();
  static_transformStamped1.transform.rotation.z = quat1.z();
  static_transformStamped1.transform.rotation.w = quat1.w();

  geometry_msgs::TransformStamped static_transformStamped2;
  static_transformStamped2.header.stamp = ros::Time::now();
  static_transformStamped2.header.frame_id = frame_id;
  static_transformStamped2.child_frame_id = "left_wheel";
  static_transformStamped2.transform.translation.x = 1.0;
  static_transformStamped2.transform.translation.y = 1.0;
  static_transformStamped2.transform.translation.z = 0.0;
  tf2::Quaternion quat2;
  quat2.setRPY(0, 0, 0);
  static_transformStamped2.transform.rotation.x = quat2.x();
  static_transformStamped2.transform.rotation.y = quat2.y();
  static_transformStamped2.transform.rotation.z = quat2.z();
  static_transformStamped2.transform.rotation.w = quat2.w();

  geometry_msgs::TransformStamped static_transformStamped3;
  static_transformStamped3.header.stamp = ros::Time::now();
  static_transformStamped3.header.frame_id = frame_id;
  static_transformStamped3.child_frame_id = "right_wheel";
  static_transformStamped3.transform.translation.x = -1.0;
  static_transformStamped3.transform.translation.y = -1.0;
  static_transformStamped3.transform.translation.z = 0.0;
  tf2::Quaternion quat3;
  quat3.setRPY(0, 0, 0);
  static_transformStamped3.transform.rotation.x = quat3.x();
  static_transformStamped3.transform.rotation.y = quat3.y();
  static_transformStamped3.transform.rotation.z = quat3.z();
  static_transformStamped3.transform.rotation.w = quat3.w();

  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  static_broadcaster.sendTransform(static_transformStamped1);
  static_broadcaster.sendTransform(static_transformStamped2);
  static_broadcaster.sendTransform(static_transformStamped3);
}