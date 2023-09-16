#include "robot.h"

Robot::Robot(float radius_, std::shared_ptr<World> w_,
            string namespace_, string frame_id_, const Pose& pose_, int id_p_)
    : radius(radius_), WorldItem(w_, namespace_,  pose_), tv(0.0), rv(0.0), nh("~"), id_p(id_p_),
      odom_pub(nh.advertise<nav_msgs::Odometry>("/" + namespace_ + "/odom", 10)),
      cmd_vel_sub(nh.subscribe("/" + namespace_ + "/cmd_vel", 10, &Robot::cmdVelCallback, this)),
      frame_id(frame_id_) {}

Robot::Robot(float radius_, std::shared_ptr<WorldItem> parent_,
            string namespace_, string frame_id_, const Pose& pose_, int id_p_)
    : radius(radius_), WorldItem(parent_, namespace_, pose_), tv(0.0), rv(0.0), nh("~"), id_p(id_p_),
      odom_pub(nh.advertise<nav_msgs::Odometry>("/" + namespace_ + "/odom", 10)),
      cmd_vel_sub(nh.subscribe("/" + namespace_ + "/cmd_vel", 10, &Robot::cmdVelCallback, this)),
      frame_id(frame_id_) {}


// Update the robot's velocity based on received commands
void Robot::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    tv = msg->linear.x;
    rv = msg->angular.z;
}

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

  // std::cout << "pose in parent robot:\n" << pose_in_parent.matrix() << "\n";
  // std::cout << "motion representation:\n" << motion.matrix() << "\n";
  // std::cout << "next pose representation:\n" << next_pose.matrix() << "\n";

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
  customTf2();
}

void Robot::customTf2() {

  // FIXED TRANSFOMATION TUTORIAL
  // static tf2_ros::TransformBroadcaster tfb;
  // geometry_msgs::TransformStamped transformStamped;
  // transformStamped.header.frame_id = "map";
  // transformStamped.child_frame_id = frame_id;
  // transformStamped.transform.translation.x = 0.0;
  // transformStamped.transform.translation.y = 2.0;
  // transformStamped.transform.translation.z = 0.0;
  // tf2::Quaternion q;
  // q.setRPY(0, 0, 0);
  // transformStamped.transform.rotation.x = q.x();
  // transformStamped.transform.rotation.y = q.y();
  // transformStamped.transform.rotation.z = q.z();
  // transformStamped.transform.rotation.w = q.w();
  // transformStamped.header.stamp = ros::Time::now();
  // tfb.sendTransform(transformStamped);
  // END

  Pose trasformation = poseInWorld();
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.stamp = ros::Time::now();
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = "map";

  tf2::Transform tf_transform;

  tf_transform.setOrigin(tf2::Vector3(trasformation.translation().x(), trasformation.translation().y(), 0.0));

  tf2::Quaternion q;
  q.setRPY(0, 0, atan2(trasformation.rotation()(1,0), trasformation.rotation()(0,0)));  // Imposta roll, pitch e yaw
  tf_transform.setRotation(q);

  // tf_transform.setRotation(tf2::Quaternion(0, 0, atan2(trasformation.rotation()(1,0), trasformation.rotation()(0,0))));

  // Impostare la traslazione
  transform_stamped.transform.translation.x = tf_transform.getOrigin().x();
  transform_stamped.transform.translation.y = tf_transform.getOrigin().y();
  transform_stamped.transform.translation.z = tf_transform.getOrigin().z();

  // Impostare la rotazione
  transform_stamped.transform.rotation.x = tf_transform.getRotation().x();
  transform_stamped.transform.rotation.y = tf_transform.getRotation().y();
  transform_stamped.transform.rotation.z = tf_transform.getRotation().z();
  transform_stamped.transform.rotation.w = tf_transform.getRotation().w();

  static tf2_ros::TransformBroadcaster br;
  br.sendTransform(transform_stamped);

}