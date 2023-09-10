#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, *r)
{
  ROS_INFO("Received cmd_vel: linear.x: [%f], angular.z: [%f]", msg->linear.x, msg->angular.z);
  r.tv = sqrt(pow(msg->linear.x,2) + pow(msg->linear.x,2))
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "robot_node");
  ros::NodeHandle nh;

  // get node namespace
  std::string namespace;
  nh.getParam("namespace", namespace);

  ROS_INFO("Parameter Value: %s", namespace.c_str());

  // Create a subscriber object subscribing to the 'cmd_vel' topic, with the callback function 'cmdVelCallback'
  ros::Subscriber sub = n.subscribe(namespace+"cmd_vel", 1000, cmdVelCallback);
  
  // Continuously pump callbacks until the node is shutdown
  ros::spin();

  return 0;
}
