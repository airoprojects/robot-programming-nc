#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, *r)
{
  ROS_INFO("Received cmd_vel: linear.x: [%f], angular.z: [%f]", msg->linear.x, msg->angular.z);
  r.tv = sqrt(pow(msg->linear.x,2) + pow(msg->linear.x,2))
}

int main(int argc, char **argv) {

  //to do:
    // read the params from argv (robot)
    // 

  // argv[robot_name, *w]
  // tmp def of robot
  
  IntPoint middle(w.rows/2, w.cols/2);
  Pose robot_pose;
  robot_pose.translation = w.grid2world(middle);
  // create new node ()
  //Robot r(0.3, &w, robot_pose);


  // Initialize the node with a name, in this case 'cmd_vel_listener'
  ros::init(argc, argv, "cmd_vel_listener");
  
  // Create a node handle to interact with ROS system
  ros::NodeHandle n;

  // Create a subscriber object subscribing to the 'cmd_vel' topic, with the callback function 'cmdVelCallback'
  ros::Subscriber sub = n.subscribe(argv[0]+"cmd_vel", 1000, cmdVelCallback);
  
  // Continuously pump callbacks until the node is shutdown
  ros::spin();

  return 0;
}
