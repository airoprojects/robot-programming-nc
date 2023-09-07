#include <ros/ros.h>

#include <opencv2/highgui.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "mrsim_node");
  ros::NodeHandle nh("/");

  // Load the configuration file and initialize the simulator

  while (ros::ok()) {
    // run a simulation iteration
    // visualize the simulation
    cv::waitKey(100);
    ros::spinOnce();
  }

  return 0;
}