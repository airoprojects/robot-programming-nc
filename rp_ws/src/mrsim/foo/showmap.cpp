#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv) {
  // Initialize the ROS Node
  ros::init(argc, argv, "pcd_to_pointcloud");

  // Create a ROS NodeHandle
  ros::NodeHandle nh;

  // Create a ROS Publisher to publish the PointCloud data
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);

  // Load the PCD file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/leeoos/Projects/robot-programming-nc/map3D/map3D.pcd", *cloud) == -1) {
    PCL_ERROR("Couldn't read the PCD file \n");
    return (-1);
  }

  // Convert the PCL PointCloud to a ROS PointCloud2 message
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);

  // Set the frame ID to the appropriate frame for your setup
  //output.header.frame_id = "base_link"; 
  output.header.frame_id = "map";

  // Create a ROS Rate object to control the loop rate
  ros::Rate loop_rate(1);

  while (ros::ok()) {
    // Publish the PointCloud2 message
    pcl_pub.publish(output);

    // Spin once to handle callbacks and wait for the next iteration
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
