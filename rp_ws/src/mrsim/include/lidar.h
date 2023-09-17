#pragma once

#include "types.h"
#include "world.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl-1.10/pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h> 

class Lidar : public WorldItem {

 public:

  // B.F.N: Constructor of a lidar that is attached to the world
  Lidar(string frame_id_,
        float fov_, 
        float vfov_, 
        float max_range_, 
        int num_beams_, 
        std::shared_ptr<World> w,
        std::string namespace_, 
        const Pose& pose_ = Pose::Identity(), 
        int id_p = -1);

  // B.F.N: Constructor of a lidar that is instead attached to a worlditem
  Lidar(string frame_id_,
        float fov_, 
        float vfov_, 
        float max_range_, 
        int num_beams_,
        std::shared_ptr<WorldItem> p_, 
        std::string namespace_, 
        const Pose& pose_ = Pose::Identity(), 
        int id_p = -1);

  // Reppresent the object in the map
  void draw() override;

  // B.F.N: Determine what do the lidat the time
  void timeTick(float dt) override;

  // Point cloud conversion
  void pointCloudConversion(const vector<IntPoint3D>& points);

  // Custom tf module from lidar to robot(parent)
  void tf2Lidar();

  // B.F.N: Attributes
  string frame_id;
  float fov, vfov, max_range; // B.F.N: fov = field of view
  int num_beams; // B.F.N:  num of beams launched from the sensor
  vector<float> ranges; // B.F.N: range of the beams launched
  int id_p = -1;
  string parent_frame_id;

  // ROS
  ros::NodeHandle nh;  // ROS Node Handle
  ros::Publisher scan_pub;  // Publisher to send odometry data
};