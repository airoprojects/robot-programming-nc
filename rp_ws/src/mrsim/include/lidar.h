#pragma once

#include "types.h"
#include "world.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"


class Lidar : public WorldItem {

 public:

  //B.F.N: Constructor of a lidar that is attached to the world
  Lidar(float fov_, float max_range_, int num_beams_, std::shared_ptr<World> w,
        std::string namespace_, const Pose& pose_ = Pose::Identity());

  //B.F.N: Constructor of a lidar that is instead attached to a worlditem
  Lidar(float fov_, float max_range_, int num_beams_,
        std::shared_ptr<WorldItem> p_, std::string namespace_, 
        const Pose& pose_ = Pose::Identity());

  //B.F.N: Determine what do the lidat the time
  void timeTick(float dt) override;

  //Reppresent the object in the map
  void draw() override;

  //B.F.N: Attributes
  float fov, max_range; // B.F.N: fov = field of view
  int num_beams; // B.F.N:  num of beams launched from the sensor
  std::vector<float> ranges; // B.F.N: range of the beams launched

  
  ros::NodeHandle nh;  // ROS Node Handle
  ros::Publisher scan_pub;  // Publisher to send odometry data
};