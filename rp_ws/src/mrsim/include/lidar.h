#pragma once
#include <vector>
#include "world.h"

class Lidar : public WorldItem {

 public:

  //B.F.N: Constructor of a lidar that is attached to the world
  Lidar(float fov_, float max_range_, int num_beams_, std::shared_ptr<World> w,
        const Pose& pose_ = Pose::Identity());

  //B.F.N: Constructor of a lidar that is instead attached to a worlditem
  Lidar(float fov_, float max_range_, int num_beams_,
        std::shared_ptr<WorldItem> p_, const Pose& pose_ = Pose::Identity());

  //B.F.N: Determine what do the lidat the time
  void timeTick(float dt) override;

  //Reppresent the object in the map
  void draw() override;

  //B.F.N: Attributes
  float fov, max_range; // B.F.N: fov = field of view
  int num_beams; // B.F.N:  num of beams launched from the sensor
  std::vector<float> ranges; // B.F.N: range of the beams launched
};