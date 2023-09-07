#pragma once
#include <vector>

#include "world.h"

class Lidar : public WorldItem {
 public:
  Lidar(float fov_, float max_range_, int num_beams_, std::shared_ptr<World> w,
        const Pose& pose_ = Pose::Identity());

  Lidar(float fov_, float max_range_, int num_beams_,
        std::shared_ptr<WorldItem> p_, const Pose& pose_ = Pose::Identity());

  void timeTick(float dt) override;

  void draw() override;

  float fov, max_range;
  int num_beams;
  std::vector<float> ranges;
};