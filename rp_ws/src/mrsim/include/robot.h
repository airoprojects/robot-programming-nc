#pragma once

#include "types.h"
#include "world.h"

struct Robot : public WorldItem {
  Robot(float radius_, std::shared_ptr<World> w_,
        const Pose& pose_ = Pose::Identity());
  Robot(float radius_, std::shared_ptr<WorldItem> parent_,
        const Pose& pose_ = Pose::Identity());

  void draw() override;
  void timeTick(float dt) override;

  float radius;
  float tv = 0, rv = 0;
};