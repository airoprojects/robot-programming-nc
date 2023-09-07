#pragma once

#include "types.h"
#include "world.h"

// Definitiopn of robot struct that extend world item
struct Robot : public WorldItem {
  
  // Robot constructor that takes as input a word to which the robot belong
  Robot(float radius_, std::shared_ptr<World> w_,
        const Pose& pose_ = Pose::Identity());
  
  // Robot contructor that takes as input a world item to which the robot belong
  Robot(float radius_, std::shared_ptr<WorldItem> parent_,
        const Pose& pose_ = Pose::Identity());

  // method to dray the robot on the map with open cv
  void draw() override;

  // method to update the robot pose in function of the velocity
  void timeTick(float dt) override;

  // robot dadious and initial translational and rotational velocity
  float radius;
  float tv = 0, rv = 0;
};    