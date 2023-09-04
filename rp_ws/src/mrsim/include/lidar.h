#pragma once
#include "world.h"

class Lidar: public WorldItem {
public:
  Lidar(float fov_, float max_range_, int num_beams_, World* w, const Pose& pose_=Pose());

  Lidar(float fov_, float max_range_, int num_beams_, WorldItem* p_, const Pose& pose_=Pose());

  ~Lidar();

  void timeTick(float dt);

  void draw();
  
  float fov, max_range;
  int num_beams;
  float *ranges;
};