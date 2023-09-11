#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "types.h"
#include <iostream>

// Here WorldItem is used to avoid cyclic dependency at compile time
struct WorldItem;


class World {

public:

  World(int id);

  inline uint8_t& at(const IntPoint& p) { 
    return _grid[cols * p.x() + p.y()]; 
  }

  inline uint8_t at(const IntPoint& p) const {
    return _grid[cols * p.x() + p.y()];
  }

  bool collides(const IntPoint& p, const int radius) const;

  inline IntPoint world2grid(const Point& p) {
    return IntPoint(p.x() * i_res, p.y() * i_res);
  }

  inline Point grid2world(const IntPoint& p) {
    return Point(p.x() * res, p.y() * res);
  }

  inline bool inside(const IntPoint& p) const {
    return p.x() >= 0 && p.y() >= 0 && p.x() < rows && p.y() < cols;
  }

  void loadFromImage(const std::string filename_);

  bool traverseBeam(IntPoint& endpoint, const IntPoint& origin,
                    const float angle, const int max_range);

  void draw();

  void timeTick(float dt);

  void add(WorldItem* item);

  unsigned int rows = 0, cols = 0;
  unsigned int size = 0;
  float res = 0.05, i_res = 20.0;
  int _id = 0;

  cv::Mat display_image;

protected:

  std::vector<uint8_t> _grid;
  std::vector<WorldItem*> _items;
};

class WorldItem {

public:
 
  WorldItem(std::shared_ptr<World> w_,
            std::string namespace_,
            const Pose& p_ = Pose::Identity());
  WorldItem(std::shared_ptr<WorldItem> parent_,
              std::string namespace_,
            const Pose& p_ = Pose::Identity());
  ~WorldItem();

  Pose poseInWorld();

  virtual void draw() = 0;
  virtual void timeTick(float dt) = 0;

  std::shared_ptr<World> world = nullptr;
  std::shared_ptr<WorldItem> parent = nullptr;
  Pose pose_in_parent;
  std::string namespace_ = "";
};