#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "types.h"
#include <iostream>

// Here WorldItem is used to avoid cycle dependency at compile time
struct WorldItem;

// Definition of class World. It is responsable of managing the whole simulation
class World {
 public:
  World(int id);

  // LC: function to insert 2d coordinates into 1D grid points
  inline uint8_t& at(const IntPoint& p) { 
    return _grid[cols * p.x() + p.y()]; 
  }

  inline uint8_t at(const IntPoint& p) const {
    return _grid[cols * p.x() + p.y()];
  }

  // LC: collision detection function
  bool collides(const IntPoint& p, const int radius) const;

  // LC: conversion from elements of the world to elements of the pixel grid and viceversa
  inline IntPoint world2grid(const Point& p) {
    return IntPoint(p.x() * i_res, p.y() * i_res);
  }

  inline Point grid2world(const IntPoint& p) {
    std::cout <<"here7.1 " << p.x() << " and " << p.y() << std::endl;
    return Point(p.x() * res, p.y() * res);
  }

  // LC: boolean function to check if a point is inside the world
  inline bool inside(const IntPoint& p) const {
    return p.x() >= 0 && p.y() >= 0 && p.x() < rows && p.y() < cols;
  }

  // LC: load image from a file
  void loadFromImage(const std::string filename_);

  // LC: return true if beam has touched someting
  bool traverseBeam(IntPoint& endpoint, const IntPoint& origin,
                    const float angle, const int max_range);

  // LC: a function to draw the map via opencv
  void draw();

  // LC: a function to update the status of the world ad it's items douring the simulation
  void timeTick(float dt);

  // LC a function to add Items to the world
  void add(WorldItem* item);

  // LC: world grid dimentios and resolution scale
  unsigned int rows = 0, cols = 0;
  unsigned int size = 0;
  float res = 0.05, i_res = 20.0;

  cv::Mat display_image;

 protected:

  // LC: referance pixel grid
  std::vector<uint8_t> _grid;

  // LC: list of items in the world
  std::vector<WorldItem*> _items;
};

// LC: definition of WorldItem class to manage items in the world
class WorldItem {

 public:
 
  // LC: constructors of world items
  WorldItem(std::shared_ptr<World> w_, const Pose& p_ = Pose::Identity());
  WorldItem(std::shared_ptr<WorldItem> parent_,
            const Pose& p_ = Pose::Identity());
  ~WorldItem();

  Pose poseInWorld();

  virtual void draw() = 0;
  virtual void timeTick(float dt) = 0;

  std::shared_ptr<World> world = nullptr;
  std::shared_ptr<WorldItem> parent = nullptr;
  Pose pose_in_parent;
  int check = 10;
};