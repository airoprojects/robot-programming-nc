#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include "types.h"
#include <string>
#include <vector>

struct WorldItem;



class World {
  public:


  World();

  //INLINE FUNCTIONS
  //the point in 3d is translate in the position in the 1d vector that represents the grid
  inline uint8_t& at(const IntPoint& p) { return _grid[cols * rows * p.z() + cols * p.x() + p.y()];  } 
  inline uint8_t at(const IntPoint& p) const { return _grid[cols * rows * p.z() + cols * p.x() + p.y()]; }

  // IntPoint is a point inside the grid -> so vector of integer
  // Point is a point inside the "real world" so vector of float
  inline IntPoint world2grid(const Point& p) {return IntPoint(p.x()*i_res, p.y()*i_res, p.z()*i_res);}
  inline Point grid2world(const IntPoint& p) {return Point(p.x()*res, p.y()*res, p.z()*res);}


  inline bool inside(const IntPoint& p) const {return p.x() >= 0 && p.y() >= 0 && p.z() >= 0 && p.x() < rows && p.y() < cols && p.z() < depth;}
  

  //FUNCTIONS TO IMPLEMENT 
  bool collides(const IntPoint& p, const int radius) const;
  bool traverseBeam(IntPoint& endpoint, const IntPoint& origin, const float azimuth, const float elevation, const int max_range); // in 3d to launch a beam you need 2 angle (vertical, horizontal)
  bool collides(const IntPoint& p, const int radius) const;

  void loadFromImage(const std::string filename_); // a function that given a filename setup word member?
  void draw(); // Q: the world draw itself? -> the world should call the draw function of each items in it. It also should show aq depth map of the map
  void timeTick(float dt); 
  void add(WorldItem* item);


  //CONSTANTS
  static constexpr int MAX_ITEMS=100;

  //MEMBER OF CLASS
  unsigned int rows = 0, cols = 0, depth = 0; 
  unsigned int size = 0; 
  float res = 0.05, i_res = 20.0;

  cv::Mat display_image; // Image for visualization (might need updating for 3D context) ATTENTION!

  protected:
    // 1D vector representing the grid
    std::vector<uint8_t> _grid;

    // List of items present in the world
    std::vector<WorldItem*> _items;
};


class WorldItem{
  public:
  
  WorldItem(std::shared_ptr<World> w_, const Pose& p_= Pose::Identity());
  WorldItem(std::shared_ptr<WorldItem> parent_, const Pose& p_ = Pose::Identity());
  ~WorldItem();

  Pose poseInWorld();

  virtual void draw() = 0;
  virtual void timeTick(float dt) = 0;

  std::shared_ptr<World> world = nullptr;
  std::shared_ptr<WorldItem> parent = nullptr;
  Pose pose_in_parent;

};
