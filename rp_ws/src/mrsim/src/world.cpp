// #include <opencv2/highgui.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/imgproc.hpp>

#include "world.h"

World::World(int id) {_id = id;}

void World::loadFromImage(const std::string filename_) {
  cv::Mat frame = cv::imread(filename_);
  if (frame.rows == 0) {
    throw runtime_error("unable to load image");
  }
  cv::cvtColor(frame, display_image, cv::COLOR_BGR2GRAY);
  size = display_image.rows * display_image.cols;
  rows = display_image.rows;
  cols = display_image.cols;
  _grid = std::vector<uint8_t>(size, 0x00);
  memcpy(_grid.data(), display_image.data, size);
  cout << "Image loaded!!!" << endl;
}

void World::draw() {
  for (const auto item : _items) item->draw();
  cv::imshow("Map", display_image);
  memcpy(display_image.data, _grid.data(), size);
}

void World::timeTick(float dt) {
  for (const auto item : _items) {
    item->timeTick(dt);
  }
}

void World::add(WorldItem* item) { _items.push_back(item); }

bool World::traverseBeam(IntPoint& endpoint, const IntPoint& origin,
                         const float angle, const int max_range) {
  Point p0 = origin.cast<float>(); // start point of a beam
  const Point dp(cos(angle), sin(angle)); // point -> (x,y) 
  int range_to_go = max_range;
  while (range_to_go > 0) {
    endpoint = IntPoint(p0.x(), p0.y());
    if (!inside(endpoint)) return false; //beam out of map
    if (at(endpoint) < 127) return true; //beam that hit a object in the map
    p0 = p0 + dp;
    --range_to_go;
  }
  return true;
}

bool World::collides(const IntPoint& p, const int radius) const {
  if (!inside(p)) return true;
  int r2 = radius * radius;
  for (int r = -radius; r <= radius; ++r) {
    for (int c = -radius; c <= radius; ++c) {
      IntPoint off(r, c);
      if (off.squaredNorm() > r2) continue;
      IntPoint p_test = p + IntPoint(r, c);
      if (!inside(p_test)) return true;

      // cerr << "r: " << r << " c: " << c << " val: " << (int) at(p_test) <<
      // endl;
      if (at(p_test) < 127) return true;
    }
  }
  return false;
}

WorldItem::WorldItem(std::shared_ptr<World> w_, 
                    std::string namespace_, const Pose& p_)
    : world(w_), parent(nullptr), 
      pose_in_parent(p_), _namespace(namespace_) {
  if (world) {
    world->add(this);
    cout<< "Added new item" << endl;
  }
}

WorldItem::WorldItem(std::shared_ptr<WorldItem> parent_, 
                    std::string namespace_, const Pose& p)
    : world(parent_->world), parent(parent_), 
      pose_in_parent(p), _namespace(namespace_) {
  if (world) world->add(this);
}

Pose WorldItem::poseInWorld() {
  if (!parent) return pose_in_parent;
  return parent->poseInWorld() * pose_in_parent;
}

WorldItem::~WorldItem() {}
