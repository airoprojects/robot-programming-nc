#include "world.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

// Default constructor for the World class. Currently empty.
World::World() {}

// Method to load an image from a file and initialize the world grid based on that image.
void World::loadFromImage(const std::string filename_) {
  // Read the image from the specified file
  cv::Mat frame = cv::imread(filename_);
  
  // If the image cannot be loaded, throw a runtime error
  if (frame.rows == 0) {
    throw std::runtime_error("unable to load image");
  }
  
  // Convert the image to grayscale
  cv::cvtColor(frame, display_image, cv::COLOR_BGR2GRAY);
  
  // Set the size, rows, and columns based on the dimensions of the loaded image
  size = display_image.rows * display_image.cols;
  rows = display_image.rows;
  cols = display_image.cols;
  
  // Initialize the grid vector with zeros
  _grid = std::vector<uint8_t>(size, 0x00);
  
  // Copy the data from the display image to the grid vector
  memcpy(_grid.data(), display_image.data, size);
}


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

void World::loadFromPCDFile(const std::string& filename_) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename_, *cloud) == -1) {
    PCL_ERROR("Couldn't read the PCD file \n");
    return;
  }

  // Assuming that the point cloud represents a 3D space of size rows x cols x depth
  // You would need to find a way to calculate rows, cols, and depth based on he tpoint cloud data
  // Here, I'm initializing them to arbitrary values as an example
  rows = 100; 
  cols = 100; 
  depth = 100; 
  size = rows * cols * depth;
  _grid.resize(size, 0x00);

  for (const auto& point : cloud->points) {
    int x = static_cast<int>(point.x);
    int y = static_cast<int>(point.y);
    int z = static_cast<int>(point.z);

    if (x >= 0 && x < rows && y >= 0 && y < cols && z >= 0 && z < depth) {
      _grid[cols * rows * x + cols * y + z] = 1; // Setting the grid cell to 1 if a point exists at this location
    }
  }
}


// Method to draw the world and its items
void World::draw() {
  // Draw each item in the world
  for (const auto item : _items) item->draw();
  
  // Display the map image
  cv::imshow("Map", display_image);
  
  // Reset the display image to match the grid data
  memcpy(display_image.data, _grid.data(), size);
}

// Method to update the world and its items for a given time step
void World::timeTick(float dt) {
  for (const auto item : _items) item->timeTick(dt);
}

// Method to add an item to the world
void World::add(WorldItem* item) { 
  _items.push_back(item); 
}

// Method to simulate the traversal of a beam (like a laser scan) in the world
bool World::traverseBeam(IntPoint& endpoint, const IntPoint& origin,
                         const float angle, const int max_range) {
  // Initialize the start point and the direction of the beam
  Point p0 = origin.cast<float>();
  const Point dp(cos(angle), sin(angle));
  int range_to_go = max_range;
  
  // Traverse the beam until it hits an obstacle or reaches the maximum range
  while (range_to_go > 0) {
    // ... (rest of the method)
  }
  // ... (rest of the method)
}

// Method to check if a point collides with an obstacle in the world
bool World::collides(const IntPoint& p, const int radius) const {
  // ... (rest of the method)
}

// Constructor for the WorldItem class, allowing to set a parent world and a pose within that world
WorldItem::WorldItem(std::shared_ptr<World> w_, const Pose& p_)
    : world(w_), parent(nullptr), pose_in_parent(p_) {
  if (world) world->add(this);
}

// Another constructor for the WorldItem class, allowing to set a parent item and a pose relative to that item
WorldItem::WorldItem(std::shared_ptr<WorldItem> parent_, const Pose& p)
    : world(parent_->world), parent(parent_), pose_in_parent(p) {
  if (world) world->add(this);
}

// Method to get the pose of the item in the world coordinates
Pose WorldItem::poseInWorld() {
  // ... (rest of the method)
}

// Destructor for the WorldItem class. Currently empty.
WorldItem::~WorldItem() {}
