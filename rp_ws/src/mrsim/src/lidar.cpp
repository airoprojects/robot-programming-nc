#include "lidar.h"

Lidar::Lidar(float fov_, float vfov_, float max_range_, int num_beams_,
             std::shared_ptr<World> w,
             std::string namespace_, const Pose& pose_, int id_p_)
    : WorldItem(w, namespace_, pose_),
      fov(fov_),
      vfov(vfov_),
      max_range(max_range_),
      num_beams(num_beams_),
      ranges(num_beams, -1.0),
      id_p(id_p_),
      scan_pub(nh.advertise<sensor_msgs::LaserScan>("/" +namespace_+ "/" +"base_scan", 1000)) {}

Lidar::Lidar(float fov_,float vfov_, float max_range_, int num_beams_,
             std::shared_ptr<WorldItem> p_,
             std::string namespace_, const Pose& pose_, int id_p_)
    : WorldItem(p_, namespace_, pose_),
      fov(fov_),
      vfov(vfov_),
      max_range(max_range_),
      num_beams(num_beams_),
      ranges(num_beams, -1.0),
      id_p(id_p_),
      scan_pub(nh.advertise<sensor_msgs::LaserScan>("/" +namespace_+ "/" +"base_scan", 1000)) {}

// modify the intern of the lidar, then when draw() is call is update on the map its position!
void Lidar::timeTick(float dt) {
  vector<IntPoint3D> lidar_points;

  Pose piw = poseInWorld();
  IntPoint origin = world->world2grid(piw.translation()); //point of origin of base scan
  if (!world->inside(origin)) return;

  float d_alpha = fov / num_beams; // the angle between each beam
  cout << "d alpha: " << d_alpha << endl;

  float alpha = Eigen::Rotation2Df(piw.linear()).angle() - fov / 2; // where we start.

  float d_beta = vfov / num_beams; // increment for beta
  cout << "d beta: " << d_beta  << endl << "vfov: " << vfov<< endl;
  float beta = 0; // start from the bottom
  float int_range = max_range * world->i_res; // (from world to grid)

  
  for (int i = 0; i < num_beams; ++i) {
    IntPoint endpoint;
    ranges[i] = max_range; //each beam 
    bool result = world->traverseBeam(endpoint, origin, alpha, int_range);  
    if (result) {
      IntPoint delta = endpoint - origin; // point where beam arrives
      lidar_points.push_back(IntPoint3D(endpoint.x(), endpoint.y(), 0));
      ranges[i] = delta.norm() * world->res; // from grid to world
    }
    alpha += d_alpha; // from sx is the first beam, after i move to right with d_alpha 
  }

  int counter = 0;
  int stop;
  // at the end of this loop we should have all the ranges for all the beams
  for (int i = 0; i < num_beams; ++i) {
    counter++;
    if (counter > 5) cin >> stop;
    beta += d_beta;
    int ex = lidar_points[i].x();
    int ey = lidar_points[i].y();
    while (beta <= vfov) {
      cout << "I am here and beta is " << beta << endl; 
      float diag_beam = ranges[i] / cos(beta);
      float z_coordinate = diag_beam * sin(beta);
      lidar_points.push_back(IntPoint3D(ex, ey, z_coordinate));
      beta += d_beta;
    }
  }

  for (const auto value: lidar_points) {
    if (value.z() < 0) cout << "ERRORE Z NEGATIVOOOOO";
    cout << "x: " << value.x() << endl
    << "y: " << value.y() << endl
    << "z: " << value.z() << endl;
  }

  // TO IMPLEMENTS MESSAGGE
  // Create a LaserScan message
  // sensor_msgs::LaserScan scan_msg;
  // scan_msg.angle_min = -fov / 2;
  // scan_msg.angle_max = fov / 2;
  // scan_msg.angle_increment = fov / num_beams;
  // scan_msg.range_min = 0.0;
  // scan_msg.range_max = max_range;
  // scan_msg.ranges = ranges;
  // lidar_publisher.publish(scan_msg);

}

void Lidar::draw() {

  Pose piw = poseInWorld();
  // std::cout << "pose in parent lidar:\n" << piw.matrix() << "\n";
  IntPoint origin = world->world2grid(piw.translation());



  if (!world->inside(origin)) return;
  

  float d_alpha = fov / num_beams;
  float alpha = -fov / 2;
  for (int i = 0; i < num_beams; ++i) {
    float r = ranges[i];
    Point p_lidar(r * cos(alpha), r * sin(alpha));
    Point p_world = piw * p_lidar;
    IntPoint epi = world->world2grid(p_world);
    cv::line(world->display_image, cv::Point(origin.y(), origin.x()),
             cv::Point(epi.y(), epi.x()), cv::Scalar(127, 127, 127), 1);
    alpha += d_alpha;
  }
 

};
