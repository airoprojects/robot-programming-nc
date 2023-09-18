#include "lidar.h"
#include <set> // ONLY FOR TEST
Lidar::Lidar( string frame_id_,
              float fov_, 
              float vfov_, 
              float max_range_, 
              int num_beams_,
              std::shared_ptr<World> w,
              std::string namespace_, const Pose& pose_, 
              int id_p_)
      : WorldItem(w, namespace_, frame_id_, pose_),
        frame_id(frame_id_),
        fov(fov_),
        vfov(vfov_),
        max_range(max_range_),
        num_beams(num_beams_),
        ranges(num_beams, -1.0),
        id_p(id_p_),
        parent_frame_id(w->world_frame_id),
        scan_pub(nh.advertise<sensor_msgs::PointCloud2>("/" +namespace_+ "/" +"base_scan", 1000)) {}

Lidar::Lidar( string frame_id_,
              float fov_,
              float vfov_,
              float max_range_, 
              int num_beams_,
              std::shared_ptr<WorldItem> p_,
              std::string namespace_, const Pose& pose_,
              int id_p_)
      : WorldItem(p_, namespace_, frame_id_, pose_),
        frame_id(frame_id_),
        fov(fov_),
        vfov(vfov_),
        max_range(max_range_),
        num_beams(num_beams_),
        ranges(num_beams, -1.0),
        id_p(id_p_),
        parent_frame_id(p_->item_frame_id),
        scan_pub(nh.advertise<sensor_msgs::PointCloud2>("/" +namespace_+ "/" +"base_scan", 1000)) {}

void Lidar::draw() {

  Pose piw = poseInWorld();
  IntPoint origin = world->world2grid(piw.translation());

  if (!world->inside(origin)) return;
  
  float d_alpha = fov / num_beams;
  float alpha = -fov / 2;
  for (int i = 0; i < num_beams; ++i) {
    float r = ranges[i];
    Point p_lidar(r * cos(alpha), r * sin(alpha));
    Point p_world = piw * p_lidar; // report to reference frame world to draw
    IntPoint epi = world->world2grid(p_world);
    cv::line(world->display_image, cv::Point(origin.y(), origin.x()),
             cv::Point(epi.y(), epi.x()), cv::Scalar(127, 127, 127), 1);
    alpha += d_alpha;
  }
}

// B.F.N.: modify the intern of the lidar, then when draw() is call is update on the map its position!
void Lidar::timeTick(float dt) {
  vector<Point3D> lidar_points;
  vector<float> short_ranges;

  Pose piw = poseInWorld();
  IntPoint origin = world->world2grid(piw.translation()); //point of origin of base scan
  if (!world->inside(origin)) return;


  float d_alpha = fov / num_beams; // the angle between each beam
  float alpha = Eigen::Rotation2Df(piw.linear()).angle() - fov / 2; // where we start.
  
  float d_beta = vfov / num_beams; // increment for beta
  float beta = 0; // start from the bottom
  float int_range = max_range * world->i_res; // (from world to grid)

  for (int i = 0; i < num_beams; ++i) {
    IntPoint endpoint;
    ranges[i] = max_range; //each beam 
    int result = world->traverseBeam(endpoint, origin, alpha, int_range);  
    // to trasform endpoint with respect the lidar frame 
    if (result > -1) {
      IntPoint delta = endpoint - origin; // point where beam arrives
      if (result > 0) {
        Point endpoint_lidar = (piw.inverse() * (endpoint.cast<float>() * world -> res)).cast<float>();
        lidar_points.push_back(Point3D(endpoint_lidar.x(), endpoint_lidar.y(), 0.0));
        short_ranges.push_back(delta.norm() * world->res); // TO REMAIN PIXEL
      }
      ranges[i] = delta.norm() * world->res; // FROM GRID TO WORLD
    }
    alpha += d_alpha; // from sx is the first beam, after i move to right with d_alpha 
  }

  
  // At the end of this loop we should have all the ranges for all the beams
  int i  = 0;
  set<float> z_test;
  for (const auto hit_point: lidar_points) {
    int num_points=0;
    beta = d_beta;
    float ex = hit_point.x();
    float ey = hit_point.y();
    while (beta <= vfov) {
      float diag_beam = short_ranges[i] / cos(beta);
      float z_coordinate = diag_beam * sin(beta);
      lidar_points.push_back(Point3D(ex, ey, z_coordinate));
      num_points++;
      beta += d_beta;
    }
    i++;
  }
  // cout << "num of points to disp: " << foo.size() << endl;

  // LC: create point cloud and brodcast lidar transformations
  pointCloudConversion(lidar_points);
  tf2Lidar();
}

// This function converts a set of 3D Int point int a point cloud 
void Lidar::pointCloudConversion(const vector<Point3D>& points) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.header.frame_id = frame_id; 
  cloud.is_dense = true;

  for (const auto& point : points) {
    pcl::PointXYZ pcl_point;
    pcl_point.x = point(0);
    pcl_point.y = point(1);
    pcl_point.z = point(2);
    cloud.points.push_back(pcl_point);
  }
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(cloud, output);
  output.header.frame_id = parent_frame_id;
  output.header.stamp = ros::Time::now();

  // Publish on ros topic /robot_i/base_scan
  scan_pub.publish(output);

};

void Lidar::tf2Lidar() {
  // Fixed transfomation from lidar to robot
  static tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped transform_stamped;

  transform_stamped.header.frame_id = parent_frame_id;
  transform_stamped.child_frame_id = frame_id;

  transform_stamped.transform.translation.x = 0.0;
  transform_stamped.transform.translation.y = 0.0;
  transform_stamped.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, 0);

  transform_stamped.transform.rotation.x = q.x();
  transform_stamped.transform.rotation.y = q.y();
  transform_stamped.transform.rotation.z = q.z();
  transform_stamped.transform.rotation.w = q.w();

  transform_stamped.header.stamp = ros::Time::now();
  tfb.sendTransform(transform_stamped);
}
