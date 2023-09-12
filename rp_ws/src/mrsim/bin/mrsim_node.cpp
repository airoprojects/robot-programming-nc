#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <jsoncpp/json/json.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include <thread>
#include <chrono>
#include <cmath>

// Custom lib
#include "types.h"
#include "world.h"
#include "robot.h"
#include "lidar.h"
#include "utils.h"

double timeMillisec() {
  struct timeval tv;
  gettimeofday(&tv,0);
  return tv.tv_sec*1000+tv.tv_usec*1e-3;
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // Update the robot's velocity based on received commands
    cout << "mrsim callback" << endl;
    // tv = msg->linear.x;
    // rv = msg->angular.z;
    // cout << "tv " << tv << endl;
    // cout << "rv " << rv << endl;
}


int main(int argc, char** argv) {

  if (argc < 2) {
    cerr << "Error: no config.jsosn file provided" << endl;
    return 1;
  }

  ros::init(argc, argv, "mrsim_node");
  ros::NodeHandle nh("/");

  ros::Subscriber cmd_vel_sub(nh.subscribe("/robot_0/cmd_vel", 10, cmdVelCallback));


  // LC: get git root directory path
  string git_root_path;
  try {
        git_root_path = getGitRootPath();
        //cerr << "Git Root Path: " << git_root_path << '\n';
    } catch(const runtime_error& e) {
        cerr << "Exception: " << e.what() << '\n';
  }

  // Load configuration file
  string config_path = git_root_path + "/config/" + argv[1];
  Json::Value root = readJson(config_path);
  string map = root["map"].asString();
  cout << "Map -> " << map << endl;
  string image_path = git_root_path + "/map/" +  map;

  // LC: new instance of World
  World world(42); 
  shared_ptr<World> world_pointer(&world, [](World*){ });   // is a lambda function 

  world.loadFromImage(image_path); //THE MOST STUPID FUNCTION IN THE UNIVERSE, BASTARD FUNCTION.
 
  // debug 
  cout << "rows " << world.rows << endl;
  cout << "cols " << world.cols << endl;
  
  int NUM_ROBOTS = 1;
  //vector<RobotPointer> robots_and_lidars =  initSimEnv(root, world_pointer, NUM_ROBOTS);

  // LC: run opkey controller node 
  string command = "gnome-terminal -- bash -c 'rosrun mrsim opkey_node " + to_string(NUM_ROBOTS) + " ; exec bash'";
  int result = system(command.c_str());
  if (result != 0) {
    ROS_ERROR("Failed to execute roslaunch command");
    return 1;
  }

  double radius = 0.5;
  // IntPoint middle(world.rows/2, world.cols/2);
  // Pose robot_pose;
  // robot_pose.translation() = world.grid2world(middle);
  
  Pose nuova_pose = Pose::Identity();

  // Definire una traslazione
  nuova_pose.translation() = world.grid2world(Eigen::Vector2i(world.rows/2, world.cols/2));

  // Definire una rotazione
  nuova_pose.linear() = Eigen::Rotation2Df(M_PI/4).matrix();

  Robot r(radius, world_pointer, "robot_0", nuova_pose);
  cout << r.world << endl;
  std::cout << "pose in parent:\n" << r.pose_in_parent.matrix() << "\n";

  for (const auto robot: world._items) {cout << robot->_namespace << endl;}
  cout << "World items: " << world._items.size() << endl;

  cout << "Running primary node" << endl;

  // LC: no robot is selected to be controlled at the beginning
  bool select_robot = true; 
  int robot_index = -1;
  float delay = 0.08;

  // LC: key press actions log
  ofstream keylog("./key.log");
  ros::Rate loop_rate(10);

  while (ros::ok()) {

    // LC: Select the index of the robot you wnat to control
   
    world.draw();
  
    int k = cv::waitKey(1);
    if (k == 27) break;

    ros::spinOnce();

    world.timeTick(delay);

    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // dorme per 100 millisecondi

  }
  // cv::destroyAllWindows();
  // keylog.close();
  return 0;
}