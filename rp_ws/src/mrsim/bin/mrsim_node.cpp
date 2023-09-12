#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <jsoncpp/json/json.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sys/time.h>

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


int main(int argc, char** argv) {

  if (argc < 2) {
    cerr << "Error: no config.jsosn file provided" << endl;
    return 1;
  }

  ros::init(argc, argv, "mrsim_node");
  ros::NodeHandle nh("/");

  // LC: get git root directory path
  string git_root_path;
  try {
        git_root_path = getGitRootPath();
        //cerr << "Git Root Path: " << git_root_path << '\n';
    } catch(const runtime_error& e) {
        cerr << "Exception: " << e.what() << '\n';
  }

  // Load the configuration file and initialize the simulator
  // TODO :
  /*
    0. Initialize a instance of world
    1. Import json config file                                                DONE
    2. Read json config file and extract:                                     DONE
      2.1 NUM_ROBOTS: number of robots in the simulation (N)  
      2.2 NUM_LIDARS: number of lidars in the simulation (M)  
      2.3 INFO about robots                                                   DONE
      2.4 INFO about lidars                                                   DONE
    3. Make a launch file to run N robot_nodes and M lidars_nodes             DONE
    4. Launch the launch file
    5. Initialize an array of publisher objects for each robot to 
       allow mrsim_node to publish on specific topics for each robot
  */

  string config_path = git_root_path + "/config/" + argv[1];
  Json::Value root = readJson(config_path);
  string map = root["map"].asString();
  cout << "Map -> " << map << endl;
  string image_path = git_root_path + "/map/" +  map;

  // LC: pointer new instance of World
  World world(42); 
  shared_ptr<World> world_pointer(&world, [](World*){ });   // is a lambda function 

  world.loadFromImage(image_path); //THE MOST STUPID FUNCTION IN THE UNIVERSE, BASTARD FUNCTION.
  // world_pointer->draw();
  // cv::waitKey(0);
   cout << "rows " << world.rows << endl;
   cout << "cols " << world.cols << endl;
  
  int NUM_ROBOTS = 1;
  //vector<RobotPointer> robots_and_lidars =  initSimEnv(root, world_pointer, NUM_ROBOTS);

  // LC: run opkey controller node 
  // string command = "rosrun mrsim opkey_node " + to_string(NUM_ROBOTS);
  string command = "gnome-terminal -- bash -c 'rosrun mrsim opkey_node " + to_string(NUM_ROBOTS) + " ; exec bash'";
  int result = system(command.c_str());
  if (result != 0) {
    ROS_ERROR("Failed to execute roslaunch command");
    return 1;
  }

  // LC: make a vector of publishers
  // vector<ros::Publisher> publishers_vector;
  // for (int i=0; i < NUM_ROBOTS; i++) {
  //   ros::Publisher foo_pub  = nh.advertise<geometry_msgs::Twist>("robot_" + to_string(i) + "/cmd_vel", 1000);
  //   publishers_vector.push_back(foo_pub);
  // }

  double radius = 0.5;
  IntPoint middle(world.rows/2, world.cols/2);
  Pose robot_pose;
  robot_pose.translation() = world.grid2world(middle);
  Robot r(radius, world_pointer, "robot_0", robot_pose);
  cout << r.world << endl;

  for (const auto robot: world._items) {cout << robot->_namespace;}
  cout << "World items: " << world._items.size() << endl;

  cout << "Running primary node" << endl;

  // LC: no robot is selected to be controlled at the beginning
  bool select_robot = true; 
  int robot_index = -1;
  float delay = 0.8;

  // LC: key press actions log
  ofstream keylog("./key.log");
  ros::Rate rate(10);

  while (ros::ok()) {

    //  // LC: Select the index of the robot you wnat to control
    // if (select_robot) {

    //   // This should be temporary, just to test key captures
    //   // cv::namedWindow("Window");
    //   cv::destroyAllWindows();

    //   while (true) {

    //     std::cout << "\nWhat robot do you want to control? " << std::endl; 
    //     std::cout << "Type a numebr beween 0 and " << NUM_ROBOTS-1 << ": ";
    //     std::cin >> robot_index;

    //     // LC: check for user error in the input
    //     if (std::cin.fail() || robot_index < 0 || robot_index > NUM_ROBOTS-1) {
    //       std::cin.clear(); // reset the fail state
    //       std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // discard invalid input
    //       std::cout << "Invalid input. Please try again." << std::endl;
    //     } 
    //     else {
    //       std::cout << "\nControlling robot " << robot_index << "\n" << std::endl;
    //       std::cout << "Press 'c' to change robot\n" << std::endl;
    //       std::cout << "Press 'ESC' to exit the simulation\n" << std::endl;
    //       break;
    //     }
    //   }
    //   // LC: reset the index to keep controlling the same robot
    //   select_robot = false;
    // }
    // cerr << "duration" << t_end-t_start << endl;
    // cerr << "image_size: " << world.rows << " " << world.cols << endl;
    // LC: draw world
    world.draw();
    // r.draw();
    cv::waitKey(0);

    double t_start=timeMillisec();
    world.timeTick(delay);
    double t_end=timeMillisec();

    // // LC: message definition
    // geometry_msgs::Twist msg;
    // msg.linear.x = 1.0;
    // msg.angular.z = 1.0;

    // // Switch case to control robot motion
    // int k = cv::waitKey(0);
    // keylog << "\nKey pressed with decimal value: " << k ;
    // switch (k) {
    //     case 81: r.rv+=0.05; keylog << " left\n"; msg.angular.z = 0.5;; break; // arow left
    //     case 82: r.tv+=1.0; keylog << " up\n"; msg.linear.x = 1.0; break; // arow up
    //     case 83: r.rv-=0.05; keylog << " right\n"; msg.angular.z = -0.5; break; // arow right
    //     case 84: r.tv-=1.0; keylog << " down\n"; msg.linear.x = -1.0; ; break; // arow down
    //     case 99: select_robot = true; break; // c key
    //     case 27: keylog << " esc\n"; return 0; // esc
    //     default: break;
    // }

    // if (!select_robot) {

    //   // publishers_vector[robot_index].publish(msg);
    //   // ros::spinOnce();
    //   // r.timeTick(delay);

    //   // for (const auto robot: robots_and_lidars) {
    //   //   // robot->timeTick(delay);
    //   //   // robot->draw();
    //   // }
    // }
    // LC: or change robot

  }
  cv::destroyAllWindows();
  keylog.close();
  return 0;
}