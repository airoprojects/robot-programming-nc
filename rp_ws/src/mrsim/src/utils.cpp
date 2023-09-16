#include "utils.h"

Json::Value readJson( string in_path) {
  
  cout << "Configuration file path:" << in_path <<  endl;
  Json::Value root; // B.F.N object to hold the parsed json file
  Json::CharReaderBuilder readerBuilder; // B.F.N reader of json file

   ifstream file(in_path,  ifstream::binary); // B.F.N file to read
   cout <<in_path <<  endl;
   string errs; // B.F.N where the errors are memorized

  if (!file.is_open()) {
    cerr << "Could not open the file: " << in_path <<  endl;
    return Json::Value();
  }
  // B.F.N check if the parse is successfull
  bool parsingSuccessful = Json::parseFromStream(readerBuilder, file, &root, &errs);
  if (!parsingSuccessful) {
      cout << "Failed to parse JSON file: " << errs <<  endl;
      return Json::Value();
  }

  file.close();
  return root;
}

// read json to initialize world items
RobotsAndLidarsVector initSimEnv(Json::Value root, WorldPointer w, int& robot_counter) {

  cout << "World id -> "  << w->_id << endl;
  WorldPointer world_ = w; 
  RobotsVector robots; // changed
  LidarsVector lidars; // changed
  IdRobotSharedMap id_shared_robots; // map id -> shared_ptr<Robot>
  RobotsVector robot_orphans;
  
  // to complete or delete
  // LidarsVector lidar_orphans;
  //LidarsVector lidars_with_parent;
  // IdItemTupleVector orphans;

  if (root["items"].isArray()) {

    for(const Json::Value& item: root["items"]) {

      const int id = item["id"].asInt();
      const  string type = item["type"].asString();
      const string namespace_ = item["namespace"].asString();
      const int id_p = item["parent"].asInt(); 
  
             
      if (type == "robot") {

        // Robot parameters
        double radius = item["radius"].asDouble();
        double pose_x = item["pose"][0].asInt();
        double pose_y = item["pose"][1].asInt();
        double theta = item["pose"][2].asDouble();

        Pose robot_pose = Pose::Identity();
        robot_pose.translation() = world_->grid2world(Eigen::Vector2i(world_->rows/2, world_->cols/2));
        robot_pose.linear() = Eigen::Rotation2Df(theta).matrix();


        Robot* r = new Robot(radius, world_, namespace_, robot_pose, id_p); // create a robot object dynamically
        RobotPointer r_(r, [](Robot* r){ }); // create a shared_ptr with a custom deleter that doesn't delete dynamically allocated object

        id_shared_robots[id] = r_; 

        robots.push_back(r_);
        if (id_p != -1) robot_orphans.push_back(r_);
        robot_counter++;
      }
      else {

        // Lidar parameters
        float fov = item["fov"].asFloat();
        float vfov = item["vfov"].asFloat();
        
        double max_range_l = item["max_range"].asDouble();
        int num_beams_l = item["num_beams"].asInt();
        float pose_x = item["pose"][0].asFloat();
        float pose_y = item["pose"][1].asFloat();
        float theta = item["pose"][2].asFloat();
    
        Pose lidar_pose = Pose::Identity();
        lidar_pose.translation() = world_->grid2world(Eigen::Vector2i(pose_x, pose_y));
        lidar_pose.linear() = Eigen::Rotation2Df(theta).matrix();

        if (id_p != -1){
          RobotPointer parent_= id_shared_robots[id_p]; // the robot where the lidar is placed
          if (parent_ == nullptr) {
            cerr << "The world item you're refering to doesn't exist, please check config.json!"<< endl;
            return RobotsAndLidarsVector({}, {});
          }
          else {
            Pose pose_parent = parent_->poseInWorld();
            // std::cout << "pose in parent:\n" << pose_parent.matrix() << "\n";
            Lidar* l = new Lidar(fov, vfov, max_range_l , num_beams_l, parent_, namespace_, lidar_pose);
            LidarPointer l_(l, [](Lidar* l){ }); // create a shared_ptr with a custom deleter that deletes the dynamically allocated object
            //lidars_with_parent.push_back(l); // to check if usefull
            lidars.push_back(l_);
          }
        }
        else {
          Lidar* l = new Lidar(fov, vfov, max_range_l , num_beams_l, world_, namespace_, lidar_pose);
          LidarPointer l_(l, [](Lidar* l){ }); // create a shared_ptr with a custom deleter that deletes the dynamically allocated object
          // lidar_orphans.push_back(l);
          lidars.push_back(l_);
        }
      }
    }
  }
  return RobotsAndLidarsVector(robots,lidars);
}

// LC: this function return the path to the root directory of the current git project
 string getGitRootPath() {

  array<char, 128> buffer;
  string root_path;

  // A pipe is opened to run the command git rev-parse --show-toplevel
  FILE* pipe = popen("git rev-parse --show-toplevel", "r");

  if (!pipe) {
    throw  runtime_error("popen() failed!");
  }

  // Read the output of the command from the pipe buffer line by line
  // accumulate it in the root_path variable
  while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
    root_path += buffer.data();
  }

  // Close the pipe and check the return code to be on the safe side
  auto return_code = pclose(pipe);

  if (return_code != 0) {
    throw  runtime_error("Error finding git root directory");
  }

  // Remove the newline character at the end of the result
  if (!root_path.empty() && root_path[root_path.size() - 1] == '\n') {
    root_path.erase(root_path.size() - 1);
  }

  return root_path;
}

double timeMillisec() {
  struct timeval tv;
  gettimeofday(&tv,0);
  return tv.tv_sec*1000+tv.tv_usec*1e-3;
}

