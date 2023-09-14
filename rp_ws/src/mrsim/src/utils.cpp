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
vector<Robot*>  initSimEnv(Json::Value root, WorldPointer w, int& robot_counter) {

  cout << "World id -> "  << w->_id << endl;
  WorldPointer world_ = w; 
  IdRobotMap id_r_map;
  RobotsVector robots;

  RobotsVector robot_orphans;
  LidarsVector lidar_orphans;

  // IdItemTupleVector orphans;

  if (root["items"].isArray()) {

    for(const Json::Value& item: root["items"]) {

      const int id = item["id"].asInt();
      const  string type = item["type"].asString();
      const string namespace_ = item["namespace"].asString();
      const int id_p = item["parent"].asInt(); 
    

      // if (world_ == nullptr) {
      //   cerr << "The world item you're refering to doesn't exist, please check config.json!"<< endl;
      //   return nullptr;
      // }
      
      // if (id_p == -1 && robot_ == nullptr) {
      //   cerr << "The world item you're refering to doesn't exist, please check config.json!"<< endl;
      //   return 1;
      // }
             
      if (type == "robot") {

        // Robot parameters
        double radius = item["radius"].asDouble();
        double pose_x = item["pose"][0].asInt();
        double pose_y = item["pose"][1].asInt();
        double theta = item["pose"][2].asDouble();

        Pose robot_pose = Pose::Identity();
        robot_pose.translation() = world_->grid2world(Eigen::Vector2i(world_->rows/2, world_->cols/2));
        robot_pose.linear() = Eigen::Rotation2Df(theta).matrix();

        Robot* r = new Robot(radius, world_, namespace_, robot_pose, id_p);
        robots.push_back(r);
        id_r_map[id] = r;
        if (id_p != -1) robot_orphans.push_back(r);
        robot_counter++;
      }
      else {

        // Lidar parameters
        float fov = item["fov"].asFloat();
        double max_range_l = item["max_range"].asDouble();
        int num_beams_l = item["num_beams"].asInt();
        float pose_x = item["pose"][0].asFloat();
        float pose_y = item["pose"][1].asFloat();
        float theta = item["pose"][2].asFloat();
    
        Pose lidar_pose = Pose::Identity();
        lidar_pose.translation() = world_->grid2world(Eigen::Vector2i(pose_x, pose_y));
        lidar_pose.linear() = Eigen::Rotation2Df(theta).matrix();

        //Lidar* l = new Lidar(fov, max_range_l , num_beams_l, world_, namespace_, lidar_pose);
        // if (id_p != -1) orphans.push_back(make_tuple(id_p, l));
      }
    }
  }

  for (const auto item: robot_orphans) {
    auto robot_ = id_r_map[item->id_p];
    if (robot_ == nullptr) {
      cerr << "The world item you're refering to doesn't exist, please check config.json!"<< endl;
      return RobotsVector{};
    }
    else {
      // change the pose in parent of the robot
    }
          
  }

  return robots;
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

