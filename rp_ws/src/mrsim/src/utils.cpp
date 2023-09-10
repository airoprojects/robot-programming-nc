// Custom lib
#include "utils.h"


Json::Value readJson(std::string in_path) {
  std::cout << "Configuration file path:" << in_path << std::endl;
  Json::Value root; // B.F.N object to hold the parsed json file
  Json::CharReaderBuilder readerBuilder; // B.F.N reader of json file
  std::ifstream file(in_path, std::ifstream::binary); // B.F.N file to read
  std::string errs; // B.F.N where the errors are memorized

  // B.F.N check if the parse is successfull
  bool parsingSuccessful = Json::parseFromStream(readerBuilder, file, &root, &errs);
  if (!parsingSuccessful) {
      std::cout << "Failed to parse JSON file: " << errs << std::endl;
      return nullptr;
  }

  return root;
}

// read json to initialize world items
std::tuple<std::map<int, std::shared_ptr<WorldItem>>, std::map<std::string, std::vector<std::shared_ptr<WorldItem>>>> 
initSimEnv(std::string in_path, std::shared_ptr<World> w) {

  Json::Value root = readJson(in_path);
  std::vector<WorldItem> items_;
  std::map<int, std::shared_ptr<WorldItem>> dict_id_r;
  std::map<std::string, std::vector<std::shared_ptr<WorldItem>>> dict_namespace_rl;
  

  if (root["items"].isArray()) {

    for(const Json::Value& item: root["items"]) {

      const int id = item["id"].asInt();
      const std::string type = item["type"].asString();

      std::shared_ptr<World> world_;
      int id_p = item["parent"].asInt(); 
      
      if (id_p == -1) {
        world_ = w; // upcasting -> reference to World
      } else { 
        // we look for the pointer of the parent inside item_dict
        std::shared_ptr<WorldItem> world_item = dict_id_r[id_p];
        if (world_item == nullptr) {
          std::cerr << "doesn't exist the world_item you're refering, please check the json!"<<std::endl;
          }
      }
           
      if (type == "robot") {
        Pose pose_r;
        double pose_x = item["pose"][0].isDouble();
        double pose_y = item["pose"][1].isDouble();
        double theta = item["pose"][2].asDouble();

        IntPoint middle(world_->rows/2, world_->cols/2); 
        pose_r.translation() = world_->grid2world(middle);

        std::shared_ptr<Robot> r = std::make_shared<Robot>(item["radius"].asDouble(), world_, pose_r);
        dict_id_r[item["id"].asInt()] = r;
        dict_namespace_rl[item["namespace"].asString()].push_back(r);
      }
      else {
        float pose_x = item["pose"][0].asFloat();
        float pose_y = item["pose"][1].asFloat();
        float theta = item["pose"][2].asFloat();
        Pose pose_l;
        pose_l.translation() = Eigen::Vector2f(pose_x, pose_y);

      // to create a ptr_shared is the only way!
        std::shared_ptr<Lidar> l = std::make_shared<Lidar>(item["fov"].asInt(), item["max_range"].asDouble(), item["num_beams"].asInt(), world_, pose_l);
        dict_namespace_rl[item["namespace"].asString()].push_back(l);
      }
    }
  }

  std::tuple<std::map<int, std::shared_ptr<WorldItem>>, std::map<std::string, std::vector<std::shared_ptr<WorldItem>>>> result = std::make_tuple(dict_id_r, dict_namespace_rl);
  return result;
}

// LC: thsi function thake as input a config.jason file and makes a ros launch file to start nodes in a simulation
int makeLaunchFile(std::string in_path, std::string out_path) {

  Json::Value root = readJson(in_path);

  // start the parse of the file
  const std::string map = root["map"].asString();
  std::cout << "Map: " << map << std::endl;

  // Robot counter initialization
  int robot_counter = 0;

  // Open the launch file
  std::ofstream outfile(out_path);
  // outfile.open(out_path, std::ofstream::out | std::ofstream::trunc);
  std::cout << "Launch file path:" << out_path <<std::endl;

  int stop;
  std::cin >> stop;

  // Check if the file was opened successfully
  if (!outfile) {
      std::cout << "Error path:" << out_path <<std::endl;
      std::cerr << "Error opening file for writing" << std::endl;
      return 1;
  }
  outfile << "<launch> \n\n";

  if (root["items"].isArray()) {
    for(const Json::Value& item: root["items"]) {

      const int id = item["id"].asInt();
      const std::string type = item["type"].asString();

      if(type == "robot"){
       
        outfile << "<node pkg='mrsim'" << " name='" << item["namespace"].asString()
                << "' type='" << type << "_node'" 
                << ">\n"
                << "  <param id='" << id << "' frame_id='" << item["frame_id"].asString()
                << "' namespace='" << item["namespace"].asString()
                << "' radious='" << item["radius"].asDouble() 
                << "' max_rv='" << item["max_rv"].asDouble() 
                << "' max_tv='" << item["max_tv"].asDouble() 
                << "' parent='" << item["parent"].asInt() << "'"
                
                << "/>\n</node> \n";

        robot_counter++;
      }
      else if (type == "lidar") {
        
        outfile << "<node pkg='mrsim'"<< " name='" << item["namespace"].asString() 
                << "' type='" << type << "_node'"
                << ">\n"
                << "  <param id='" << id << "' frame_id='" << item["frame_id"].asString()
                << "' namespace='" << item["namespace"].asString()
                << "' fov='" << item["fov"].asDouble()
                << "' num_beams='" << item["num_beams"].asInt()
                << "' parent='" << item["parent"].asInt() << "'"

                << "/>\n</node> \n";
      }
      else std::cerr << "Type error reading JSON file" << std::endl;
      outfile << "\n";
    }
  }
  outfile << "\n</launch>";
  outfile.close();
  std::cout << "Launch file created successfully!!!" << std::endl;
  return robot_counter;
}
       

// LC: this function return the path to the root directory of the current git project
std::string getGitRootPath() {

  std::array<char, 128> buffer;
  std::string root_path;

  // A pipe is opened to run the command git rev-parse --show-toplevel
  FILE* pipe = popen("git rev-parse --show-toplevel", "r");

  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }

  // Read the output of the command from the pipe buffer line by line
  // accumulate it in the root_path variable
  while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
    root_path += buffer.data();
  }

  // Close the pipe and check the return code to be on the safe side
  auto return_code = pclose(pipe);

  if (return_code != 0) {
    throw std::runtime_error("Error finding git root directory");
  }

  // Remove the newline character at the end of the result
  if (!root_path.empty() && root_path[root_path.size() - 1] == '\n') {
    root_path.erase(root_path.size() - 1);
  }

  return root_path;
}

// // temporary metodh to be  delated or implemented
// void onMouse(int event, int x, int y, int flags, void* userdata) {
//     if (event == cv::EVENT_LBUTTONDOWN) {
//         // Define the button's region (here, a rectangle at the top-left corner of the window)
//         if (x > 50 && x < 200 && y > 50 && y < 100) {
//             std::cout << "Button clicked!" << std::endl;
//         }
//         cv::destroyWindow("ButtonsWindow");
//     }
// }