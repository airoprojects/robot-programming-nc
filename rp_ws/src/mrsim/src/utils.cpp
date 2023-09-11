// Custom lib
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
//  tuple< map<int,  shared_ptr<WorldItem>>,  map< string,  vector< shared_ptr<WorldItem>>>> 
RobotLidarMap initSimEnv(Json::Value root, WorldPointer w_ptr, int& robot_counter) {

  cout << "World id -> "  << w_ptr->_id << endl;

  vector<WorldItem> items_;
  WorldItemMap dict_id_r;
  RobotLidarMap dict_namespace_rl;

  if (root["items"].isArray()) {

    for(const Json::Value& item: root["items"]) {

      const int id = item["id"].asInt();
      const  string type = item["type"].asString();

      WorldPointer world_ ; // IL MONDOOOOOOO
      int id_p = item["parent"].asInt(); 
      
      if (id_p == -1) {
        world_ = w_ptr; // upcasting -> reference to World
        if (world_ == nullptr) cerr << "The world item you're refering to doesn't exist, please check config.json !"<< endl;
      } 
      else { 
        // Here we look for the pointer of the parent inside item_dict
        shared_ptr<WorldItem> world_item = dict_id_r[id_p];
        if (world_item == nullptr) cerr << "The world item you're refering to doesn't exist, please check config.json !"<< endl;
      }
           
      if (type == "robot") {
        Pose pose_r;
        double pose_x = item["pose"][0].asInt();
        double pose_y = item["pose"][1].asInt();
        double theta = item["pose"][2].asDouble();

        IntPoint my_point(pose_x, pose_y); // libero su cauzione
        pose_r.translation() = world_->grid2world(my_point);

        shared_ptr<Robot> r =  make_shared<Robot>(item["radius"].asDouble(), world_,item["namespace"].asString(), pose_r);
        dict_id_r[item["id"].asInt()] = r;
        dict_namespace_rl[item["namespace"].asString()].push_back(r);
        robot_counter++;
      }
      else {
        Pose pose_l;
        float pose_x = item["pose"][0].asFloat();
        float pose_y = item["pose"][1].asFloat();
        float theta = item["pose"][2].asFloat();
        pose_l.translation() = Eigen::Vector2f(pose_x, pose_y);

        // This is to create a ptr_shared  the only way!
        shared_ptr<Lidar> l =  make_shared<Lidar>(item["fov"].asInt(), item["max_range"].asDouble(), item["num_beams"].asInt(), world_,item["namespace"].asString(), pose_l);
        dict_namespace_rl[item["namespace"].asString()].push_back(l);
      }
    }
  }

  // DictTuple result =  make_tuple(dict_id_r, dict_namespace_rl);
  return dict_namespace_rl;
}

// LC: thsi function thake as input a config.jason file and makes a ros launch file to start nodes in a simulation
int makeLaunchFile( string in_path,  string out_path) {

  Json::Value root = readJson(in_path);

  // start the parse of the file
  const  string map = root["map"].asString();
  cout << "Map: " << map <<  endl;

  // Robot counter initialization
  int robot_counter = 0;

  // Open the launch file
  ofstream outfile(out_path);
  // outfile.open(out_path,  ofstream::out |  ofstream::trunc);
  cout << "Launch file path:" << out_path << endl;

  int stop;
  cin >> stop;

  // Check if the file was opened successfully
  if (!outfile) {
       cout << "Error path:" << out_path << endl;
       cerr << "Error opening file for writing" <<  endl;
      return 1;
  }
  outfile << "<launch> \n\n";

  if (root["items"].isArray()) {
    for(const Json::Value& item: root["items"]) {

      const int id = item["id"].asInt();
      const  string type = item["type"].asString();

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
      else  cerr << "Type error reading JSON file" <<  endl;
      outfile << "\n";
    }
  }
  outfile << "\n</launch>";
  outfile.close();
  cout << "Launch file created successfully!!!" <<  endl;
  return robot_counter;
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



// // temporary metodh to be  delated or implemented
// void onMouse(int event, int x, int y, int flags, void* userdata) {
//     if (event == cv::EVENT_LBUTTONDOWN) {
//         // Define the button's region (here, a rectangle at the top-left corner of the window)
//         if (x > 50 && x < 200 && y > 50 && y < 100) {
//              cout << "Button clicked!" <<  endl;
//         }
//         cv::destroyWindow("ButtonsWindow");
//     }
// }