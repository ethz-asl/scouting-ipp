//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#include "planexp_base_planners/planexp_base_planners.h"
#include "planexp_base_planners/common/arg_parser.h"
#include "planexp_base_planners/common/drawing_helper.h"
#include "planexp_base_planners/common/utils.h"
#include <fstream>


void getFloatArgument(const InputParser& input, float& argument, std::string identifier, bool verbose = true) {
  const std::string &arguments_string_const = input.getCmdOption(identifier);
  if (!arguments_string_const.empty()){
    try
    {
      argument = std::stof(arguments_string_const);
    }
    catch (std::invalid_argument& e)
    {
      if (verbose) {
        std::cout << "[PlannerPlanExpEval] Unable to parse argument " << argument << ": "<< arguments_string_const << std::endl;
        std::cout << "[PlannerPlanExpEval] Using default instead!" << std::endl;
      }
    }
  }
}

bool getConfig(const InputParser& input, config_t& planner_config, bool verbose = true) {

  const std::string &config_path = input.getCmdOption("-c");
  if (!config_path.empty() && file_exists(config_path)){
    planner_config = parse_config_file(config_path);
  } else {
    const std::string &config_path_replace = fs::path(getenv("HOME"))/"ros_ws"/"src"/"scouting-ipp"/"planexp_base_planners"/"cfg"/"rrt_star_config.yaml";
    if (file_exists(config_path_replace)) {
      if (verbose) std::cout << "[PlannerPlanExpEval] Config file not defined. Defaulting to: " << config_path_replace << std::endl;
      planner_config = parse_config_file(config_path_replace);
    } else {
      if (verbose) std::cout << "[PlannerPlanExpEval] No config file found.";
      return false;
    }
  }
  return true;
}

void getImageOutputDir(const InputParser& input, const std::string& output_dir, std::string& image_output_dir, bool verbose = true) {
  fs::path output_path = output_dir;
  const std::string &image_output_dir_name_const = input.getCmdOption("-i");
  if (!image_output_dir_name_const.empty() && dir_exists(output_path / image_output_dir_name_const)){
    image_output_dir = output_path / image_output_dir_name_const;
  }
  else if (!image_output_dir_name_const.empty() && !dir_exists(output_path / image_output_dir_name_const)) {
    image_output_dir = output_path / image_output_dir_name_const;
    fs::create_directories(image_output_dir);
  }
  else {
    image_output_dir = output_path / "rrt_maps";
    if (!dir_exists(image_output_dir)) fs::create_directories(image_output_dir);
  }
}

void getPathOutputDir(const InputParser& input, const std::string& output_dir, std::string& path_output_dir, bool verbose = true) {
  fs::path output_path = output_dir;
  const std::string &path_output_dir_name_const = input.getCmdOption("-p");
  if (!path_output_dir_name_const.empty() && dir_exists(output_path / path_output_dir_name_const)){
    path_output_dir = output_path / path_output_dir_name_const;
  }
  else if (!path_output_dir_name_const.empty() && !dir_exists(output_path / path_output_dir_name_const)) {
    path_output_dir = output_path / path_output_dir_name_const;
    fs::create_directories(path_output_dir);
  }
  else {
    path_output_dir = output_path / "rrt_paths";
    if (!dir_exists(path_output_dir)) fs::create_directories(path_output_dir);
  }
}

bool getOutputDir(const InputParser& input, std::string& output_dir, bool verbose = true) {
  const std::string &output_dir_const = input.getCmdOption("-o");
  if (!output_dir_const.empty() && dir_exists(output_dir_const)){
    output_dir = output_dir_const;
  } else {
    const std::string &output_dir_replace = fs::temp_directory_path();
    if (dir_exists(output_dir_replace)) {
      if (verbose) std::cout << "[PlannerPlanExpEval] Output directory not found. Defaulting to: " << output_dir_replace << std::endl;
      output_dir = output_dir_replace;
    } else {
      if (verbose) std::cout << "[PlannerPlanExpEval] No output directory found.";
      return false;
    }
  }
  return true;
}

void getOutputFile(const InputParser& input, const std::string& output_dir, std::string& output_file, bool verbose = true) {
  fs::path output_path = output_dir;
  const std::string header = "MapName,Width,Height,StartX,StartY,GoalX,GoalY,Success,TravelCost";
  const std::string header2 = "-,m,m,m,m,m,m,-,1";
  const std::string &output_file_name_const = input.getCmdOption("-f");
  std::ofstream myfile;
  if (!output_file_name_const.empty() && file_exists(output_path / output_file_name_const)){
    output_file = output_path / output_file_name_const;
  }
  else if (!output_file_name_const.empty() && !file_exists(output_path / output_file_name_const)) {
    output_file = output_path / output_file_name_const;
    myfile.open (output_file);
    myfile << header << std::endl;
    myfile << header2 << std::endl;
    myfile.close();
  }
  else {
    output_file = output_path / "rrt_results.csv";
    if (!file_exists(output_file)) {
      myfile.open(output_file);
      myfile << header << std::endl;
      myfile.close();
    }
  }
  if (verbose) std::cout << "[PlannerPlanExpEval] Output file: " << output_file << std::endl;
}

bool getMapPath(const InputParser& input, std::string& map_path, bool verbose = true) {
  const std::string &map_path_const = input.getCmdOption("-m");
  if (!map_path_const.empty() && file_exists(map_path_const)){
    map_path = map_path_const;
  } else {
    if (verbose) std::cout << "[PlannerPlanExpEval] No map found at location: " << map_path_const << std::endl;
    return false;
  }
  return true;
}

bool getMapConfigPath(const InputParser& input, std::string& map_config_path, bool verbose = true) {
  const std::string &map_config_path_const = input.getCmdOption("-cm");
  if (!map_config_path_const.empty() && file_exists(map_config_path_const)){
    map_config_path = map_config_path_const;
  } else {
    if (verbose) std::cout << "[PlannerPlanExpEval] No map config found at location: " << map_config_path_const << std::endl;
    return false;
  }
  return true;
}

bool getStartAndGoal(const InputParser& input, Eigen::Vector2f& start, Eigen::Vector2f& goal, bool verbose = true) {
  std::vector<std::string> identifiers = {"-sx", "-sy", "-gx", "-gy"};
  std::vector<float> values;
  for (std::string identifier : identifiers) {
    float value = -1.0;
    getFloatArgument(input, value, identifier, verbose);
    values.push_back(value);
  }
  start.x() = values[0];
  start.y() = values[1];
  goal.x() = values[2];
  goal.y() = values[3];

  return true;
}

void getPlannerType(const InputParser& input, planexp_base_planners::PlannerType& planner_type, bool verbose = true) {
  const std::string &planner_type_str_const = input.getCmdOption("-t");
  std::string planner_type_str = "PLANEXP_PRM_STAR_PLANNER";
  planner_type = planexp_base_planners::PlannerType::PRM_STAR_PLANNER;

  if (!planner_type_str_const.empty()){
    if (planner_type_str_const == "prm_star") {
      planner_type_str = "PLANEXP_PRM_STAR_PLANNER";
      planner_type = planexp_base_planners::PlannerType::PRM_STAR_PLANNER;
    } else if (planner_type_str_const == "rrt_star") {
      planner_type_str = "PLANEXP_RRT_STAR_PLANNER";
      planner_type = planexp_base_planners::PlannerType::RRT_STAR_PLANNER;
    }
  }

  if (verbose) std::cout << "[PlannerPlanExpEval] Planner type: " << planner_type_str << std::endl;
}

bool printHelp(const InputParser& input) {
  if (input.cmdOptionExists("-h")) {
    std::cout << "usage: planner_rockenbauer_eval -m M [-t T] [-c C] [-cm CM] [-f F] [-i I] [-p P] [-o O] [-sx SX] [-sy SY] [-gx GX] [-gy GY]" << std::endl;
    std::cout << std::endl << "required:" << std::endl;
    std::cout << "M " << "  ... " << "Absolute path to the map file. (*.png)" << std::endl;
    std::cout << std::endl << "optional:" << std::endl;
    std::cout << "T " << " ... " << "Planner type: rrt_star, prm_star (Default: prm_star)" << std::endl;
    std::cout << "C " << " ... " << "Absolute path to the config file. (*.yaml)" << std::endl;
    std::cout << "CM" << " ... " << "Absolute path to the map config file. (*.png)" << std::endl;
    std::cout << "F " << " ... " << "Name of the output file including extension. (Default: rrt_results.csv)" << std::endl;
    std::cout << "I " << " ... " << "Name of the directory to store map images. (Default: rrt_maps)" << std::endl;
    std::cout << "P " << " ... " << "Name of the directory to store paths. (Default: rrt_paths)" << std::endl;
    std::cout << "O " << " ... " << "Absolut path to the directory were all outputs are to be stored." << std::endl;
    std::cout << "SX" << " ... " << "X component of the start location." << std::endl;
    std::cout << "SY" << " ... " << "Y component of the start location." << std::endl;
    std::cout << "GX" << " ... " << "X component of the goal location." << std::endl;
    std::cout << "GY" << " ... " << "Y component of the goal location." << std::endl;
    return true;
  }
  return false;
}

int main(int argc, char **argv) {
  InputParser input(argc, argv);

  bool need_help = printHelp(input);
  if (need_help) return 0;

  config_t planner_config;
  bool success = getConfig(input, planner_config, planner_config.verbose); //TODO: Verbose is not set yet so default value defines output

  if (planner_config.verbose) std::cout << "[PlannerPlanExpEval] Loading map config!" << std::endl;
  std::string map_config_path = "";
  map_config_t map_config = map_config_t();
  bool found_map_config = getMapConfigPath(input, map_config_path, planner_config.verbose);
  if (found_map_config) map_config = parse_map_config_file(map_config_path);

  if (planner_config.verbose) std::cout << "[PlannerPlanExpEval] Getting output directory!" << std::endl;
  std::string output_dir;
  success &= getOutputDir(input, output_dir, planner_config.verbose);

  if (planner_config.verbose) std::cout << "[PlannerPlanExpEval] Getting output directory!" << std::endl;
  std::string output_file;
  getOutputFile(input, output_dir, output_file, planner_config.verbose);

  if (planner_config.verbose) std::cout << "[PlannerPlanExpEval] Getting map!" << std::endl;
  std::string map_path = "";
  if (planner_config.map_source == MS_IMAGE) {
    success &= getMapPath(input, map_path, planner_config.verbose);
  } else {
    getMapPath(input, map_path, planner_config.verbose);
  }
  if (!success) return 0;

  if (planner_config.verbose) std::cout << "[PlannerPlanExpEval] Getting visualization output directory!" << std::endl;
  std::string image_output_dir;
  if (planner_config.save_output_map) getImageOutputDir(input, output_dir, image_output_dir, planner_config.verbose);

  if (planner_config.verbose) std::cout << "[PlannerPlanExpEval] Getting path output directory!" << std::endl;
  std::string path_output_dir;
  if (planner_config.save_path) getPathOutputDir(input, output_dir, path_output_dir, planner_config.verbose);

  if (planner_config.verbose) std::cout << "[PlannerPlanExpEval] Getting planner type!" << std::endl;
  planexp_base_planners::PlannerType planner_type;
  getPlannerType(input, planner_type, planner_config.verbose);

  if (planner_config.verbose) std::cout << "[PlannerPlanExpEval] Getting start and goal location!" << std::endl;
  Eigen::Vector2f start, goal;
  success &= getStartAndGoal(input, start, goal, planner_config.verbose);
  if (!success) return 0;

  if (planner_config.verbose) std::cout << "[PlannerPlanExpEval] Loading planner config!" << std::endl;
  int n_data_classes;
  planexp_base_planners::PlannerParams planner_params;
  success &= get_planner_params(planner_params, n_data_classes, planner_config, map_config, start, goal);
  if (!success) return 0;

  if (planner_config.verbose) std::cout << "[PlannerPlanExpEval] Loading map!" << std::endl;
  cv::Mat image;
  map_t map = load_map(planner_config, image, map_path, n_data_classes, planner_config.verbose);

  if (planner_config.verbose) std::cout << "[PlannerPlanExpEval] Creating planner!" << std::endl;
  std::unique_ptr<planexp_base_planners::PlannerBase> planner_planexp;
  switch (planner_type) {
    case planexp_base_planners::RRT_STAR_PLANNER:
      planner_planexp = std::unique_ptr<planexp_base_planners::PlannerBase>(new planexp_base_planners::RRTStarPlanner(planner_params));
      break;
    case planexp_base_planners::PRM_STAR_PLANNER:
      planner_planexp = std::unique_ptr<planexp_base_planners::PlannerBase>(new planexp_base_planners::PRMStarPlanner(planner_params));
      planner_planexp->setup();
      break;
    case planexp_base_planners::UNKNOWN_PLANNER:
    default:
      planner_planexp = std::unique_ptr<planexp_base_planners::PlannerBase>(new planexp_base_planners::RRTStarPlanner(planner_params));
  }

  if (planner_config.verbose) std::cout << "[PlannerPlanExpEval] Inserting map into planner!" << std::endl;
  planner_planexp->update_states(Eigen::Vector2i(0,0), map);

  if (planner_config.verbose) std::cout << "[PlannerPlanExpEval] Initialized!" << std::endl;
  if (planner_config.verbose)
    std::cout << "[PlannerPlanExpEval] map_source: " << (int) planner_config.map_source << std::endl;

  if (planner_config.verbose) std::cout << "[PlannerPlanExpEval] Planning: " << std::endl;
  double path_cost = -1.0;
  std::vector<Eigen::Vector2d> current_path_estimate;
  planexp_base_planners::ResultStatus result = planner_planexp->plan(&current_path_estimate, path_cost);


  if (result == planexp_base_planners::ResultStatus::VALID || result == planexp_base_planners::ResultStatus::TERMINAL_VALID) {
    if (planner_config.verbose) std::cout << "[PlannerPlanExpEval] Found valid solution! " << std::endl;
    if (planner_config.save_output_map)
      visualize_results(image_output_dir / fs::path(map_path).filename(), image, map, planner_params, current_path_estimate, true);
//    if (planner_config.save_tree_map)
//      planner_planexp->visualize_tree(fs::path(image_output_dir / (fs::path(map_path).stem().operator+=("_tree.png"))));
  } else {
    if (planner_config.verbose) std::cout << "[PlannerPlanExpEval] Did not find valid solution. " << std::endl;
  }

  if (planner_config.verbose) std::cout << "[PlannerPlanExpEval] Storing result!" << std::endl;
  store_results(output_file, fs::path(map_path).filename(), path_cost, planner_params);
  if (planner_config.verbose) std::cout << "[PlannerPlanExpEval] Storing Path!" << std::endl;
  store_path(path_output_dir, fs::path(map_path).stem(), path_cost, current_path_estimate, planner_params, planner_config.verbose);
  if (planner_config.verbose) std::cout << "[PlannerPlanExpEval] Finished!" << std::endl;
  return 0;
}