//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#include "planexp_base_planners/planexp_base_planners.h"
#include "planexp_base_planners/common/arg_parser.h"
#include "planexp_base_planners/common/drawing_helper.h"
#include "planexp_base_planners/common/utils.h"

#include <ros/package.h>
#include <ros/callback_queue.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "planexp_base_planner_node");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  bool success = true;

  std::string rootdir = ros::package::getPath("planexp_base_planners"); // Leaks memory
  std::string config_path = rootdir + "/cfg/rrt_star_config.yaml";
  std::string map_config_path = "";

  nh_private.param("planner_config", config_path, config_path);
  nh_private.param("map_config_path", map_config_path, map_config_path);

  std::cout << "[PlannerPlanExpNode] planner_config_path: " << config_path << std::endl;
  std::cout << "[PlannerPlanExpNode] map_config_path: " << map_config_path << std::endl;

  config_t planner_config = parse_config_file(config_path);
  map_config_t map_config = parse_map_config_file(map_config_path);

  std::string planner_type_str_select;
  nh_private.param("planner_type", planner_type_str_select, std::string(""));
  std::string planner_type_str = "PRM_STAR_PLANNER";
  planexp_base_planners::PlannerType planner_type = planexp_base_planners::PRM_STAR_PLANNER;

  if (!planner_type_str_select.empty()){
    if (planner_type_str_select == "prm_star") {
      planner_type_str = "PRM_STAR_PLANNER";
      planner_type = planexp_base_planners::PRM_STAR_PLANNER;
    } else if (planner_type_str_select == "rrt_star") {
      planner_type_str = "RRT_STAR_PLANNER";
      planner_type = planexp_base_planners::RRT_STAR_PLANNER;
    }
  }
  if (planner_config.verbose) std::cout << "[PlannerPlanExpNode] Planner type: " << planner_type_str << std::endl;

  int n_data_classes;
  planexp_base_planners::PlannerParams planner_params;
  success &= get_planner_params(planner_params, n_data_classes, planner_config, map_config, Eigen::Vector2f(-1.0, -1.0), Eigen::Vector2f(-1.0, -1.0));
  if (!success) return 0;

  //std::string map_path = create_map();

  cv::Mat image;
  map_t map = load_map(planner_config, image, "", n_data_classes, planner_config.verbose);

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

  planner_planexp->update_states(Eigen::Vector2i(0,0), map);

  if (planner_config.verbose) std::cout << "[PlannerPlanExpNode] Initialized!" << std::endl;
  if (planner_config.verbose)
    std::cout << "[PlannerPlanExpNode] map_source: " << (int) planner_config.map_source << std::endl;

  double path_cost = -1.0;
  std::vector<Eigen::Vector2d> current_path_estimate;
  planexp_base_planners::ResultStatus result = planner_planexp->plan(&current_path_estimate, path_cost);

  std::cout << "[PlannerPlanExpNode] Path cost: " << path_cost << std::endl << std::endl;
  if (result == planexp_base_planners::ResultStatus::VALID || result == planexp_base_planners::ResultStatus::TERMINAL_VALID) {
    std::cout << "[PlannerPlanExpNode] Path: " << std::endl;
    for (auto node:current_path_estimate) std::cout << node.x() << "," << node.y() << std::endl;

    if (planner_config.save_output_map)
      visualize_results((std::string)(fs::temp_directory_path()/fs::path(planner_config.map_path).filename()), image, map, planner_params, current_path_estimate, true);

  }

  return 0;
}

std::string create_map(unsigned int width = 640, unsigned int height = 480) {
  cv::Size size(width,height);
  cv::Mat map(size, CV_8UC3);

  cv::resize(map, map,size);

  cv::Rect rect1(0, 0, width/3, height);
  cv::Rect rect2(width/3, 0, width/3, height);
  cv::Rect rect3((width*2)/3, 0, width - (width*2)/3, height);
  cv::Rect obstacle((int)(width*0.15), (int)(height*0.4), (int)(width*0.7), (int)(height*0.2));

#if CV_MAJOR_VERSION >= 4
  cv::rectangle(map, rect1, cv::Scalar(0, 0, 255), cv::FILLED);
  cv::rectangle(map, rect2, cv::Scalar(0, 255, 0), cv::FILLED);
  cv::rectangle(map, rect3, cv::Scalar(255, 0, 0), cv::FILLED);
  cv::rectangle(map, obstacle, cv::Scalar(0, 0, 0), cv::FILLED);
#else
  cv::rectangle(map, rect1, cv::Scalar(0, 0, 255), CV_FILLED);
  cv::rectangle(map, rect2, cv::Scalar(0, 255, 0), CV_FILLED);
  cv::rectangle(map, rect3, cv::Scalar(255, 0, 0), CV_FILLED);
  cv::rectangle(map, obstacle, cv::Scalar(0, 0, 0), CV_FILLED);
#endif
  cv::imwrite((std::string)(fs::temp_directory_path()/"input_new.png"), map);

  return (std::string)(fs::temp_directory_path()/"input_new.png");
}