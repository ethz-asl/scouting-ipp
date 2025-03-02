//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#ifndef PLANEXP_BASE_PLANNERS_UTILS_H
#define PLANEXP_BASE_PLANNERS_UTILS_H

#include "planexp_base_planners/common/types.h"

#include <vector>
#include <algorithm>
#include <sys/stat.h>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <yaml-cpp/yaml.h>

#if __has_include(<filesystem>)
#include <filesystem>
  namespace fs = std::filesystem;
#elif __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
error "Missing the <filesystem> header."
#endif

template <typename T> std::vector<T> unique(const cv::Mat& input, bool sort = false)
{
  if (input.channels() > 1)
  {
    std::cerr << "unique !!! Only works with 1-channel Mat" << std::endl;
    return std::vector<T>();
  }

  std::vector<T> out;
  for (int y = 0; y < input.rows; ++y)
  {
    const T* row_ptr = input.ptr<T>(y);
    for (int x = 0; x < input.cols; ++x)
    {
      T value = row_ptr[x];

      if ( std::find(out.begin(), out.end(), value) == out.end() ) {
        out.push_back(value);
      }
    }
  }

  if (sort)
    std::sort(out.begin(), out.end());

  return out;
}

inline bool file_exists (const std::string& p) {
  return (fs::exists(p) && fs::is_regular_file(p));
}

bool dir_exists(const std::string &p)
{
  return (fs::exists(p) && fs::is_directory(p));
}

void roundImg(cv::Mat& image) {
  if (image.channels() == 1) {
    image.convertTo(image, CV_32FC1);
    for(int i=0; i<image.rows; i++)
      for(int j=0; j<image.cols; j++)
        image.at<float>(i, j) = std::round(image.at<float>(i, j));
    image.convertTo(image, CV_32SC1);
  }
  else if (image.channels() == 3) {
    image.convertTo(image, CV_32FC3);
    for(int i=0; i<image.rows; i++) {
      for(int j=0; j<image.cols; j++) {
        cv::Point3f point;
        point.x = std::round(image.at<cv::Point3f>(i, j).x);
        point.y = std::round(image.at<cv::Point3f>(i, j).y);
        point.z = std::round(image.at<cv::Point3f>(i, j).z);
        image.at<cv::Point3f>(i, j) = point;
      }
    }
    image.convertTo(image, CV_32SC3);
  }
}

map_t extract_lables(const cv::Mat& input, bool verbose = true, int num_classes = 15) {
  cv::Mat image;
  cv::Mat channels[3];
  double step = num_classes/255.0;
  int c2 = std::pow(num_classes,2);

  input.convertTo(image, CV_32FC3);
  image.convertTo(image, CV_32FC3, step, 0 );
  roundImg(image);
  image.convertTo(image, CV_32SC3);
  cv::split(image, channels);
  cv::Mat labels = channels[0] + channels[1] * num_classes + channels[2] * c2;

  std::vector<int> destinct_labels;
  destinct_labels = unique<int>(labels, true);

  if (verbose) {
    std::cout << "labels: " << std::endl;
    for(auto label:destinct_labels) std::cout << label << std::endl;
  }

  map_t map;
  cv::cv2eigen(labels, map);

  return map;
}

config_t parse_config_file(std::string file_path) {
  config_t planner_config;
  YAML::Node config = YAML::LoadFile(file_path);
  planner_config.map_source = MapSource(config["map"]["source"].as<int>());
  switch (planner_config.map_source) {
    case MS_IMAGE:
      planner_config.map_path = config["map"]["path"].as<std::string>();
      break;
    case MS_OAISYS:
      planner_config.map_topic = config["map"]["topic"].as<std::string>();
      break;
    default:
      break;
  }
  planner_config.map_height = config["map"]["height"].as<double>();
  planner_config.map_width = config["map"]["width"].as<double>();
  planner_config.max_iterations = config["map"]["max_iterations"].as<int>();
  planner_config.intraversable_label_ids = config["map"]["intraversable_label_ids"].as<std::vector<int>>();
  planner_config.min_distance = config["planner"]["min_distance"].as<double>();
  planner_config.sample_dist = config["planner"]["sample_distance"].as<double>();
  planner_config.step_range = config["planner"]["step_range"].as<double>();
  planner_config.optim_time = config["planner"]["optim_time"].as<double>();
  planner_config.goal_bias = config["planner"]["goal_bias"].as<double>();
  planner_config.success_threshold = config["planner"]["success_threshold"].as<double>();
  planner_config.verbose = config["verbose"].as<bool>();
  planner_config.save_output_map = config["save_output_map"].as<bool>();
  planner_config.save_tree_map = config["save_tree_map"].as<bool>();
  planner_config.save_path = config["save_path"].as<bool>();
  return planner_config;
}

bool check_map_config(map_config_t map_config, std::string file_path = "", bool verbose = true) {
  bool config_valid = true;

  if (map_config.resolution.size() == 2 && map_config.goal_location.size() == 2) {
    config_valid &= (bool)map_config.name.size();
    config_valid &= (map_config.width > 0.0) && (map_config.height > 0.0) && (map_config.resolution[0] > 0) && (map_config.resolution[1] > 0);
    config_valid &= (std::abs(1 - ((map_config.width/map_config.resolution[0]) / (map_config.height/map_config.resolution[1]))) < 0.01);
    config_valid &= (map_config.n_data_classes) > 0 && (map_config.n_data_classes < 255);
    int max_id = std::pow(map_config.n_data_classes, 3) + std::pow(map_config.n_data_classes, 2) - 1;
    for (int id:map_config.present_ids) config_valid &= (id >= 0) && (id <= max_id);
    for (int id:map_config.obstacle_ids_mav) config_valid &= (id >= 0) && (id <= max_id);
    for (int id:map_config.obstacle_ids_rov) config_valid &= (id >= 0) && (id <= max_id);
    config_valid &= (map_config.unknown_id >= 0) && (map_config.unknown_id <= max_id);
    config_valid &= (map_config.goal_location[0] >= 0.0) && (map_config.goal_location[1] >= 0.0) && (map_config.goal_location[0] < map_config.width) && (map_config.goal_location[1] < map_config.height);
    config_valid &= (map_config.start_location[0] >= 0.0) && (map_config.start_location[1] >= 0.0) && (map_config.start_location[0] < map_config.width) && (map_config.start_location[1] < map_config.height);
    config_valid &= (map_config.start_location[0] != map_config.goal_location[0]) || (map_config.start_location[1] >= map_config.goal_location[1]);
  } else {
    config_valid = false;
  }

  if (!config_valid && verbose) std::cout << "[PlannerPlanExp] Invalid configuration from map config file: " << file_path << std::endl;
  return config_valid;
}

map_config_t parse_map_config_file(std::string file_path) {
  map_config_t map_config;
  try {
    YAML::Node config = YAML::LoadFile(file_path);
    map_config.name = config["name"].as<std::string>();
    std::string map_type = config["type"].as<std::string>();
    if (map_type == "artificial") map_config.type = MT_ARTIFICIAL;
    else if (map_type == "real") map_config.type = MT_REAL;
    else map_config.type = MT_UNKNOWN;
    map_config.resolution = config["resolution"].as<std::vector<int>>();
    map_config.width = config["width"].as<double>();
    map_config.height = config["height"].as<double>();
    map_config.n_data_classes = config["n_data_classes"].as<int>();
    map_config.present_ids = config["present_ids"].as<std::vector<int>>();
    map_config.obstacle_ids_mav = config["obstacle_ids_mav"].as<std::vector<int>>();
    map_config.obstacle_ids_rov = config["obstacle_ids_rov"].as<std::vector<int>>();
    map_config.unknown_id = config["unknown_id"].as<int>();
    map_config.start_location = config["start_location"].as<std::vector<double>>();
    map_config.goal_location = config["goal_location"].as<std::vector<double>>();
    map_config.explorable_free_space = config["explorable_free_space"].as<double>();
  }
  catch (...) {
    std::cout << "[PlannerPlanExp] Unable to read or parse map config file: " << file_path << std::endl;
    return map_config_t();
  }
  if (!check_map_config(map_config, file_path)) return map_config_t();
  return map_config;
}

map_t load_map(config_t& config, cv::Mat& image, const std::string& map_path="", int n_data_classes = 15, bool verbose = true) {
  map_t map;
  cv::Mat cv_map;
  switch (config.map_source) {
    case MS_OAISYS:
    case MS_RANDOM:
      map = Eigen::MatrixXi::Random(480, 640);
      image = cv::Mat(map.rows(), map.cols(), CV_8UC3, cv::Scalar(0, 0, 0));
      cv_map = cv::Mat(map.rows(), map.cols(), CV_16SC1, map.data());
      cv::applyColorMap(cv_map, image, cv::COLORMAP_COOL);
      return map.cast<int>();
    case MS_IMAGE:
      if (map_path == "") {

#if CV_MAJOR_VERSION >= 4
        image = cv::imread(config.map_path, cv::IMREAD_COLOR);
#else
        image = cv::imread(config.map_path, CV_LOAD_IMAGE_COLOR);
#endif
      } else {
#if CV_MAJOR_VERSION >= 4
        image = cv::imread(map_path, cv::IMREAD_COLOR);
#else
        image = cv::imread(map_path, CV_LOAD_IMAGE_COLOR);
#endif

      }
      map = extract_lables(image, verbose, n_data_classes);
      return map;

    default:
      image = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
      return Eigen::MatrixXi::Zero(480, 640);
  }
}

inline bool id_in_vec(const int id, const std::vector<int> vec) {
  return (std::find(vec.begin(), vec.end(), id) != vec.end());
}

bool get_planner_params(planexp_base_planners::PlannerParams &params, int &n_data_classes, config_t config, map_config_t map_config, Eigen::Vector2f start, Eigen::Vector2f goal) {
  Eigen::Vector2d start_position;
  Eigen::Vector2d goal_position;

  int min_cost = INT32_MAX;
  int max_cost = -1;

  double width, height;
  int px_x, px_y;
  int unknown_id, n_data_classes_;
  std::vector<int> intraversable_ids;

  bool map_config_valid = check_map_config(map_config, "", false);

  if (map_config_valid) {
    width = map_config.width;
    height = map_config.height;
    unknown_id = map_config.unknown_id;
    intraversable_ids = map_config.obstacle_ids_rov;
    start_position = Eigen::Vector2d(map_config.start_location.data());
    goal_position = Eigen::Vector2d(map_config.goal_location.data());
    n_data_classes = map_config.n_data_classes;
    px_x = map_config.resolution[0];
    px_y = map_config.resolution[1];
  } else {
    width = config.map_width;
    height = config.map_height;
    n_data_classes = 15; //TODO: Find better solution to initialize.
    unknown_id = std::pow(n_data_classes_, 3) + std::pow(n_data_classes_, 2) - 1;
    intraversable_ids = config.intraversable_label_ids;
    px_x = 640; //TODO: Find better solution to initialize.
    px_y = 480; //TODO: Find better solution to initialize.
  }

  intraversable_ids.push_back(unknown_id);

  if (start.x() >= 0.0 && start.y() >= 0.0) start_position = start.cast<double>();
  else if (!map_config_valid) return false;
  if (goal.x() >= 0.0 && goal.y() >= 0.0) goal_position = goal.cast<double>();
  else if (!map_config_valid) return false;

  for (int id:map_config.present_ids) {
    if (!id_in_vec(id, intraversable_ids)) {
      if (id <= min_cost) min_cost = id;
      if (id >= max_cost) max_cost = id;
    }
  }

  if (max_cost < min_cost) return false;

  params = planexp_base_planners::PlannerParams{
      .px_x=px_x,
      .px_y=px_y,
      .width=width, .height=height,
      .min_cost=(double)min_cost,
      .max_cost=(double)max_cost,
      .init_cost=(double)min_cost,
      .start=start_position,
      .goal=goal_position,
      .obstacle_ids=intraversable_ids,
      .unknown_id=unknown_id,
      .time_limit=30.0};

  return true;
}

cv::Point coordinate_to_pixle(planexp_base_planners::PlannerParams params, Eigen::Vector2d coordinates)
{
  double spacing_y = params.height / params.px_y;
  double spacing_x = params.width / params.px_x;
  return cv::Point((int)(coordinates[0]/spacing_x), (int)(coordinates[1]/spacing_y));
}

cv::Mat compute_visualisation_image(map_t map, planexp_base_planners::PlannerParams params) {
  cv::Mat map_img(map.rows(),map.cols(), CV_8UC3, CV_BLACK);
  cv::eigen2cv(map, map_img);
  std::vector<int> ids = unique<int>(map_img, true);
  std::vector<int> obst_ids = params.obstacle_ids;
  int unknown_id = params.obstacle_ids.back();

  for (int i = 0; i<ids.size();) {
    if (!id_in_vec(ids[i], obst_ids)) {
      ++i;
    } else {
      ids.erase(ids.begin() + i);
    }
  }
  obst_ids.pop_back();

  cv::Mat vis_image;
  vis_image.create(map.rows(), map.cols(), CV_8UC3);
  double s = 0;
  for(int i = 0; i<map.rows(); ++i) {
    for(int j = 0; j<map.cols(); ++j) {
      int data_point = map(i,j);
      if (ids.back() != ids.front()) s = (double)(data_point - ids.front()) / (ids.back()-ids.front());
      if (id_in_vec(data_point, obst_ids)) vis_image.at<cv::Vec3b>(i,j) = CV_BLACK;
      else if (data_point == unknown_id) vis_image.at<cv::Vec3b>(i,j) = CV_GRAY;
      else {
        if (s > 0.5) {
          s = (s - 0.5) * 2.0;
          vis_image.at<cv::Vec3b>(i,j) = (cv::Vec3b)(s * CV_RED + (1-s) * CV_YELLOW);
        } else {
          s = s * 2;
          vis_image.at<cv::Vec3b>(i,j) = (cv::Vec3b)(s * CV_YELLOW + (1-s) * CV_GREEN);
        }

      }
    }
  }
  return vis_image;
}

void visualize_results(const std::string map_path, cv::Mat orig_image, map_t map, planexp_base_planners::PlannerParams params, std::vector<Eigen::Vector2d> current_path_estimate, bool use_visualization) {
  cv::Mat image;
  if (use_visualization) {
    image = compute_visualisation_image(map, params);
  } else {
    image = orig_image.clone();
  }
  draw_start(image, coordinate_to_pixle(params, current_path_estimate.front()));
  draw_goal(image, coordinate_to_pixle(params, current_path_estimate.back()));

  for( size_t i = 0 ; i < current_path_estimate.size( )-1 ; ++i )
    draw_path_segment(image, coordinate_to_pixle(params, current_path_estimate[i]), coordinate_to_pixle(params, current_path_estimate[i+1]));

  cv::imwrite(map_path, image);
}

void store_results(const std::string& output_file_path, const std::string map_name, double cost, planexp_base_planners::PlannerParams params) {
  bool target_reached = (cost != -1.0);

  std::ofstream output_file;
  output_file.open(output_file_path, std::ios::app);
  output_file << map_name << "," << params.width << "," << params.height << "," << params.start[0] << "," << params.start[1] << "," << params.goal[0] << "," << params.goal[1] << "," << (int)target_reached << "," << cost << std::endl;
}

void store_path(const std::string& output_path, const std::string map_name, double cost, std::vector<Eigen::Vector2d> current_path_estimate, planexp_base_planners::PlannerParams params, bool verbose = true) {
  if (cost != -1.0) {
    std::string output_file_path = fs::path(output_path) / (map_name + "_path.csv");
    std::ofstream output_file;
    output_file.open(output_file_path, std::ios::out | std::ios::trunc);
    output_file << "PositionX" << "," << "PositionY" << "," << "LocationX" << "," << "LocationY" << std::endl;
    output_file << "m" << "," << "m" << "," << "px" << "," << "px" << std::endl;
    for (auto point:current_path_estimate) output_file << point.x() << "," << point.y() << "," << coordinate_to_pixle(params, point).x << "," << coordinate_to_pixle(params, point).y << std::endl;
    output_file.close();
    if (verbose) std::cout << "[PlannerPlanExp] Wrote path to file: " << output_file_path << std::endl;
  }
}

#endif //PLANEXP_BASE_PLANNERS_UTILS_H
