//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//
#include "map_server_planexp/map_server_planexp.h"

MapServerPlanExp::MapServerPlanExp(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) :
                                      nh_(nh), nh_private_(nh_private) {
  // Set up subscriber.
  map_data_sub_ = nh_private_.subscribe("/map_data", 1000,
                                        &MapServerPlanExp::mapDataCallback, this);
  depth_data_sub_ = nh_private_.subscribe("/map_data_depth", 1000,
                                        &MapServerPlanExp::depthDataCallback, this);

  clear_map_srv_ = nh_private_.advertiseService(
      "clear_map", &MapServerPlanExp::clearMapCallback, this);
  save_map_srv_ = nh_private_.advertiseService(
      "save_map", &MapServerPlanExp::saveMapCallback, this);
  load_map_srv_ = nh_private_.advertiseService(
      "load_map", &MapServerPlanExp::loadMapCallback, this);
  publish_map_srv_ = nh_private_.advertiseService(
      "publish_map", &MapServerPlanExp::publishMapCallback, this);
  goal_explored_srv_ = nh_private_.advertiseService(
      "goal_explored", &MapServerPlanExp::goalExploredCallback, this);

  planner_status_pub_ = nh_private_.advertise<planexp_msgs::PlannerStatus>("planner_status", 1000);

  image_transport::ImageTransport it(nh_);
  occupancy_map_vis_pub_ = it.advertise("/occupancy_map_vis",100);
  cost_map_vis_pub_ = it.advertise("/cost_map_vis",100);
  uncertainty_map_vis_pub_ = it.advertise("/uncertainty_map_vis",100);

#if RVIZ_VISUALIZATION
  cost_map_rviz_pub_ = nh_private_.advertise<grid_map_msgs::GridMap>("/cost_map_rviz", 1000);
#if CREATE_RRT_PLANNER_MAP
  path_estimate_vis_pub_ = nh_private_.advertise<nav_msgs::Path>("/path_estimate_vis", 1000);
#endif
#endif

  nh_private_.param("use_uncertainty_map", p_use_uncertainty_map_, false);
  nh_private_.param("use_height_map", p_use_height_map_, false);
  nh_private_.param("compute_reachability", p_compute_reachability_, true);
  nh_private_.param("compute_closest_observed", p_compute_closest_observed_, false);
  nh_private_.param("compute_path", p_compute_path_, false);
  std::string p_unknown_init_cost_str;
  nh_private_.param("unknown_init_cost", p_unknown_init_cost_str, std::string("min"));

  store_paths_ = false;
  nh_private_.param("paths_path", paths_path_, std::string(""));
  if (paths_path_ != "") store_paths_ = true;

  bool ignore_mav_obstacles;
  nh_private_.param("ignore_mav_obstacles", ignore_mav_obstacles, false);

  std::string planner_type_str;
  nh_private_.param("planner_type", planner_type_str, std::string("prm_star"));

  double planner_time;
  nh_private_.param("planner_time", planner_time, 10.0);

  map_state_ = std::make_shared<OccupancyMap_t>();
  map_cost_ = std::make_shared<CostMap_t>();
  map_uncertainty_ = std::make_shared<UncertaintyMap_t>();
  map_height_ = std::make_shared<HeightMap_t>();
  map_height_uncertainty_ = std::make_shared<UncertaintyMap_t>();
  map_cost_closest_ = std::make_shared<CostMap_t>();
  map_distance_closest_ = std::make_shared<DistanceMap_t>();
  map_reachability_ = std::make_shared<ReachabilityMap_t>();

  bool default_init = true;
  start_location_ = Eigen::Vector2i(0, 0);
  start_position_ = Eigen::Vector3d(0.0, 0.0, 0.0);

#if MS_USE_MAP_CONFIG
  std::string map_config_path = "";
  try {
    nh_private_.getParam("map_config_path", map_config_path);
    if (!map_config_path.empty()) {
        YAML::Node config = YAML::LoadFile(map_config_path);
        std::string map_name = config["name"].as<std::string>();
        std::string map_type = config["type"].as<std::string>();
        std::vector<int> map_resolution = config["resolution"].as<std::vector<int>>();
        double map_width = config["width"].as<double>();
        double map_height = config["height"].as<double>();
        int n_data_classes = config["n_data_classes"].as<int>();
        std::vector<int> map_ids = config["present_ids"].as<std::vector<int>>();
        std::vector<int> map_obstacle_ids = config["obstacle_ids_mav"].as<std::vector<int>>();
        std::vector<int> map_rov_obstacle_ids = config["obstacle_ids_rov"].as<std::vector<int>>();
        int unknown_id = config["unknown_id"].as<int>();
        std::vector<double> map_start_location = config["start_location"].as<std::vector<double>>();
        std::vector<double> map_goal_location = config["goal_location"].as<std::vector<double>>();
        double map_explorable_area = config["explorable_free_space"].as<double>();

      bool config_valid = true;
      if (map_resolution.size() == 2 && map_goal_location.size() == 2) {
        config_valid &= (bool)map_name.size();
        config_valid &= (map_width > 0.0) && (map_height > 0.0) && (map_resolution[0] > 0) && (map_resolution[1] > 0);
        config_valid &= (std::abs(1 - ((map_width/map_resolution[0]) / (map_height/map_resolution[1]))) < 0.01);
        config_valid &= (n_data_classes) > 0 && (n_data_classes < 255);
        int max_id = std::pow(n_data_classes, 3) + std::pow(n_data_classes, 2) - 1;
        for (int id:map_ids) config_valid &= (id >= 0) && (id <= max_id);
        for (int id:map_obstacle_ids) config_valid &= (id >= 0) && (id <= max_id);
        for (int id:map_rov_obstacle_ids) config_valid &= (id >= 0) && (id <= max_id);
        config_valid &= (unknown_id >= 0) && (unknown_id <= max_id);
        config_valid &= (map_goal_location[0] >= 0.0) && (map_goal_location[1] >= 0.0) && (map_goal_location[0] < map_width) && (map_goal_location[1] < map_height);
        config_valid &= (map_start_location[0] >= 0.0) && (map_start_location[1] >= 0.0) && (map_start_location[0] < map_width) && (map_start_location[1] < map_height);
        config_valid &= (map_start_location[0] != map_goal_location[0]) || (map_start_location[1] >= map_goal_location[1]);
      } else {
        config_valid = false;
      }
      if (config_valid) {
        horizontal_units_ = map_resolution[0];
        vertical_units_ = map_resolution[1];
        width_ = map_width;
        height_ = map_height;
        obstacle_ids_ = map_obstacle_ids;
        rov_obstacle_ids_ = map_rov_obstacle_ids;
        unknown_id_ = unknown_id;
        m_per_pixel_ = width_ / horizontal_units_;
        goal_location_ = Eigen::Vector2i((int)(map_goal_location[0]/m_per_pixel_), (int)(map_goal_location[1]/m_per_pixel_));
        goal_position_ = Eigen::Vector3d(map_goal_location[0], map_goal_location[1], 0.0);
        start_location_ = Eigen::Vector2i((int)(map_start_location[0]/m_per_pixel_), (int)(map_start_location[1]/m_per_pixel_));
        start_position_ = Eigen::Vector3d(map_start_location[0], map_start_location[1], 0.0);
        for (int id:map_ids) if (id > max_cost_) max_cost_ = id;
        min_rov_cost_ = std::pow(15, 3) + std::pow(15, 2) - 1.0;
        for (int id:map_ids) if (!idIsRovObstacle(id)) if (id < min_rov_cost_) min_rov_cost_ = id;
        max_rov_cost_ = 0;
        for (int id:map_ids) if (!idIsRovObstacle(id)) if (id > max_rov_cost_) max_rov_cost_ = id;
        clearMap();
        default_init = false;
        ROS_DEBUG("[MapServerPlanExp] Successfully read config file: %s", map_config_path.c_str());
      } else {
        ROS_WARN("[MapServerPlanExp] Config file (%s) invalid:", map_config_path.c_str());
        default_init = true;
      }
    }
  }
  catch (...) {
    ROS_WARN("[MapServerPlanExp] Unable to read or parse config file: %s", map_config_path.c_str());
    default_init = true;
  }

#endif

  if (default_init) {
    clearMap();
    m_per_pixel_ = width_ / horizontal_units_;
    goal_location_ = Eigen::Vector2i((int)(39.5/m_per_pixel_), (int)(29.5/m_per_pixel_));
    goal_position_ = Eigen::Vector3d(39.5, 29.5, 0.0);
    start_location_ = Eigen::Vector2i((int)(0.0/m_per_pixel_), (int)(0.0/m_per_pixel_));
    start_position_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    unknown_id_ = std::pow(15, 3) + std::pow(15, 2) - 1;
    max_cost_ = 0.0;
    min_rov_cost_ = 1.0;
    max_rov_cost_ = std::pow(15, 3) + std::pow(15, 2) - 1.0;
    obstacle_ids_.push_back(obstacle_id_);
    rov_obstacle_ids_.push_back(obstacle_id_);
  }

  if (ignore_mav_obstacles) obstacle_ids_.clear();

  current_path_map_estimate_ = Eigen::ArrayXXi::Zero(vertical_units_, horizontal_units_);
  path_requested_ = p_compute_path_;
  planner_thread_done_ = true;

  if (p_unknown_init_cost_str == "min") {
    unknown_init_cost_ = min_rov_cost_;
    ROS_INFO("[MapServerPlanExp] Using init mode: min (cost: %lf)", unknown_init_cost_);
  } else if (p_unknown_init_cost_str == "max") {
    unknown_init_cost_ = max_rov_cost_;
    ROS_INFO("[MapServerPlanExp] Using init mode: max (cost: %lf)", unknown_init_cost_);
  } else {
    try {
      unknown_init_cost_ = std::stod(p_unknown_init_cost_str);
    }
    catch (...) {
      ROS_WARN("[MapServerPlanExp] Unable to parse p_unknown_init_cost_str: %s Defaulting to min_rov_cost: %lf", p_unknown_init_cost_str.c_str(), min_rov_cost_);
      unknown_init_cost_ = min_rov_cost_;
    }
  }

  planexp_base_planners::PlannerParams planner_params = {
      .px_x=(int)horizontal_units_,
      .px_y=(int)vertical_units_,
      .width=width_, .height=height_,
      .min_cost=min_rov_cost_,
      .max_cost=max_rov_cost_,
      .init_cost=unknown_init_cost_,
      .start=start_position_.head(2),
      .goal=goal_position_.head(2),
      .obstacle_ids=rov_obstacle_ids_,
      .unknown_id=unknown_id_,
      .time_limit=planner_time};

  if (planner_type_str == "rrt_star") planner_type_ = planexp_base_planners::RRT_STAR_PLANNER;
  else if (planner_type_str == "prm_star") planner_type_ = planexp_base_planners::PRM_STAR_PLANNER;
  else {
    ROS_WARN("Planner type %s is unknown and plannner was set to default.", planner_type_str.c_str());
    planner_type_ = planexp_base_planners::PRM_STAR_PLANNER;
    planner_type_str = "prm_star";
  }

  ROS_DEBUG("[MapServerPlanExp] Planner type: %s", planner_type_str.c_str());

  switch (planner_type_) {
    case planexp_base_planners::RRT_STAR_PLANNER:
      planner_ = std::unique_ptr<planexp_base_planners::PlannerBase>(new planexp_base_planners::RRTStarPlanner(planner_params));
      break;
    case planexp_base_planners::PRM_STAR_PLANNER:
      planner_ = std::unique_ptr<planexp_base_planners::PlannerBase>(new planexp_base_planners::PRMStarPlanner(planner_params));
      break;
    case planexp_base_planners::UNKNOWN_PLANNER:
    default:
      planner_ = std::unique_ptr<planexp_base_planners::PlannerBase>(new planexp_base_planners::RRTStarPlanner(planner_params));
  }
  planner_->setup();

  ROS_DEBUG("[MapServerPlanExp] Initialization finished!");
}

void MapServerPlanExp::mapDataCallback(const planexp_msgs::MapData& msg) {
  ROS_DEBUG("[MapServerPlanExp] Callback called!");
  bool pre_update_goal_status = goalExplored();
  Eigen::ArrayXXi new_map_data = msg2EigenArray(msg.data);
  Eigen::Vector2i new_map_data_location = position2pixelLocation(msg2Position(msg.pose));

  //change from center to corner
  new_map_data_location.x() = new_map_data_location.x()-(int)(new_map_data.cols()/2);
  new_map_data_location.y() = new_map_data_location.y()-(int)(new_map_data.rows()/2);

  /*//enforce lower bounds
  new_map_data_location.x() = (new_map_data_location.x() >= 0) ? new_map_data_location.x() : 0;
  new_map_data_location.y() = (new_map_data_location.y() >= 0) ? new_map_data_location.y() : 0;

  //enforce upper bounds
  new_map_data_location.x() = (new_map_data_location.x()+new_map_data.cols() < map_cost_->cols()) ? new_map_data_location.x() : map_cost_->cols()-new_map_data.cols();
  new_map_data_location.y() = (new_map_data_location.y()+new_map_data.rows() < map_cost_->rows()) ? new_map_data_location.y() : map_cost_->rows()-new_map_data.rows();*/

  for (int i = 0; i < new_map_data.rows(); ++i) {
    for (int j = 0; j < new_map_data.cols(); ++j) {
      if (0 <= new_map_data_location.y()+i && 0 <= new_map_data_location.x()+j && new_map_data_location.y()+i < map_cost_->rows() && new_map_data_location.x()+j < map_cost_->cols()) {
        int data_point = new_map_data(i, j);
        (*map_cost_)(new_map_data_location.y() + i, new_map_data_location.x() + j) = (double) data_point;

        if (p_use_uncertainty_map_) (*map_uncertainty_)(new_map_data_location.y()+i,new_map_data_location.x()+j) = 1.0;

        if ((*map_state_)(new_map_data_location.y() + i, new_map_data_location.x() + j) == MSR_UNKNOWN) {
          mean_cost_ = mean_cost_ * ((double)mean_counter_/(mean_counter_ + 1)) + data_point / (mean_counter_ + 1);
          ++mean_counter_;
        } else if ((*map_state_)(new_map_data_location.y() + i, new_map_data_location.x() + j) == MSR_FREE) {
          mean_cost_ = mean_cost_ + (data_point-(*map_cost_)(new_map_data_location.y() + i, new_map_data_location.x() + j)) / mean_counter_;
        }
        if (idIsObstacle(data_point)) {
          (*map_state_)(new_map_data_location.y() + i, new_map_data_location.x() + j) = MSR_OCCUPIED;
        } else {
          (*map_state_)(new_map_data_location.y() + i, new_map_data_location.x() + j) = MSR_FREE;
        }
        if (data_point > max_cost_) max_cost_ = data_point;
      }
    }
  }

  planner_->update_states(new_map_data_location, new_map_data);

  if (p_compute_closest_observed_) setClosestObservedMaps(new_map_data_location, new_map_data_location + Eigen::Vector2i(new_map_data.cols(), new_map_data.rows())); //TODO: Finish implementation here!!!!

  if (p_compute_reachability_) {
    for (int i = 0; i < new_map_data.rows(); ++i) {
      for (int j = 0; j < new_map_data.cols(); ++j) {
        if (0 <= new_map_data_location.y() + i && 0 <= new_map_data_location.x() + j &&
            new_map_data_location.y() + i < map_cost_->rows() && new_map_data_location.x() + j < map_cost_->cols()) {
          setReachabilityMap(new_map_data_location + Eigen::Vector2i(j, i));
        }
      }
    }
  }
  if (goalExplored() && !pre_update_goal_status && unknown_init_cost_ != min_rov_cost_) planner_->update_unknown_cost(min_rov_cost_);
  publishVisualisation();
  if (p_compute_path_ && planner_thread_done_ && path_requested_) {
    path_requested_ = false;
    planner_thread_ = new std::thread(&MapServerPlanExp::computeCurrentPathEstimate, this);
  }
}

void MapServerPlanExp::depthDataCallback(const planexp_msgs::MapData& msg) {
  if (!p_use_height_map_) return;
  ROS_DEBUG("[MapServerPlanExp] Depth callback called!");
  Eigen::ArrayXXd new_map_data = msg2EigenArrayd(msg.data);
  Eigen::Vector2i new_map_data_location = position2pixelLocation(msg2Position(msg.pose));

  //change from center to corner
  new_map_data_location.x() = new_map_data_location.x()-(int)(new_map_data.cols()/2);
  new_map_data_location.y() = new_map_data_location.y()-(int)(new_map_data.rows()/2);

  for (int i = 0; i < new_map_data.rows(); ++i) {
    for (int j = 0; j < new_map_data.cols(); ++j) {
      if (0 <= new_map_data_location.y()+i && 0 <= new_map_data_location.x()+j && new_map_data_location.y()+i < map_height_->rows() && new_map_data_location.x()+j < map_height_->cols()) {
        double depth = new_map_data(i, j);
        (*map_height_)(new_map_data_location.y() + i, new_map_data_location.x() + j) = msg.pose.position.z - depth;
        if (p_use_uncertainty_map_) (*map_height_uncertainty_)(new_map_data_location.y() + i, new_map_data_location.x() + j) = 1.0;
      }
    }
  }

}

Eigen::ArrayXXi MapServerPlanExp::msg2EigenArray(const std_msgs::Int32MultiArray &msg) {
  //float dstride0 = msg.layout.dim[0].stride;
  //float dstride1 = msg.layout.dim[1].stride;
  float h = msg.layout.dim[0].size;
  float w = msg.layout.dim[1].size;

  std::vector<int> data = msg.data;

  Eigen::Map<Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> mat(data.data(), h, w);
  Eigen::ArrayXXi data_arr = (Eigen::ArrayXXi)mat;
  return data_arr;
}

Eigen::ArrayXXd MapServerPlanExp::msg2EigenArrayd(const std_msgs::Int32MultiArray &msg) {
  //float dstride0 = msg.layout.dim[0].stride;
  //float dstride1 = msg.layout.dim[1].stride;
  float h = msg.layout.dim[0].size;
  float w = msg.layout.dim[1].size;

  std::vector<int> data = msg.data;

  Eigen::Map<Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> mat(data.data(), h, w);
  Eigen::ArrayXXi data_arr = (Eigen::ArrayXXi)mat;
  return (data_arr.cast<double>()) / 100.0;
}

Eigen::Vector3d MapServerPlanExp::msg2Position(const geometry_msgs::Pose& msg) {
  Eigen::Vector3d position;
  position.x() = msg.position.x;
  position.y() = msg.position.y;
  position.z() = msg.position.z;
  return position;
}

Eigen::Vector2i MapServerPlanExp::position2pixelLocation(const Eigen::Vector3d position) {
  return Eigen::Vector2i((int)(position[0] * map_state_->cols() / width_), (int)(position[1] * map_state_->rows() / height_));
}

void MapServerPlanExp::publishVisualisation() {
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.seq = ++vis_seq_id_;

  cv::Mat cost_image;
  costMap2image(cost_image);
  sensor_msgs::ImagePtr cost_msg = cv_bridge::CvImage(header, "bgr8", cost_image).toImageMsg();
  if (cost_map_vis_pub_.getNumSubscribers()) cost_map_vis_pub_.publish(cost_msg);

  if (p_use_uncertainty_map_) {
    cv::Mat uncertainty_image;
    uncertaintyMap2image(uncertainty_image);
    sensor_msgs::ImagePtr uncertainty_msg = cv_bridge::CvImage(header, "bgr8", uncertainty_image).toImageMsg();
    if (uncertainty_map_vis_pub_.getNumSubscribers()) uncertainty_map_vis_pub_.publish(uncertainty_msg);
  }

  cv::Mat occupancy_image;
  occupancyMap2image(occupancy_image);
  sensor_msgs::ImagePtr occupancy_msg = cv_bridge::CvImage(header, "mono8", occupancy_image).toImageMsg();
  if (occupancy_map_vis_pub_.getNumSubscribers()) occupancy_map_vis_pub_.publish(occupancy_msg);

#if RVIZ_VISUALIZATION
  if (cost_map_rviz_pub_.getNumSubscribers()) publishRvizCostMap();
#if CREATE_RRT_PLANNER_MAP
  if (path_estimate_vis_pub_.getNumSubscribers()) publishPathEstimate();
#endif
#endif
}

void MapServerPlanExp::occupancyMap2image(cv::Mat& occupancy_image) {
  occupancy_image.convertTo(occupancy_image, CV_8U);
  cv::eigen2cv((Eigen::MatrixXi) *map_state_, occupancy_image);
  occupancy_image.convertTo(occupancy_image, CV_8U, 127, 0);
}

void MapServerPlanExp::uncertaintyMap2image(cv::Mat& uncertainty_image) {
  if (!p_use_uncertainty_map_) return;
  uncertainty_image.convertTo(uncertainty_image, CV_32F);
  cv::eigen2cv((Eigen::MatrixXd) *map_uncertainty_, uncertainty_image);
  uncertainty_image.convertTo(uncertainty_image, CV_8U, 255.0, 0);
  cv::applyColorMap(uncertainty_image, uncertainty_image, cv::COLORMAP_JET);
}

void MapServerPlanExp::costMap2image(cv::Mat& cost_image) {
  cost_image.convertTo(cost_image, CV_32F);
  cv::eigen2cv((Eigen::MatrixXd) *map_cost_, cost_image);
  double max_img, min_img;
  cv::minMaxLoc( cost_image, &min_img, &max_img);
  cost_image.convertTo(cost_image, CV_8U, 255.0/max_img, 0);
  cv::applyColorMap(cost_image, cost_image, cv::COLORMAP_JET);
}

void MapServerPlanExp::heightMap2image(cv::Mat& height_image) {
  if (!p_use_height_map_) return;
  height_image.convertTo(height_image, CV_32F);
  cv::eigen2cv((Eigen::MatrixXd) *map_cost_, height_image);
  double max_img, min_img;
  cv::minMaxLoc(height_image, &min_img, &max_img);
  height_image.convertTo(height_image, CV_8U, 255.0/max_img, 0);
  cv::applyColorMap(height_image, height_image, cv::COLORMAP_JET);
}

void MapServerPlanExp::heightUncertaintyMap2image(cv::Mat& height_uncertainty_image) {
  if (!p_use_uncertainty_map_ && !p_use_height_map_) return;
  height_uncertainty_image.convertTo(height_uncertainty_image, CV_32F);
  cv::eigen2cv((Eigen::MatrixXd) *map_height_uncertainty_, height_uncertainty_image);
  height_uncertainty_image.convertTo(height_uncertainty_image, CV_8U, 255.0, 0);
  cv::applyColorMap(height_uncertainty_image, height_uncertainty_image, cv::COLORMAP_JET);
}

void MapServerPlanExp::createRRTPlannerMap(cv::Mat& rrt_image) {
  rrt_image.convertTo(rrt_image, CV_32F);
  cv::eigen2cv((Eigen::MatrixXd) *map_cost_, rrt_image);
  rrt_image.convertTo(rrt_image, CV_32F);
  for (int i = 0; i < rrt_image.rows; ++i) {
    for (int j = 0; j < rrt_image.cols; ++j) {
      if ( idIsRovObstacle((*map_cost_)(i, j))) rrt_image.at<float>(i, j) = rov_default_obstacle_id_;
      if ((*map_state_)(i, j) == MSR_UNKNOWN) rrt_image.at<float>(i, j) = unknown_id_;
    }
  }
  labels2image(rrt_image);
}

bool MapServerPlanExp::clearMapCallback(std_srvs::Empty::Request& request,
                                        std_srvs::Empty::Response& response) {
  return clearMap();
}

bool MapServerPlanExp::saveMapCallback(planexp_msgs::FilePath::Request& request,
                                        planexp_msgs::FilePath::Response& response) {
  return saveMap(request.file_path);
}

bool MapServerPlanExp::loadMapCallback(planexp_msgs::FilePath::Request& request,
                                        planexp_msgs::FilePath::Response& response) {
  return loadMap(request.file_path);
}

bool MapServerPlanExp::publishMapCallback(std_srvs::Empty::Request& request,
                                           std_srvs::Empty::Response& response) {
  publishVisualisation();
  return true;
}

bool MapServerPlanExp::goalExploredCallback(std_srvs::Trigger::Request &request,
                                             std_srvs::Trigger::Response &response) {
  response.success = goalExplored();

  if (p_compute_reachability_) {
    if ((*map_reachability_)(AM(goal_location_)) == MSR_REACHABLE)
      response.message = "Goal is reachable!";
  }

  return true;
}

bool MapServerPlanExp::clearMap() {
  (*map_state_) = OccupancyMap_t::Ones(vertical_units_, horizontal_units_) * MSR_UNKNOWN;
  (*map_cost_) = CostMap_t::Zero(vertical_units_, horizontal_units_);
  if (p_use_uncertainty_map_) (*map_uncertainty_) = UncertaintyMap_t::Zero(vertical_units_, horizontal_units_);
  if (p_use_height_map_) (*map_height_) = HeightMap_t::Zero(vertical_units_, horizontal_units_);
  if (p_use_uncertainty_map_ && p_use_height_map_) (*map_height_uncertainty_) = UncertaintyMap_t::Zero(vertical_units_, horizontal_units_);

  if (p_compute_closest_observed_) {
    (*map_cost_closest_) = CostMap_t::Zero(vertical_units_, horizontal_units_);
    (*map_distance_closest_) =
        CostMap_t::Ones(vertical_units_, horizontal_units_) * sqrt(width_ * width_ + height_ * height_);
  }

  if (p_compute_reachability_) {
    (*map_reachability_) = ReachabilityMap_t::Ones(vertical_units_, horizontal_units_) * MSR_REACHABILITY_UNKNOWN;
    (*map_reachability_)(AM(start_location_)) = MSR_REACHABLE;
    connected_areas_.clear();
    std::vector<Eigen::Vector2i> start_area;
    start_area.push_back(start_location_);
    connected_areas_.push_back(start_area);
    max_area_id_ = 1;
  }

  mean_cost_ = 0;
  mean_counter_ = 0;
  return true;
}

bool MapServerPlanExp::saveMap(std::string path) {
  cv::Mat cost_image;
  costMap2image(cost_image);
  std::string cost_path = path + "_cost";
  cv::imwrite(cost_path + ".png", cost_image);
  writeToCSVfile(cost_path + ".csv", map_cost_->matrix());

  if (p_use_uncertainty_map_) {
    cv::Mat uncertainty_image;
    uncertaintyMap2image(uncertainty_image);
    std::string uncertainty_path = path + "_uncertainty";
    cv::imwrite(uncertainty_path + ".png", uncertainty_image);
    writeToCSVfile(uncertainty_path + ".csv", map_uncertainty_->matrix());
  }

  cv::Mat occupancy_image;
  occupancyMap2image(occupancy_image);
  std::string occupancy_path = path + "_occupancy";
  cv::imwrite(occupancy_path + ".png", occupancy_image);
  writeToCSVfile(occupancy_path + ".csv", map_state_->matrix());

  if (p_use_height_map_) {
    cv::Mat height_image;
    heightMap2image(height_image);
    std::string height_path = path + "_height";
    cv::imwrite(height_path + ".png", height_image);
    writeToCSVfile(height_path + ".csv", map_height_->matrix());
    if (p_use_uncertainty_map_) {
      cv::Mat height_uncertainty_image;
      heightUncertaintyMap2image(height_uncertainty_image);
      std::string height_uncertainty_path = path + "_height_uncertainty";
      cv::imwrite(height_uncertainty_path + ".png", height_uncertainty_image);
      writeToCSVfile(height_uncertainty_path + ".csv", map_height_uncertainty_->matrix());
    }
  }

#if CREATE_RRT_PLANNER_MAP
  cv::Mat rrt_image;
  createRRTPlannerMap(rrt_image);
  std::string rrt_map_path = path;
  cv::imwrite(rrt_map_path + ".png", rrt_image);
#endif

  //TODO: Add implementation for closest distance and other maps here

  return true;
}

bool MapServerPlanExp::loadMap(std::string path) {

  std::string cost_path = path + "_cost.csv";
  (*map_cost_) = load_csv<CostMap_t>(cost_path);

  if (p_use_uncertainty_map_) {
    std::string uncertainty_path = path + "_uncertainty.csv";
    (*map_uncertainty_) = load_csv<UncertaintyMap_t>(uncertainty_path);
  }

  std::string occupancy_path = path + "_occupancy.csv";
  (*map_state_) = load_csv<OccupancyMap_t>(occupancy_path);

  if (p_use_height_map_) {
    std::string height_path = path + "_height.csv";
    (*map_height_) = load_csv<HeightMap_t>(height_path);

    if (p_use_uncertainty_map_) {
      std::string height_uncertainty_path = path + "_height_uncertainty.csv";
      (*map_height_uncertainty_) = load_csv<UncertaintyMap_t>(height_uncertainty_path);
    }
  }

  mean_counter_ = 0;
  mean_cost_ = 0;

  for (int i = 0; i < (*map_state_).rows(); ++i) {
    for (int j = 0; j < (*map_state_).cols(); ++j) {
      if ((*map_state_)(i,j) == MSR_FREE) {
        mean_cost_ += (*map_cost_)(i,j);
        ++mean_counter_;
      }
    }
  }
  mean_cost_ /= mean_counter_;

  //TODO: Add implementation for closest distance and other maps here

  return true;
}

#if RVIZ_VISUALIZATION
Eigen::ArrayXXf MapServerPlanExp::eigen2gridMap() {
  Eigen::ArrayXXf result = Eigen::ArrayXXf::Zero(map_state_->rows(), map_state_->cols());
  for(int i = 0; i<map_state_->rows(); ++i) {
    for(int j = 0; j<map_state_->cols(); ++j) {
      signed char state = (*map_state_)(i, j);
      int path = (current_path_map_estimate_)(i, j); 
      if (path != 0) {result(i,j) = cost2gridMapColor(-0.5);}
      else if (state == MSR_UNKNOWN) {result(i,j) = cost2gridMapColor(-1);}
      else if (state == MSR_FREE) {result(i,j) = cost2gridMapColor((*map_cost_)(i, j) / max_cost_);}
      else {result(i,j) = cost2gridMapColor(0);} 
    }
  }
  return result;
}

void MapServerPlanExp::publishRvizCostMap() {

  grid_map_msgs::GridMap msg;

  msg.info.header.stamp = ros::Time::now();
  msg.info.header.frame_id = "world";
  msg.info.header.seq = ++cost_map_rviz_seq_id_;
  msg.info.resolution = m_per_pixel_;
  msg.info.length_x = width_;
  msg.info.length_y = height_;
  msg.info.pose.position.x = width_/2.0;
  msg.info.pose.position.y = height_/2.0;
  msg.info.pose.position.z = 0.0;
  msg.info.pose.orientation.x = 0.0;
  msg.info.pose.orientation.y = 0.0;
  msg.info.pose.orientation.z = 0.0;
  msg.info.pose.orientation.w = 1.0;

  const std::vector<std::string>& layers = {"elevation", "color"};

  msg.layers = layers;

  msg.data.clear();


  Eigen::ArrayXXf color = eigen2gridMap().transpose().reverse().eval();
  Eigen::ArrayXXf elevation;
  if (p_use_height_map_) elevation = map_height_->cast<float>().transpose().reverse().eval();
  else elevation = Eigen::ArrayXXf::Ones(color.rows(), color.cols()) * -0.1;


  std_msgs::Float32MultiArray elevationDataArray;
  matrixEigenCopyToMultiArrayMessage(elevation, elevationDataArray);
  msg.data.push_back(elevationDataArray);
  std_msgs::Float32MultiArray colorDataArray;
  matrixEigenCopyToMultiArrayMessage(color, colorDataArray);
  msg.data.push_back(colorDataArray);


  msg.outer_start_index = 0;
  msg.inner_start_index = 0;

  cost_map_rviz_pub_.publish(msg);
}

#if CREATE_RRT_PLANNER_MAP
void MapServerPlanExp::publishPathEstimate() {
  nav_msgs::Path msg;

  msg.header.stamp = ros::Time::now();
  msg.header.seq = ++path_estimate_vis_seq_id_;
  msg.header.frame_id = "world";

  std::unique_lock<std::mutex> guard(current_path_estimate_mutex_);
  std::vector<Eigen::Vector2d> current_path_estimate = current_path_estimate_;
  guard.unlock();

  int idx = 0;
  for (auto point:current_path_estimate) {
    geometry_msgs::PoseStamped pose;

    pose.header.frame_id = "world";
    pose.header.seq = ++idx;
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    msg.poses.push_back(pose);
  }

  path_estimate_vis_pub_.publish(msg);
}
#endif
#endif

#if CREATE_RRT_PLANNER_MAP
void MapServerPlanExp::labels2image(cv::Mat &labels) {
  /*
   * Takes a cv::Mat in the format CV_32FC1 containing the labels and returns the corresponding color representation.
   */
  cv::Mat red, green, blue;
  int num_classes_ = 15;
  double step = num_classes_/255.0;
  int c2 = std::pow(num_classes_,2);
  int delta_s = 1.0/step;

  labels.convertTo(red, CV_32FC1);
  red.convertTo(red, CV_32FC1, 1.0/c2, 0); //red = labels/c2;
  floorImg(red);
  red.convertTo(red, CV_8UC1, delta_s, 0); //red *= delta_s;

  labels.convertTo(green, CV_32F);
  moduloImg(green, c2);
  green.convertTo(green, CV_32F);
  green.convertTo(green, CV_32F, 1.0/num_classes_, 0); //green /= num_classes_;
  floorImg(green);
  green.convertTo(green, CV_8U, delta_s, 0); //green *= delta_s;

  labels.convertTo(blue, CV_32F);
  moduloImg(blue, c2);
  moduloImg(blue, num_classes_);
  blue.convertTo(blue, CV_8U, delta_s, 0); //blue *= delta_s;

  std::vector<cv::Mat> channels;
  channels.push_back(blue);
  channels.push_back(green);
  channels.push_back(red);

  labels.convertTo(labels, CV_8UC3);
  cv::merge(channels, labels);
}

void MapServerPlanExp::floorImg(cv::Mat& image) {
  if (image.channels() == 1) {
    image.convertTo(image, CV_32FC1);
    for(int i=0; i<image.rows; i++)
      for(int j=0; j<image.cols; j++)
        image.at<float>(i, j) = std::floor(image.at<float>(i, j));
    image.convertTo(image, CV_32SC1);
  }
  else if (image.channels() == 3) {
    image.convertTo(image, CV_32FC3);
    for(int i=0; i<image.rows; i++) {
      for(int j=0; j<image.cols; j++) {
        cv::Point3f point;
        point.x = std::floor(image.at<cv::Point3f>(i, j).x);
        point.y = std::floor(image.at<cv::Point3f>(i, j).y);
        point.z = std::floor(image.at<cv::Point3f>(i, j).z);
        image.at<cv::Point3f>(i, j) = point;
      }
    }
    image.convertTo(image, CV_32SC3);
  }
}

void MapServerPlanExp::moduloImg(cv::Mat& image, int n) {
  if (image.channels() == 1) {
    image.convertTo(image, CV_32FC1);
    for(int i=0; i<image.rows; i++)
      for(int j=0; j<image.cols; j++)
        image.at<float>(i, j) = std::fmod(image.at<float>(i, j), n);
    image.convertTo(image, CV_32SC1);
  }
  else if (image.channels() == 3) {
    image.convertTo(image, CV_32FC3);
    for(int i=0; i<image.rows; i++) {
      for(int j=0; j<image.cols; j++) {
        cv::Point3f point;
        point.x = std::fmod(image.at<cv::Point3f>(i, j).x, n);
        point.y = std::fmod(image.at<cv::Point3f>(i, j).y, n);
        point.z = std::fmod(image.at<cv::Point3f>(i, j).z, n);
        image.at<cv::Point3f>(i, j) = point;
      }
    }
    image.convertTo(image, CV_32SC3);
  }
}
#endif

double MapServerPlanExp::getCostAtLocation(Eigen::Vector2d position) {
  if (0<= position[0] && 0<= position[1] && width_> position[0] && height_> position[1]) {
    return (*map_cost_)((int)(position[1] / m_per_pixel_), (int)(position[0] / m_per_pixel_));
  }
  return -1.0;
}

int MapServerPlanExp::getStatusAtLocation(Eigen::Vector2d position) {
  if (0<= position[0] && 0<= position[1] && width_> position[0] && height_> position[1]) {
    return (*map_state_)((int)(position[1] / m_per_pixel_), (int)(position[0] / m_per_pixel_));
  }
  return MSR_UNKNOWN;
}

double MapServerPlanExp::getUncertaintyAtLocation(Eigen::Vector2d position) {
  if (!p_use_uncertainty_map_) return -1.0;
  if (0<= position[0] && 0<= position[1] && width_> position[0] && height_> position[1]) {
    return (*map_uncertainty_)((int)(position[1] / m_per_pixel_), (int)(position[0] / m_per_pixel_));
  }
  return -1.0;
}

double MapServerPlanExp::getHeightAtLocation(Eigen::Vector2d position) {
  if (!p_use_height_map_) return -1;
  if (0<= position[0] && 0<= position[1] && width_> position[0] && height_> position[1]) {
    return (*map_height_)((int)(position[1] / m_per_pixel_), (int)(position[0] / m_per_pixel_));
  }
  return -1;
}

double MapServerPlanExp::getHeightUncertaintyAtLocation(Eigen::Vector2d position) {
  if (!p_use_uncertainty_map_ && !p_use_height_map_) return -1.0;
  if (0<= position[0] && 0<= position[1] && width_> position[0] && height_> position[1]) {
    return (*map_height_uncertainty_)((int)(position[1] / m_per_pixel_), (int)(position[0] / m_per_pixel_));
  }
  return -1.0;
}

double MapServerPlanExp::getClosestCostAtLocation(Eigen::Vector2d position) {
  if (!p_compute_closest_observed_) return -1.0;
  if (0<= position[0] && 0<= position[1] && width_> position[0] && height_> position[1]) {
   return (*map_cost_closest_)((int)(position[1] / m_per_pixel_), (int)(position[0] / m_per_pixel_));
 }
 return -1.0;
}

void MapServerPlanExp::setClosestObservedMaps(Eigen::Vector2i top_left, Eigen::Vector2i bottom_right) {
  Eigen::Vector2i window_size = bottom_right - top_left;
  Eigen::Vector2i closest_observed;
  int off_x = 15;
  int off_y = 10;
  for (int i = top_left[0]-off_x; i <= bottom_right[0]+off_x; ++i) {
    if (i < 0 || horizontal_units_ < i) continue;
    for (int j = top_left[1]-off_y; j <= bottom_right[1]+off_y; ++j) {
      if (j < 0 || vertical_units_ < j) continue;
      if (((*map_state_)(j, i)) != MSR_UNKNOWN) continue;
      int closest_x = std::max(top_left[0], i);
      closest_x = std::min(bottom_right[0], closest_x);
      int closest_y = std::max(top_left[1], j);
      closest_y = std::min(bottom_right[1], closest_y);
      double distance = sqrt(pow(i-closest_x, 2) + pow(j-closest_y, 2));
      if ((*map_distance_closest_)(j, i) <= distance) continue;
      (*map_distance_closest_)(j, i) = distance;
      (*map_cost_closest_)(j, i) = (*map_cost_)(closest_y, closest_x);
    }
  }
}

void MapServerPlanExp::setReachabilityMap(Eigen::Vector2i start_point) {
  if ((*map_reachability_)(AM(start_point)) != MSR_REACHABILITY_UNKNOWN || (*map_state_)(AM(start_point)) == MSR_OCCUPIED) return;
  std::vector<Eigen::Vector2i> point_vector;
  point_vector.push_back(start_point);
  std::vector<int> found_area_ids;
  int current_area_id = max_area_id_ + 1;
  std::vector<Eigen::Vector2i> current_area;
  while (!point_vector.empty()) {
    Eigen::Vector2i point = point_vector.back();
    point_vector.pop_back();
    for (int i = -1; i <= 1; ++i) {
      for (int j = -1; j <= 1; ++j) {
        Eigen::Vector2i candidate = point + Eigen::Vector2i(i,j);
        if (!(0 <= candidate.y() && 0 <= candidate.x() && candidate.y() < map_cost_->rows() && candidate.x() < map_cost_->cols())) continue;
        if ((*map_state_)(AM(candidate)) == MSR_FREE) {
          int reachability_state = (*map_reachability_)(AM(candidate));
          if (reachability_state == current_area_id) continue;
          if (reachability_state != MSR_REACHABILITY_UNKNOWN) {
            if (std::find(found_area_ids.begin(), found_area_ids.end(), reachability_state) == found_area_ids.end()) found_area_ids.push_back(reachability_state);
            continue;
          }
          (*map_reachability_)(AM(candidate)) = current_area_id;
          current_area.push_back(candidate);
          point_vector.push_back(candidate);
        }
      }
    }
  }
  if (found_area_ids.empty()) {
    //new unconnected area
    connected_areas_.push_back(current_area);
    ++max_area_id_;
  }
  else {
    int min_area_id = INT32_MAX;
    for (int area_id:found_area_ids) if (area_id < min_area_id) min_area_id = area_id;
    //connected area found
    for (int area_id:found_area_ids) {
      if (area_id != min_area_id) {
        current_area.insert( current_area.end(), connected_areas_[area_id-1].begin(), connected_areas_[area_id-1].end());
        connected_areas_[area_id-1].clear();
      }
    }
    connected_areas_[min_area_id-1].insert( connected_areas_[min_area_id-1].end(), current_area.begin(), current_area.end());
  }
}

bool MapServerPlanExp::getReachabilityAtLocation(Eigen::Vector2d position) {
  if (!p_compute_reachability_) return false;
  if (0<= position[0] && 0<= position[1] && width_> position[0] && height_> position[1]) {
    return MSR_REACHABLE == ((*map_reachability_)((int)(position[1] / m_per_pixel_), (int)(position[0] / m_per_pixel_)));
  }
  return false;
}

void MapServerPlanExp::computeCurrentPathEstimate() {
  planner_thread_done_ = false;

  double path_cost;
  std::vector<Eigen::Vector2d> current_path_estimate;
  planexp_base_planners::ResultStatus result = planner_->plan(&current_path_estimate, path_cost);
  if (store_paths_) {
    const int length = 4;
    std::string counter_str = std::to_string(path_counter_ + 1);
    int precision = length - std::min(length, (int)(counter_str.length()));
    std::string file_name = counter_str.insert(0, precision, '0');
    planner_->store_path(file_name, paths_path_);
  }
  if (planner_status_pub_.getNumSubscribers()) publishPlannerStatus(result);
  Eigen::ArrayXXi current_path_map_estimate = computePathMap(current_path_estimate);
  std::lock_guard<std::mutex> guard(current_path_estimate_mutex_);
  path_counter_ += 1;
  current_path_estimate_ = current_path_estimate;
  current_path_map_estimate_ = current_path_map_estimate;
  planner_thread_done_ = true;
}

Eigen::ArrayXXi MapServerPlanExp::computePathMap(std::vector<Eigen::Vector2d>& path) {
  std::vector<Eigen::Vector2i> discrete_path;
  Eigen::ArrayXXi path_map = Eigen::ArrayXXi::Zero(vertical_units_, horizontal_units_);
  discrete_path.push_back(position2pixelLocation(Eigen::Vector3d(path[0][0], path[0][1], 0.0)));
  for (int j = 1; j < path.size(); ++j) {
    double distance = (path[j] - path[j-1]).norm();
    int samples = (int)(distance/(m_per_pixel_/3.0));
    if (samples > 0) {
      Eigen::Vector2d dpose = (path[j]-path[j-1]) / samples;

      double dt = distance / samples;

      for (int i = 0; i<samples; i++) {
        Eigen::Vector2d inter_pos = path[j-1] + i * dpose;
        Eigen::Vector2i position = position2pixelLocation(Eigen::Vector3d(inter_pos[0], inter_pos[1], 0.0));
        if (discrete_path.back() != position) discrete_path.push_back(position);
      }
    } else {
      Eigen::Vector2i position = position2pixelLocation(Eigen::Vector3d(path[j][0], path[j][1], 0.0));
      if (discrete_path.back() != position) discrete_path.push_back(position);
    }
  }
  for (Eigen::Vector2i point:discrete_path) path_map(AM(point)) = 1;
  return path_map;
}

Eigen::ArrayXXi MapServerPlanExp::getCurrentPathMap() {
  if (p_compute_path_) {
    std::lock_guard<std::mutex> guard(current_path_estimate_mutex_);
    return current_path_map_estimate_;
  } else {
    ROS_ERROR("[MapServerPlanExp] Planner path requested but it was not computed.");
    return Eigen::ArrayXXi();
  }
}

int MapServerPlanExp::getCurrentPathMapStatusAtLocation(Eigen::Vector2i point) {
  if (p_compute_path_) {
    std::lock_guard<std::mutex> guard(current_path_estimate_mutex_);
    return current_path_map_estimate_(AM(point));
  } else {
    ROS_ERROR("[MapServerPlanExp] Planner path requested but it was not computed.");
    return 0;
  }
}

inline uint8_t status2ROS(planexp_base_planners::ResultStatus status) {
  switch(status) {
    case planexp_base_planners::VALID:
      return planexp_msgs::PlannerStatus::STATUS_VALID;
      break;
    case planexp_base_planners::TERMINAL_VALID:
      return planexp_msgs::PlannerStatus::STATUS_TERMINAL_VALID;
      break;
    case planexp_base_planners::INVALID:
      return planexp_msgs::PlannerStatus::STATUS_INVALID;
      break;
    case planexp_base_planners::TERMINAL_INVALID:
      return planexp_msgs::PlannerStatus::STATUS_TERMINAL_INVALID;
      break;
    default:
      return planexp_msgs::PlannerStatus::STATUS_UNKNOWN;
  }
}

inline uint8_t type2ROS(planexp_base_planners::PlannerType type) {
  switch(type) {
    case planexp_base_planners::PRM_STAR_PLANNER:
      return planexp_msgs::PlannerStatus::TYPE_PRM_STAR;
      break;
    case planexp_base_planners::RRT_STAR_PLANNER:
      return planexp_msgs::PlannerStatus::TYPE_RRT_STAR;
      break;
    case planexp_base_planners::UNKNOWN_PLANNER:
    default:
      return planexp_msgs::PlannerStatus::TYPE_UNKNOWN;
  }
}

void MapServerPlanExp::publishPlannerStatus(planexp_base_planners::ResultStatus status) {
  planexp_msgs::PlannerStatus msg;
  msg.header.stamp = ::ros::Time::now();
  msg.header.seq = ++planner_status_seq_id_;
  msg.header.frame_id = "";
  msg.status = status2ROS(status);
  msg.type = type2ROS(planner_type_);
  planner_status_pub_.publish(msg);
}
