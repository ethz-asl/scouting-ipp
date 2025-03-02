//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//
#include "mav_simulator/mav_simulator.h"

MavSimulator::MavSimulator(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) :
                          nh_(nh), nh_private_(nh_private) {
  ROS_DEBUG("[MavSimulatorPlanExp] Initializing!");

  std::string map_source;
  nh_private_.param("map_source", map_source, std::string("image"));
  nh_private_.param("max_step_size", max_step_size_, max_step_size_);
  nh_private_.param("provide_init_map", provide_init_map_, provide_init_map_);
  nh_private_.param("provide_full_init_depth", provide_full_init_depth_, provide_full_init_depth_);
  nh_private_.param("use_timing", use_timing_, use_timing_);
  nh_private_.param("mav_speed", mav_speed_, mav_speed_);
  nh_private_.param("frame_rate", frame_rate_, frame_rate_);
  bool ignore_mav_obstacles;
  nh_private_.param("ignore_mav_obstacles", ignore_mav_obstacles, false);

  nh_private_.param("map_config_path", map_config_path_, map_config_path_);
  if (!map_config_path_.empty()) parse_map_config_file(map_config_path_);

  if (map_config_valid_) {
    start_position_ = Eigen::Vector3d(map_config_.start_location[0], map_config_.start_location[1], 0.0);
    goal_position_ = Eigen::Vector3d(map_config_.goal_location[0], map_config_.goal_location[1], 0.0);
  } else {
    start_position_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    goal_position_ = Eigen::Vector3d(-1.0, -1.0, -1.0);
  }

  std::string cost_formulation;
  nh_private_.param("cost_formulation", cost_formulation, std::string("time"));
  if (cost_formulation == "map") {
    cost_formulation_ = MapCost;
  }
  else if (cost_formulation == "time") {
    cost_formulation_ = TimeCost;
  }
  else if (cost_formulation == "distance") {
    cost_formulation_ = DistanceCost;
  } else {
    ROS_ERROR("[MavSimulatorPlanExp] Unknown cost formulation: %s", cost_formulation.c_str());
  }

#if COLLISION_CHECK
  if (map_config_valid_ && !nh_private_.hasParam("obstacle_label")) {
    obstacle_ids_ = map_config_.obstacle_ids_mav;
    if (ignore_mav_obstacles) obstacle_ids_.clear();
  } else {
    nh_private_.param("obstacle_label", obstacle_id_, obstacle_id_);
    obstacle_ids_.push_back(obstacle_id_);
  }
  if (map_config_valid_ && !nh_private_.hasParam("obstacle_label_rov")) {
    rov_obstacle_ids_ = map_config_.obstacle_ids_rov;
  } else {
    int obstacle_id_rov = 0;
    nh_private_.param("obstacle_label_rov", obstacle_id_rov, obstacle_id_rov);
    rov_obstacle_ids_.push_back(obstacle_id_rov);
  }
#endif
  if (map_config_valid_ && (!nh_private_.hasParam("map_width") || !nh_private_.hasParam("map_height"))) {
    map_height_ = map_config_.height;
    map_width_ = map_config_.width;
  } else {
    nh_private_.param("map_width", map_width_, map_width_);
    nh_private_.param("map_height", map_height_, map_height_);
  }

  if (map_config_valid_ && (!nh_private_.hasParam("num_seg_classes"))) {
    num_seg_classes_ = map_config_.n_data_classes;
  } else {
    nh_private_.param("num_seg_classes", num_seg_classes_, num_seg_classes_);
  }

  if (map_source == "image") {
    nh_private_.param("map_path", map_path_, map_path_);
    nh_private_.param("height_map_path", height_map_path_, height_map_path_);
#if SIMULATE_ROV
    nh_private_.param("rov_map_path", rov_map_path_, rov_map_path_);
#endif
  }
  else if (map_source == "oaisys") {
#if OAISYS_MAP
    ROS_DEBUG("[MavSimulatorPlanExp] Using Oaisys as map supplier!");
#else
    ROS_ERROR("[MavSimulatorPlanExp] Try to use Oaisys as map supplier but the necessary build flag was not set!");
#endif
  }
  else {
    ROS_ERROR("[MavSimulatorPlanExp] Unknown map source: %s", map_source.c_str());
  }

  odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("odometry", 1000);
  image_pub_ = nh_.advertise<planexp_msgs::MapData>("map_data", 1000);
  depth_image_pub_ = nh_.advertise<planexp_msgs::MapData>("map_data_depth", 1000);
  goal_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal_pose", 1000);
#if SIMULATE_ROV
  rov_image_pub_ = nh_.advertise<planexp_msgs::MapData>("rov_map_data", 1000);
#endif
#if COLLISION_CHECK
  collision_pub_ = nh_.advertise<std_msgs::String>("collision", 1000);
#endif

  trajectory_sub_ = nh_.subscribe("command/trajectory", 1, &MavSimulator::trajectoryCallback, this );

  path_cost_srv_ = nh_.advertiseService("path_cost", &MavSimulator::pathCostCallback, this);
  path_costs_.clear();

#if EXPLORABLE_AREA
  explorable_area_srv_ = nh_.advertiseService("explorable_area", &MavSimulator::explorableAreaCallback, this);
  explored_area_srv_ = nh_.advertiseService("explored_area", &MavSimulator::exploredAreaCallback, this);
#endif

  //TODO: Initialize Parameters from config file here.

#if OAISYS_MAP
  oaisys_request_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/oaisys_request", 1000);
  oaisys_segmenation_sub_ = nh_.subscribe("/oaisys_sementation_output", 1000, &MavSimulator::oaisysSegmentationCallback, this);
#else
  loadMap(map_, map_path_);
  loadHeightMap(height_map_, height_map_path_);
  if (map_config_valid_) {
    if (map_.cols() != map_config_.resolution[0] || map_.rows() != map_config_.resolution[1]) {
      ROS_ERROR("[MavSimulatorPlanExp] Mismatch between actual image size and expected image size from map config file: %s", map_config_path_.c_str());
    }
    if (height_map_.cols() != map_config_.resolution[0] || height_map_.rows() != map_config_.resolution[1]) {
      ROS_ERROR("[MavSimulatorPlanExp] Mismatch between actual image size and expected image size from map config file: %s", map_config_path_.c_str());
    }
  }
#endif

#if SIMULATE_ROV
  loadMap(rov_map_, rov_map_path_);
  if (rov_map_.size() != map_.size()) {
    ROS_ERROR("[MavSimulatorPlanExp] The provided maps for the rover and the drone do not match in size!");
  }
#endif

  goal_location_requested_pub_ = nh_.advertise<std_msgs::Empty>("goal_location_requested", 1000);

  m_per_pixel_ = map_width_ / map_.cols();
  if(abs(m_per_pixel_ - (map_height_ / map_.rows())) > 0.01 ) {
    ROS_WARN("[MavSimulatorPlanExp] The format of the provided map and the defined map size does not match! The map height has been changed: %.4f -> %.4f", map_height_, map_.rows() * m_per_pixel_);
    map_height_ = map_.rows() * m_per_pixel_;
  }

  double cam_view_width, cam_view_height;
  nh_private_.param("cam_view_width", cam_view_width, 4.0);
  nh_private_.param("cam_view_height", cam_view_height, 4.0);
  fov_x_=std::round(cam_view_width / m_per_pixel_);
  fov_y_=std::round(cam_view_height / m_per_pixel_);

  if (provide_init_map_) {
    double init_area_width, init_area_height;
    nh_private_.param("init_area_with", init_area_width, 5.0*4);
    nh_private_.param("init_area_height", init_area_height, 5.0*4);
    init_fov_x_=std::round(init_area_width / m_per_pixel_);
    init_fov_y_=std::round(init_area_height / m_per_pixel_);
  }

  image_ = Eigen::ArrayXXi::Zero(fov_y_, fov_x_);
  depth_image_ = Eigen::ArrayXXd::Zero(fov_y_, fov_x_);
#if SIMULATE_ROV
  rov_image_ = Eigen::ArrayXXi::Zero(fov_y_, fov_x_);
  ROS_DEBUG("[MavSimulatorPlanExp] Using ROV + MAV mode!");
#endif
#if EXPLORABLE_AREA
  explored_space_map_ = Eigen::ArrayXXi::Zero(map_.rows(), map_.cols());
  explorability_map_ = Eigen::ArrayXXi::Zero(map_.rows(), map_.cols());
  if (map_config_valid_ && rov_obstacle_ids_.empty()) {
    explorable_pixels_ = map_.rows() * map_.cols();
    explorable_area_ = map_config_.width * map_config_.height;
    explorability_map_.setOnes();
    ROS_DEBUG("[MavSimulatorPlanExp] Explorable Area (overwrite for empty obstacle ids list): %.4f", explorable_area_);
  } else {
    auto t0 = ros::Time::now();
    computeMaxExplorableArea(position2pixelLocation(start_position_));
    auto t1 = ros::Time::now();
    explorable_area_ = explorable_pixels_ * m_per_pixel_ * m_per_pixel_;
    ROS_DEBUG("[MavSimulatorPlanExp] Explorable Area (computed): %.4f (%.4fs)", explorable_area_, (t1-t0).toSec());
  }
#endif

  if (map_config_valid_) {
    start_location_ = position2pixelLocation(Eigen::Vector3d(map_config_.start_location[0], map_config_.start_location[1], 0.0));
    goal_location_ = position2pixelLocation(Eigen::Vector3d(map_config_.goal_location[0], map_config_.goal_location[1], 0.0));
  } else {
    start_location_ = Eigen::Vector2i(0, 0);
    goal_location_ = Eigen::Vector2i(0, 0);
  }

  set_initial_position_srv_ = nh_.advertiseService("set_initial_position", &MavSimulator::setInitialPositionCallback, this);

  earliest_next_pub_time_ = std::chrono::steady_clock::now();

  nh_private_.param("dump_trajectory", dump_trajectory_, dump_trajectory_);
  if (dump_trajectory_) setup_trajectory_file();

  ROS_DEBUG("[MavSimulatorPlanExp] Finished initialisation!");
}

bool MavSimulator::check_map_config(map_config_t map_config, std::string file_path, bool verbose) {
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

  if (!config_valid && verbose) std::cout << "[MavSimulatorPlanExp] Invalid configuration from map config file: " << file_path << std::endl;
  return config_valid;
}

void MavSimulator::parse_map_config_file(std::string file_path) {
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
    std::cout << "[MavSimulatorPlanExp] Unable to read or parse map config file: " << file_path << std::endl;
    map_config_ = map_config_t();
    map_config_valid_ = false;
    return;
  }
  if (!check_map_config(map_config, file_path)) {
    map_config_ = map_config_t();
    map_config_valid_ = false;
    return;
  }
  map_config_ = map_config;
  map_config_valid_ = true;
}

void MavSimulator::trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory& msg) {
  mav_msgs::eigenTrajectoryPointVectorFromMsg(msg, &trajectory_);
#if USE_TEST_MODE
  ROS_DEBUG("[MavSimulatorPlanExp] Called callback!");
  mav_msgs::EigenTrajectoryPoint start, goal;
  start.position_W.x() = 0.0;
  start.position_W.y() = 0.0;
  goal.position_W.x() = map_width_-m_per_pixel_;
  goal.position_W.y() = map_height_-m_per_pixel_;
  trajectory_.push_back(start);
  trajectory_.push_back(goal);
#endif
  if (!trajectory_.empty()) {
    //std::cout << "[MavSimulatorPlanExp] Recieved odometry!" << std::endl;
    /*std::cout << "front:" << trajectory_.front().position_W << std::endl;
    std::cout << "front:" << trajectory_.front().orientation_W_B.matrix() << std::endl;
    std::cout << "back:" << trajectory_.back().position_W << std::endl;
    std::cout << "back:" << trajectory_.back().orientation_W_B.matrix() << std::endl;
    std::cout << std::endl;*/
    computeMeasurements(trajectory_.back());
    publishGoal(trajectory_.back());
  }
}

void MavSimulator::addOdometryToQueue(const mav_msgs::EigenTrajectoryPoint& target) {
  nav_msgs::Odometry msg;
  mav_msgs::EigenOdometry odometry;
  odometry.position_W = target.position_W;
  odometry.orientation_W_B = target.orientation_W_B;
  odometry.angular_velocity_B = target.angular_velocity_W;
  odometry.velocity_B = target.velocity_W;
  odometry.timestamp_ns = target.timestamp_ns;
  mav_msgs::msgOdometryFromEigen(odometry, &msg);
  msg.header.seq = (int)(++seq_id_odometry_);
  msg.header.frame_id = "world";
  odometry_queue_.push(msg);
  last_odometry_ = odometry;
}

void MavSimulator::publishOdometry() {
  nav_msgs::Odometry msg = odometry_queue_.front();
  msg.header.stamp = ros::Time::now();

  odometry_pub_.publish(msg);
  odometry_queue_.pop();
  odometry_time_queue_.pop();
}

void MavSimulator::publishGoal(const mav_msgs::EigenTrajectoryPoint &target) {
  geometry_msgs::PoseStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.header.seq = ++seq_id_goal_pose_;
  msg.header.frame_id = "world";

  msg.pose.position.x = target.position_W.x();
  msg.pose.position.y = target.position_W.y();
  msg.pose.position.z = target.position_W.z();
  msg.pose.orientation.x = target.orientation_W_B.x();
  msg.pose.orientation.y = target.orientation_W_B.y();
  msg.pose.orientation.z = target.orientation_W_B.z();
  msg.pose.orientation.w = target.orientation_W_B.w();


  goal_pose_pub_.publish(msg);
}

void MavSimulator::mainLoop() {
  while (ros::ok()) {
    if (!time_queue_.empty()) {
      while (std::chrono::steady_clock::now() >= time_queue_.front()) {
        publishImage();
        publishDepthImage();
        if (time_queue_.empty()) break;
      }
    }
    if (!odometry_time_queue_.empty()) {
      while (std::chrono::steady_clock::now() >= odometry_time_queue_.front()) {
        publishOdometry();
        if (odometry_time_queue_.empty()) break;
      }
    }
    if (!goal_request_published_ && goal_requested_ && std::chrono::steady_clock::now() >= first_goal_request_time_) {
      goal_location_requested_pub_.publish(std_msgs::Empty());
      goal_request_published_ = true;
    }
    ros::spinOnce();
  }
}

void MavSimulator::computeMeasurements(const mav_msgs::EigenTrajectoryPoint& target) {
  if (trajectory_.size() >= 2) {
    auto start = std::chrono::steady_clock::now();
    for (int i = 1; i < trajectory_.size(); ++i) {
      mav_msgs::EigenTrajectoryPoint start_point = trajectory_.at(i - 1);
      mav_msgs::EigenTrajectoryPoint next_point = trajectory_.at(i);

      double segment_cost = computeCost(start_point.position_W, next_point.position_W);

#if USE_TEST_MODE
      ROS_DEBUG("[MavSimulatorPlanExp] Max distance to travel: %.3lf", (trajectory_.back().position_W-trajectory_.front().position_W).norm());
#endif

      double distance = (next_point.position_W - start_point.position_W).norm();

      Eigen::Vector3d direction(0.0, 0.0, 0.0);
      if (distance > 0.001) direction = (next_point.position_W - start_point.position_W) / distance;

      Eigen::Vector3d current_position = start_point.position_W;

      double distance_per_frame = mav_speed_ / frame_rate_;

#if USE_TEST_MODE
      ROS_DEBUG("[MavSimulatorPlanExp] Distance to travel: %.3lf", distance + distance_left_over_);
#endif
      double accumulated_sub_segment_cost = 0.0;
      while ((distance + distance_left_over_) >= distance_per_frame || seq_id_image_ == 0) {
        if (earliest_next_pub_time_ > start) {
          start = earliest_next_pub_time_;
        } else {
          earliest_next_pub_time_=start;
        }
        Eigen::Vector3d new_position = current_position + direction * (distance_per_frame - distance_left_over_);

        accumulated_sub_segment_cost += computeCost(current_position, new_position);

        if (seq_id_image_ == 0) {
          getMapArea(new_position, true);
        } else {
          getMapArea(new_position);
        }

        double time_to_wait = 0.0;
        if (use_timing_) {
          time_to_wait = (distance_per_frame - distance_left_over_) / mav_speed_;
        }

        earliest_next_pub_time_ += std::chrono::nanoseconds((int)(time_to_wait*1000000000));

        path_costs_.push_back(accumulated_path_cost_+accumulated_sub_segment_cost);
        time_queue_.push(start + std::chrono::nanoseconds((int)(time_to_wait*1000000000)));

        if (dump_trajectory_)
          dump_trajectory_to_file(start + std::chrono::nanoseconds((int)(time_to_wait*1000000000)), new_position);

        planexp_msgs::MapData msg, depth_msg;
        addImageToQueue(msg, new_position);
        addDepthImageToQueue(depth_msg, new_position);
        double explored_area;
        computeExploredArea(explored_area);
        explored_areas_.push_back(explored_area);

        distance -= (distance_per_frame - distance_left_over_);
        current_position = new_position;
        distance_left_over_ = 0.0;
        if (!goal_requested_) {
          if (explored_space_map_(AM(goal_location_)) == 1) {
            goal_requested_ = true;
            first_goal_request_time_ = start + std::chrono::nanoseconds((int)(time_to_wait*1000000000));
          }
        }
      }
      if (use_timing_) {
        earliest_next_pub_time_ += std::chrono::nanoseconds((int)(distance*1000000000/mav_speed_));
      }
      distance_left_over_ += distance;
      accumulated_path_cost_ += segment_cost;
    }
    odometry_time_queue_.push(earliest_next_pub_time_);
    addOdometryToQueue(target);
  } else {
    ROS_WARN("[MavSimulatorPlanExp] Recieved trajectory with %d entries!", (int)trajectory_.size());
  }
}

void MavSimulator::getMapArea(const Eigen::Vector3d& position, const bool init, const bool mark_explored) {
  Eigen::Vector2i pixel_location = position2pixelLocation(position);
  Eigen::Vector2i bottom_right_corner;

  Eigen::Vector2i fov(fov_x_, fov_y_);
  if (init) {
    fov.x() = init_fov_x_;
    fov.y() = init_fov_y_;
  }

  image_.setZero(fov.y(), fov.x());
  depth_image_.setZero(fov.y(), fov.x());
#if SIMULATE_ROV
  rov_image_.setZero(fov_y, fov_x);
#endif

  //change from center to corner
  pixel_location = pixel_location - (fov/2);
  bottom_right_corner = pixel_location + fov;

  Eigen::Vector2i new_pixel_location = enforceMapConstraints(pixel_location);
  Eigen::Vector2i new_bottom_right_corner = enforceMapConstraints(bottom_right_corner);
  Eigen::Vector2i reduced_fov = new_bottom_right_corner - new_pixel_location;
  Eigen::Vector2i offset = new_pixel_location - pixel_location;

  if (reduced_fov.x() > 0 && reduced_fov.y() > 0 ) {
    image_.block(offset.y(), offset.x(), reduced_fov.y(), reduced_fov.x()) =
        map_.block(new_pixel_location.y(), new_pixel_location.x(), reduced_fov.y(), reduced_fov.x());
    if (init && provide_full_init_depth_) {
      depth_image_ = height2depth(height_map_, position);
    } else {
      depth_image_.block(offset.y(), offset.x(), reduced_fov.y(), reduced_fov.x()) =
        height2depth(height_map_.block(new_pixel_location.y(), new_pixel_location.x(), reduced_fov.y(), reduced_fov.x()), position);
    }
    
    if (mark_explored) explored_space_map_.block(new_pixel_location.y(), new_pixel_location.x(), reduced_fov.y(), reduced_fov.x()).setOnes();
#if SIMULATE_ROV
    rov_image_.block(offset.y(), offset.x(), reduced_fov.y(), reduced_fov.x()) =
        rov_map_.block(new_pixel_location.y(), new_pixel_location.x(), reduced_fov.y(), reduced_fov.x());
#endif
  }
}

void MavSimulator::addImageToQueue(planexp_msgs::MapData& msg, const Eigen::Vector3d& position) {

  msg.header.seq = ++seq_id_image_;

  std_msgs::Int32MultiArray image_msg;
  matrixEigenToIntMsg(image_.matrix(), image_msg);
  msg.data = image_msg;

  geometry_msgs::Pose pose_msg;
  pose_msg.position.x = position.x();
  pose_msg.position.y = position.y();
  pose_msg.position.z = position.z();
  msg.pose = pose_msg;

  msg_queue_.push(msg);
#if USE_TEST_MODE
  ROS_DEBUG("[MavSimulatorPlanExp] Published MAV image!");
#endif

#if SIMULATE_ROV
  matrixEigenToIntMsg(rov_image_.matrix(), image_msg);
  msg.data = image_msg;
  rov_msg_queue_.push(msg);
#if USE_TEST_MODE
  ROS_DEBUG("[MavSimulatorPlanExp] Published ROV image!");
#endif
#endif

}

void MavSimulator::addDepthImageToQueue(planexp_msgs::MapData& msg, const Eigen::Vector3d& position) {

  msg.header.seq = ++seq_id_depth_image_;

  std_msgs::Int32MultiArray depth_image_msg;
  matrixEigenToIntMsg((depth_image_ * 100.0).cast<int>().matrix(), depth_image_msg);
  msg.data = depth_image_msg;

  geometry_msgs::Pose pose_msg;
  if (msg.header.seq <= 1 && provide_full_init_depth_) {
    ROS_ERROR("Providing initial map at location: x:%lf, y: %lf, z: %lf", map_width_ / 2.0, map_height_/ 2.0, position.z());
    pose_msg.position.x = map_width_ / 2.0;
    pose_msg.position.y = map_height_ / 2.0;
    pose_msg.position.z = position.z();
  } else {
    pose_msg.position.x = position.x();
    pose_msg.position.y = position.y();
    pose_msg.position.z = position.z();
  }
  msg.pose = pose_msg;

  msg_depth_queue_.push(msg);
#if USE_TEST_MODE
  ROS_DEBUG("[MavSimulatorPlanExp] Published MAV depth image!");
#endif
}

void MavSimulator::publishImage() {
  //std::cout << "[MavSimulatorPlanExp] Published image!" << std::endl;
  planexp_msgs::MapData msg;
  msg = msg_queue_.front();
  msg.header.stamp = ros::Time::now();

  image_pub_.publish(msg);
#if USE_TEST_MODE
  ROS_DEBUG("[MavSimulatorPlanExp] Published MAV image!");
#endif

#if SIMULATE_ROV
  msg = rov_msg_queue_.front();
  msg.header.stamp = ros::Time::now();
  rov_image_pub_.publish(msg);
#if USE_TEST_MODE
  ROS_DEBUG("[MavSimulatorPlanExp] Published ROV image!");
#endif
#endif
  msg_queue_.pop();
  time_queue_.pop();
#if SIMULATE_ROV
  rov_msg_queue_.pop();
#endif

}

void MavSimulator::publishDepthImage() {
  if (msg_depth_queue_.empty()) {
    ROS_WARN("[MavSimulatorPlanExp] Tried to publish depth image, but the queue was empty!");
    return;
  }
  planexp_msgs::MapData msg;
  msg = msg_depth_queue_.front();
  msg.header.stamp = ros::Time::now();

  depth_image_pub_.publish(msg);
#if USE_TEST_MODE
  ROS_DEBUG("[MavSimulatorPlanExp] Published MAV depth image!");
#endif

  msg_depth_queue_.pop();
}

Eigen::Vector2i MavSimulator::position2pixelLocation(Eigen::Vector3d position) {
  Eigen::Vector2i pixel_location = Eigen::Vector2i::Ones();
  pixel_location.x() = (int)(position.x() * map_.cols()/ map_width_);
  pixel_location.y() = (int)(position.y() * map_.rows()/ map_height_);
  return pixel_location;
}

void MavSimulator::loadMap(Eigen::ArrayXXi& target, const std::string path, const bool print) {
  cv::Mat cv_image = cv::imread(path, cv::IMREAD_COLOR);
  target = image2map(cv_image);
  if (print) {
    ROS_DEBUG("Loaded map from location: %s", path.c_str());
    ROS_DEBUG("Map width (cols): %d", (int)target.cols());
    ROS_DEBUG("Map height (rows): %d", (int)target.rows());
  }
}

void MavSimulator::loadHeightMap(Eigen::ArrayXXd& target, const std::string path, const bool print) {
  cv::redirectError(ignoreError);
  try {
    ROS_INFO("Try loading height map from location: %s", path.c_str());
    cv::Mat cv_image = cv::imread(path, cv::IMREAD_ANYDEPTH);
    target = image2heightMap(cv_image);
    if (print) {
      ROS_DEBUG("Loaded height map from location: %s", path.c_str());
      ROS_DEBUG("Map width (cols): %d", (int)target.cols());
      ROS_DEBUG("Map height (rows): %d", (int)target.rows());
    }
  } catch (...) {
    ROS_WARN("Height Map could not be loaded. Defaulting to -0.1m height.");
    target = Eigen::ArrayXXd::Ones(map_.rows(), map_.cols()) * -0.1;
  }
  cv::redirectError(nullptr);
}

void MavSimulator::computeMaxExplorableArea(Eigen::Vector2i start_location) {
  Eigen::ArrayXXi already_checked;
  already_checked.setZero(map_.rows(), map_.cols());
  Eigen::ArrayXXi explorability_map;
  explorability_map.setZero(map_.rows(), map_.cols());
  Eigen::Vector2i current_location, pixel_location(0,0), bottom_right_corner;
  current_location = start_location;
  std::queue<Eigen::Vector2i> position_queue;
  position_queue.push(current_location);
  already_checked(AM(current_location)) = 1;
  Eigen::Vector2i fov(fov_x_, fov_y_);
  int position_counter = 0;
  while (!position_queue.empty()) {
    current_location = position_queue.front();
    position_queue.pop();
    ++position_counter;

#if EXPLORABLE_FREE_SPACE == 0
    pixel_location = current_location - (fov/2);
    bottom_right_corner = pixel_location + fov;

    Eigen::Vector2i new_pixel_location = enforceMapConstraints(pixel_location);
    Eigen::Vector2i new_bottom_right_corner = enforceMapConstraints(bottom_right_corner);
    Eigen::Vector2i reduced_fov = new_bottom_right_corner - new_pixel_location;
    Eigen::Vector2i offset = new_pixel_location - pixel_location;
    explorability_map.block(new_pixel_location.y(), new_pixel_location.x(), reduced_fov.y(), reduced_fov.x()).setOnes();
#endif

    for (int i = -1; i <= 1; ++i) {
      for (int j = -1; j <= 1; ++j) {
        Eigen::Vector2i loc = current_location + Eigen::Vector2i(i, j);
        if (checkMapConstraints(loc)) {
          if (!idIsRovObstacle(map_(AM(loc))) && already_checked(AM(loc)) != 1) {
            position_queue.push(loc);
            already_checked(AM(loc)) = 1;
          }
        }
      }
    }
  }
#if EXPLORABLE_FREE_SPACE
  explorable_pixels_ = already_checked.count();
  explorability_map_ = already_checked;
#else
  explorable_pixels_ = explorability_map.count();
  explorability_map_ = explorability_map;
#endif
}

Eigen::ArrayXXi MavSimulator::image2map(const cv::Mat& image) {
  /*
   * Extracts the labels out of an BGR image using the Oaisys conversion convention.
   */
  cv::Mat seg_image;
  cv::Mat seg_channels[3];
  int num_classes = num_seg_classes_;
  double step = num_classes/255.0;
  int c2 = std::pow(num_classes,2);

  seg_image = image;

  seg_image.convertTo(seg_image, CV_32FC3);
  seg_image.convertTo(seg_image, CV_32FC3, step, 0 );
  roundImg(seg_image);
  seg_image.convertTo(seg_image, CV_32SC3);
  cv::split(seg_image, seg_channels);
  cv::Mat labels = seg_channels[0] + seg_channels[1] * num_classes + seg_channels[2] * c2;

  Eigen::MatrixXi map;
  cv::cv2eigen(labels, map);

  return map.array();
}

Eigen::ArrayXXd MavSimulator::image2heightMap(const cv::Mat& image) {

  Eigen::MatrixXd map;
  cv::cv2eigen(image, map);

  return map.array();
}

Eigen::Vector2i MavSimulator::enforceMapConstraints(Eigen::Vector2i& pixel_location) {
  Eigen::Vector2i new_pixel_location = pixel_location;
  //enforce lower bounds
  new_pixel_location.x() = (new_pixel_location.x() >= 0) ? new_pixel_location.x() : 0;
  new_pixel_location.y() = (new_pixel_location.y() >= 0) ? new_pixel_location.y() : 0;

  //enforce upper bounds
  new_pixel_location.x() = (new_pixel_location.x() < map_.cols()) ? new_pixel_location.x() : map_.cols();
  new_pixel_location.y() = (new_pixel_location.y() < map_.rows()) ? new_pixel_location.y() : map_.rows();

  return new_pixel_location;

}

bool MavSimulator::checkMapConstraints(Eigen::Vector2i& pixel_location) {
  if ((pixel_location.x() < map_.cols() && pixel_location.x() >= 0) && (pixel_location.y() < map_.rows() && pixel_location.y() >= 0)) return true;
  return false;
}

bool MavSimulator::pathCostCallback(planexp_msgs::PathCost::Request& request,
                                    planexp_msgs::PathCost::Response& response) {
  int map_id = request.map_id;

  std::string cost_type;
  switch (cost_formulation_) {
    case MapCost:
      cost_type = "map";
      break;
    case DistanceCost:
      cost_type = "distance";
      break;
    case TimeCost:
      cost_type = "time";
      break;
    default:
      ROS_ERROR("[MavSimulatorPlanExp] Unknown cost formulation: %d", cost_formulation_);
      break;
  }

  if ((int)path_costs_.size() < map_id || path_costs_.empty()) {
    return false;
  } else {
    if (map_id < 0) map_id = path_costs_.size() - 1;
    response.cost = path_costs_.at(map_id);
    response.cost_type = cost_type.c_str();
    return true;
  }
}

bool MavSimulator::setInitialPositionCallback(std_srvs::Trigger::Request& request,
                                              std_srvs::Trigger::Response& response) {
  char text[100];
  Eigen::Vector2i start_location = position2pixelLocation(start_position_);
  if (idIsObstacle(map_(AM(start_location)))) {
    sprintf(text, "Start position is not in free space! (x: %.03f, y: %.03f, z: %.03f)",
            start_position_.x(), start_position_.y(), start_position_.z());
    response.success = false;
    response.message = text;
    return true;
  }
  if(seq_id_odometry_ == 0) {
    nav_msgs::Odometry msg;
    mav_msgs::EigenOdometry odometry;
    odometry.position_W = start_position_;
    odometry.orientation_W_B = Eigen::Quaternion<double>(1.0,0.0,0.0,0.0);
    odometry.angular_velocity_B = Eigen::Vector3d(0.0, 0.0, 0.0);
    odometry.velocity_B = Eigen::Vector3d(0.0, 0.0, 0.0);
    odometry.timestamp_ns = ros::Time::now().toNSec();
    mav_msgs::msgOdometryFromEigen(odometry, &msg);
    msg.header.seq = (int)(++seq_id_odometry_);
    msg.header.frame_id = "world";
    last_odometry_ = odometry;

    odometry_pub_.publish(msg);
    sprintf(text, "Start position set to: x: %.03f, y: %.03f, z: %.03f", odometry.position_W.x(),
            odometry.position_W.y(), odometry.position_W.z());
    response.success = true;
    response.message = text;
  } else {
    sprintf(text, "Start position not set! (seq_id=%d)", seq_id_odometry_);
    response.success = false;
    response.message = text;
  }
  return true;
}

double MavSimulator::computeCost(Eigen::Vector3d start, Eigen::Vector3d goal) {

  double  cost = 0.0;
  double distance = (goal - start).norm();
  int n_elements = ceil(distance/max_step_size_);
  if (n_elements > 0) {
    double step_distance = distance / n_elements;
    Eigen::Vector3d step = (goal - start) / n_elements;
    Eigen::Vector3d current_position = start;

    for (int i = 0; i<n_elements; ++i) {
      Eigen::Vector3d current_target = current_position + step;
      Eigen::Vector2i segment_start = position2pixelLocation(current_position);
      Eigen::Vector2i segment_end = position2pixelLocation(current_target);
      double average_segment_cost = (map_(AM(segment_start)) + map_(AM(segment_end)))/2.0;
#if COLLISION_CHECK
      if (idIsObstacle(map_(AM(segment_end)))) publishCollision(current_target); //TODO: Maybe implement a timed version of the collision check.
#endif
      switch (cost_formulation_) {
        case MapCost:
          cost += average_segment_cost * step_distance;
          break;
        case DistanceCost:
          cost += step_distance;
          break;
        case TimeCost:
          cost += (step_distance / mav_speed_);
          break;
        default:
          ROS_ERROR("[MavSimulatorPlanExp] Unknown cost formulation: %d", cost_formulation_);
          break;
      }
      current_position = current_target;
    }
  }
  return cost;
}

#if OAISYS_MAP
void MavSimulator::oaisysSegmentationCallback(const sensor_msgs::Image& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  image_ = image2map(cv_ptr->image); //TODO: Continue implementation of oaisys version here.
}
#endif

void MavSimulator::roundImg(cv::Mat& image) {
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

#if COLLISION_CHECK
void MavSimulator::publishCollision(Eigen::Vector3d position) {
  std_msgs::String msg;
  char text[100];
  sprintf(text, "Collision at: x: %.03f, y: %.03f, z: %.03f", position.x(), position.y(), position.z());
  msg.data = (std::string) text;

  collision_pub_.publish(msg);
}
#endif

#if EXPLORABLE_AREA
bool MavSimulator::explorableAreaCallback(planexp_msgs::ExplorableArea::Request& request,
                                          planexp_msgs::ExplorableArea::Response& response) {
  response.explorable_area = explorable_area_;
  return true;
}

bool MavSimulator::exploredAreaCallback(planexp_msgs::ExploredArea::Request& request,
                                             planexp_msgs::ExploredArea::Response& response) {
  int map_id = request.map_id;
  if ((int)explored_areas_.size() < map_id || explored_areas_.empty()) {
    return false;
  } else {
    if (map_id < 0) map_id = explored_areas_.size() - 1;
    response.explored_area = explored_areas_.at(map_id);
    return true;
  }
}
#endif

bool MavSimulator::computeExploredArea(double& explored_area) {
  int n_exp_pixel = ((explored_space_map_ + explorability_map_) == 2).count();
  explored_area = n_exp_pixel * m_per_pixel_ * m_per_pixel_;
  return true;
}

void MavSimulator::setup_trajectory_file() {
  nh_private_.param("trajectory_file_path", trajectory_file_path_, trajectory_file_path_);
  std::ofstream outfile;
  outfile.open(trajectory_file_path_, std::ios::out | std::ios::trunc );
  outfile << "Time,PositionX,PositionY" << std::endl;
  outfile.close();
}

void MavSimulator::dump_trajectory_to_file(std::chrono::time_point<std::chrono::steady_clock> time, Eigen::Vector3d position) {
  std::ofstream outfile;
  outfile.open(trajectory_file_path_, std::ios::out | std::ios::app );
  outfile << time.time_since_epoch().count() << "," << position.x() << "," << position.y()<< std::endl;
  outfile.close();
}

Eigen::ArrayXXd MavSimulator::height2depth(Eigen::ArrayXXd height, Eigen::Vector3d position) {
  Eigen::ArrayXXd depth = Eigen::ArrayXXd::Zero(height.rows(), height.cols());

  depth = position.z() - height; //TODO: Check if this is correct

  return depth;
}