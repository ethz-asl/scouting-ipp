//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#ifndef MAV_SIMULATOR_MAV_SIMULATOR_H
#define MAV_SIMULATOR_MAV_SIMULATOR_H

#define USE_TEST_MODE 0
#define SIMULATE_ROV 0
#define OAISYS_MAP 0
#define COLLISION_CHECK 1
#define EXPLORABLE_AREA 1
#define EXPLORABLE_FREE_SPACE 1

#define AM(x_) (x_)[1], (x_)[0]

#include <chrono>
#include <queue>

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include "std_srvs/Trigger.h"
#include "nav_msgs/Odometry.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "mav_msgs/conversions.h"
#include "mav_simulator/conversions.h"
#include "mav_simulator/opencv_helpers.h"
#include "mav_simulator/types.h"
#include "planexp_msgs/MapData.h"
#include "planexp_msgs/PathCost.h"
#if EXPLORABLE_AREA
#include "planexp_msgs/ExplorableArea.h"
#include "planexp_msgs/ExploredArea.h"
#endif
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#if OAISYS_MAP
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PoseStamped.h"
#endif
#include <fstream>
#include <iostream>

class MavSimulator {
public:
  MavSimulator(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

  ~MavSimulator() = default;

  void mainLoop();

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher odometry_pub_;
  ros::Publisher image_pub_;
  ros::Publisher depth_image_pub_;
  ros::Publisher goal_pose_pub_;
#if SIMULATE_ROV
  ros::Publisher rov_image_pub_;
#endif
#if COLLISION_CHECK
  ros::Publisher collision_pub_;
#endif

  ros::Subscriber trajectory_sub_;

#if OAISYS_MAP
  ros::Publisher oaisys_request_pub_;
  ros::Subscriber oaisys_segmenation_sub_;

  int seq_id_oaisys{0};
#endif

  ros::ServiceServer path_cost_srv_;
#if EXPLORABLE_AREA
  ros::ServiceServer explorable_area_srv_;
  ros::ServiceServer explored_area_srv_;
#endif

  ros::ServiceServer set_initial_position_srv_;

  ros::Publisher goal_location_requested_pub_;

  int seq_id_odometry_{0};
  int seq_id_image_{0};
  int seq_id_depth_image_{0};
  int seq_id_goal_pose_{0};

  std::string map_config_path_="";
  map_config_t map_config_;
  bool map_config_valid_ = false;

  std::vector<double> path_costs_;
  double accumulated_path_cost_{0};
  double max_step_size_{0.05};

  mav_msgs::EigenTrajectoryPointVector trajectory_;

  mav_msgs::EigenOdometry last_odometry_;

  std::string map_path_{"/home/root/data/maps/map_212.png"};
  std::string height_map_path_{"/home/root/data/maps/height/map_200.exr"}; 
  Eigen::ArrayXXi map_;
  Eigen::ArrayXXd height_map_;

#if SIMULATE_ROV
  std::string rov_map_path_{"/home/root/data/maps/map_212.png"};
  Eigen::ArrayXXi rov_map_;
#endif

  double map_height_{30.0}, map_width_{40.0};

  Eigen::ArrayXXi explored_space_map_;
  Eigen::ArrayXXi explorability_map_;
  std::vector<double> explored_areas_;
  double explorable_area_{0.0};

  Eigen::Vector3d start_position_;
  Eigen::Vector3d goal_position_;
  Eigen::Vector2i start_location_;
  Eigen::Vector2i goal_location_;

  double distance_left_over_{0.0};

  double mav_speed_{1.0};
  double frame_rate_{2};

  Eigen::ArrayXXi image_;
  Eigen::ArrayXXd depth_image_;
#if SIMULATE_ROV
  Eigen::ArrayXXi rov_image_;
#endif

  double m_per_pixel_{0.0};

  int fov_y_, fov_x_;
  int explorable_pixels_{0};

  bool use_timing_{true};

  bool provide_init_map_{true};
  int init_fov_y_, init_fov_x_;
  bool provide_full_init_depth_{false};

  CostFunction_t cost_formulation_;

  int num_seg_classes_{15};
  int unknown_id_{3599};
#if COLLISION_CHECK
  int obstacle_id_{0};
  std::vector<int> obstacle_ids_;
  std::vector<int> rov_obstacle_ids_;
#endif

  std::chrono::time_point<std::chrono::steady_clock> earliest_next_pub_time_;

  std::queue<nav_msgs::Odometry> odometry_queue_;
  std::queue<std::chrono::time_point<std::chrono::steady_clock>> odometry_time_queue_;

  std::queue<std::chrono::time_point<std::chrono::steady_clock>> time_queue_;
  std::queue<planexp_msgs::MapData> msg_queue_;
  std::queue<planexp_msgs::MapData> msg_depth_queue_;
#if SIMULATE_ROV
  std::queue<planexp_msgs::MapData> rov_msg_queue_;
#endif
  bool goal_requested_{false};
  bool goal_request_published_{false};
  std::chrono::time_point<std::chrono::steady_clock> first_goal_request_time_;

  bool dump_trajectory_{false};
  std::string trajectory_file_path_ = "/tmp/trajectory_file_mav_simulator.csv";

  inline bool idIsObstacle(const int id) {return (std::find(obstacle_ids_.begin(), obstacle_ids_.end(), id) != obstacle_ids_.end());};
  inline bool idIsRovObstacle(const int id) {return (std::find(rov_obstacle_ids_.begin(), rov_obstacle_ids_.end(), id) != rov_obstacle_ids_.end());};
  void loadMap(Eigen::ArrayXXi& target, const std::string path, bool print = true);
  void loadHeightMap(Eigen::ArrayXXd& target, const std::string path, bool print = true);
  void computeMaxExplorableArea(Eigen::Vector2i start_location = Eigen::Vector2i(0,0));
  bool computeExploredArea(double& explored_area);
  void trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory& msg);
  bool pathCostCallback(planexp_msgs::PathCost::Request& request,
                        planexp_msgs::PathCost::Response& response);
  bool setInitialPositionCallback(std_srvs::Trigger::Request& request,
                                  std_srvs::Trigger::Response& response);
#if EXPLORABLE_AREA
  bool explorableAreaCallback(planexp_msgs::ExplorableArea::Request& request,
                              planexp_msgs::ExplorableArea::Response& response);
  bool exploredAreaCallback(planexp_msgs::ExploredArea::Request& request,
                            planexp_msgs::ExploredArea::Response& response);
#endif
  void addOdometryToQueue(const mav_msgs::EigenTrajectoryPoint& traget);
  void publishOdometry();
  void computeMeasurements(const mav_msgs::EigenTrajectoryPoint& target);
  void getMapArea(const Eigen::Vector3d& position, const bool init = false, const bool mark_explored = true);
  void publishImage();
  void publishDepthImage();
  void publishGoal(const mav_msgs::EigenTrajectoryPoint &target);
#if COLLISION_CHECK
  void publishCollision(Eigen::Vector3d position);
#endif
  void addImageToQueue(planexp_msgs::MapData& msg, const Eigen::Vector3d& position = Eigen::Vector3d::Zero());
  void addDepthImageToQueue(planexp_msgs::MapData& msg, const Eigen::Vector3d& position = Eigen::Vector3d::Zero());
  Eigen::Vector2i enforceMapConstraints(Eigen::Vector2i& pixel_location);
  bool checkMapConstraints(Eigen::Vector2i& pixel_location);
  void roundImg(cv::Mat& image);
  Eigen::Vector2i position2pixelLocation(Eigen::Vector3d position);
  Eigen::ArrayXXi image2map(const cv::Mat& image);
  Eigen::ArrayXXd image2heightMap(const cv::Mat& image);
  double computeCost(Eigen::Vector3d start, Eigen::Vector3d goal);
  bool check_map_config(map_config_t map_config, std::string file_path = "", bool verbose = true);
  void parse_map_config_file(std::string file_path);
#if OAISYS_MAP
  void oaisysSegmentationCallback(const sensor_msgs::Image& msg);
#endif

  void setup_trajectory_file();
  void dump_trajectory_to_file(std::chrono::time_point<std::chrono::steady_clock> time, Eigen::Vector3d position);
  Eigen::ArrayXXd height2depth(Eigen::ArrayXXd height, Eigen::Vector3d position);

};

#endif //MAV_SIMULATOR_MAV_SIMULATOR_H
