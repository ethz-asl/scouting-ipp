//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#ifndef ROS_WS_MAP_SERVER_PLANEXP_H
#define ROS_WS_MAP_SERVER_PLANEXP_H

#include <ros/ros.h>
#include <queue>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <thread>
#include <atomic>
#include <mutex>

#include "map_server_planexp/eigen_csv.h"
#include "map_server_planexp/grid_map_helpers.h"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "planexp_base_planners/planexp_base_planners.h"
#include "planexp_msgs/MapData.h"
#include "planexp_msgs/FilePath.h"
#include "planexp_msgs/PlannerStatus.h"
#include "nav_msgs/OccupancyGrid.h"
#include "grid_map_msgs/GridMap.h"
#include "nav_msgs/Path.h"

#define AM(x_) (x_)[1], (x_)[0]
#define MSR_OCCUPIED 0
#define MSR_FREE 1
#define MSR_UNKNOWN 2

#define RVIZ_VISUALIZATION 1
#define CREATE_RRT_PLANNER_MAP 1
#define MS_USE_MAP_CONFIG 1

#define MSR_REACHABILITY_UNKNOWN 0
#define MSR_REACHABLE 1

typedef Eigen::ArrayXXd CostMap_t;
typedef Eigen::ArrayXXi OccupancyMap_t;
typedef Eigen::ArrayXXd UncertaintyMap_t;
typedef Eigen::ArrayXXd HeightMap_t;
typedef Eigen::ArrayXXd DistanceMap_t;
typedef Eigen::ArrayXXi ReachabilityMap_t;


class MapServerPlanExp {
public:
  MapServerPlanExp(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  inline std::shared_ptr<CostMap_t> getCostMapPtr() { return map_cost_; }
  inline std::shared_ptr<const CostMap_t> getCostMapPtr() const { return map_cost_; }

  inline std::shared_ptr<OccupancyMap_t> getOccupancyMapPtr() { return map_state_; }
  inline std::shared_ptr<const OccupancyMap_t> getOccupancyMapPtr() const { return map_state_; }

  inline std::shared_ptr<UncertaintyMap_t> getUncertaintyMapPtr() { return map_uncertainty_; }
  inline std::shared_ptr<const UncertaintyMap_t> getUncertaintyMapPtr() const { return map_uncertainty_; }

  inline std::shared_ptr<CostMap_t> getHeightMapPtr() { return map_height_; }
  inline std::shared_ptr<const CostMap_t> getHeightMapPtr() const { return map_height_; }

  inline std::shared_ptr<UncertaintyMap_t> getHeightUncertaintyMapPtr() { return map_height_uncertainty_; }
  inline std::shared_ptr<const UncertaintyMap_t> getHeightUncertaintyMapPtr() const { return map_height_uncertainty_; }

  inline std::shared_ptr<DistanceMap_t> getDistanceMapPtr() { return map_distance_closest_; }
  inline std::shared_ptr<const DistanceMap_t> getDistanceMapPtr() const { return map_distance_closest_; }

  inline std::shared_ptr<CostMap_t> getCostClosestMapPtr() { return map_cost_closest_; }
  inline std::shared_ptr<const CostMap_t> getCostClosestMapPtr() const { return map_cost_closest_; }

  inline std::shared_ptr<ReachabilityMap_t> getReachabilityMapPtr() { return map_reachability_; }
  inline std::shared_ptr<const ReachabilityMap_t> getReachabilityMapPtr() const { return map_reachability_; }

  double getMapWidth() const {return width_;};
  double getMapHeight() const {return height_;};
  double getMapResolutionH() const {return horizontal_units_;};
  double getMapResolutionV() const {return vertical_units_;};
  double getCellSize() const {return m_per_pixel_;};
  double getMeanCost() const {return mean_cost_;};
  double getMaxCost() const {return max_cost_;};
  Eigen::Vector3d getGoalPosition() const {return goal_position_;}
  Eigen::Vector3d getStartPosition() const {return start_position_;}

  double getCostAtLocation(Eigen::Vector2d position);
  int getStatusAtLocation(Eigen::Vector2d position);
  double getUncertaintyAtLocation(Eigen::Vector2d position);
  double getHeightAtLocation(Eigen::Vector2d position);
  double getHeightUncertaintyAtLocation(Eigen::Vector2d position);
  double getClosestCostAtLocation(Eigen::Vector2d position);
  bool getReachabilityAtLocation(Eigen::Vector2d position);

  Eigen::ArrayXXi getCurrentPathMap();
  int getCurrentPathMapId() const {return path_counter_;}
  int getCurrentPathMapStatusAtLocation(Eigen::Vector2i point);
  int getCurrentPathMapStatusAtPosition(Eigen::Vector3d position)
    {return getCurrentPathMapStatusAtLocation(position2pixelLocation(position));};
  void request_path() {path_requested_ = true;};

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber map_data_sub_;
  ros::Subscriber depth_data_sub_;

  ros::Publisher planner_status_pub_;

  ros::ServiceServer clear_map_srv_;
  ros::ServiceServer save_map_srv_;
  ros::ServiceServer load_map_srv_;
  ros::ServiceServer publish_map_srv_;
  ros::ServiceServer goal_explored_srv_;

  image_transport::Publisher occupancy_map_vis_pub_;
  image_transport::Publisher cost_map_vis_pub_;
  image_transport::Publisher uncertainty_map_vis_pub_;

  bool p_use_uncertainty_map_;
  bool p_use_height_map_;
  bool p_compute_reachability_;
  bool p_compute_closest_observed_;
  bool p_compute_path_;

#if RVIZ_VISUALIZATION
  ros::Publisher cost_map_rviz_pub_;
  int cost_map_rviz_seq_id_{0};

#if CREATE_RRT_PLANNER_MAP
  ros::Publisher path_estimate_vis_pub_;
  int path_estimate_vis_seq_id_{0};
#endif
#endif

  int vis_seq_id_{0};
  int planner_status_seq_id_{0};

  int unknown_id_{3599};
  int obstacle_id_{0};
  int rov_default_obstacle_id_{0};
  std::vector<int> obstacle_ids_;
  std::vector<int> rov_obstacle_ids_;

  double height_{30.0};
  double width_{40.0};

  unsigned int vertical_units_{480};
  unsigned int horizontal_units_{640};

  double m_per_pixel_{0.0};

  double mean_cost_{0.0};
  double max_cost_{0.0};
  int mean_counter_{0};

  Eigen::Vector2i start_location_;
  Eigen::Vector2i goal_location_;
  Eigen::Vector3d start_position_;
  Eigen::Vector3d goal_position_;

  std::shared_ptr<CostMap_t> map_cost_;
  std::shared_ptr<UncertaintyMap_t> map_uncertainty_;
  std::shared_ptr<OccupancyMap_t> map_state_;
  std::shared_ptr<HeightMap_t> map_height_;
  std::shared_ptr<UncertaintyMap_t> map_height_uncertainty_;
  std::shared_ptr<CostMap_t> map_cost_closest_;
  std::shared_ptr<DistanceMap_t> map_distance_closest_;
  std::shared_ptr<ReachabilityMap_t> map_reachability_;

  std::vector<std::vector<Eigen::Vector2i>> connected_areas_;
  int max_area_id_;

  bool store_paths_;
  bool path_requested_;
  std::string paths_path_;
  int path_counter_{0};
  double min_rov_cost_{0.0};
  double max_rov_cost_{0.0};
  double unknown_init_cost_{0.0};
  std::vector<Eigen::Vector2d> current_path_estimate_;
  Eigen::ArrayXXi current_path_map_estimate_;
  std::vector<Eigen::Vector2i> current_discrete_path_estimate_;
  std::thread* planner_thread_;
  std::atomic<bool> planner_thread_done_;
  std::mutex current_path_estimate_mutex_;
  planexp_base_planners::PlannerType planner_type_;
  std::unique_ptr<planexp_base_planners::PlannerBase> planner_;

  inline bool idIsObstacle(const int id) {return (std::find(obstacle_ids_.begin(), obstacle_ids_.end(), id) != obstacle_ids_.end());};
  inline bool idIsRovObstacle(const int id) {return (std::find(rov_obstacle_ids_.begin(), rov_obstacle_ids_.end(), id) != rov_obstacle_ids_.end());};
  inline bool goalExplored() {return ((*map_state_)(AM(goal_location_)) != MSR_UNKNOWN);};
  void mapDataCallback(const planexp_msgs::MapData& msg);
  void depthDataCallback(const planexp_msgs::MapData& msg);
  bool clearMapCallback(std_srvs::Empty::Request& request,
                        std_srvs::Empty::Response& response);
  bool saveMapCallback(planexp_msgs::FilePath::Request& request,
                       planexp_msgs::FilePath::Response& response);
  bool loadMapCallback(planexp_msgs::FilePath::Request& request,
                       planexp_msgs::FilePath::Response& response);
  bool publishMapCallback(std_srvs::Empty::Request& request,
                          std_srvs::Empty::Response& response);
  bool goalExploredCallback(std_srvs::Trigger::Request& request,
                            std_srvs::Trigger::Response& response);
  bool clearMap();
  bool saveMap(std::string path);
  bool loadMap(std::string path);
  Eigen::ArrayXXi msg2EigenArray(const std_msgs::Int32MultiArray& msg);
  Eigen::ArrayXXd msg2EigenArrayd(const std_msgs::Int32MultiArray& msg);
  Eigen::Vector3d msg2Position(const geometry_msgs::Pose& msg);
  Eigen::Vector2i position2pixelLocation(const Eigen::Vector3d position);

  void occupancyMap2image(cv::Mat& occupancy_image);
  void uncertaintyMap2image(cv::Mat& uncertainty_image);
  void costMap2image(cv::Mat& cost_image);
  void heightMap2image(cv::Mat& height_image);
  void heightUncertaintyMap2image(cv::Mat& height_uncertainty_image);

  void publishVisualisation();
#if RVIZ_VISUALIZATION
  void publishRvizCostMap();
  void publishOccupancyMap();
  void eigen2occupancyMap(std::vector<signed char>& data);
#if CREATE_RRT_PLANNER_MAP
  void publishPathEstimate();
#endif
#endif
#if CREATE_RRT_PLANNER_MAP
  void createRRTPlannerMap(cv::Mat& rrt_image);
  void labels2image(cv::Mat& labels);
  void floorImg(cv::Mat& image);
  void moduloImg(cv::Mat& image, int n);
#endif
  void publishPlannerStatus(planexp_base_planners::ResultStatus status);

  void setClosestObservedMaps(Eigen::Vector2i top_left, Eigen::Vector2i bottom_right);
  void setReachabilityMap(Eigen::Vector2i point);

  void createInternalRRTStarPlannerMap(std::shared_ptr<Eigen::ArrayXXd> map);
  void computeCurrentPathEstimate();
  Eigen::ArrayXXi computePathMap(std::vector<Eigen::Vector2d>& path);
  Eigen::ArrayXXf eigen2gridMap();

  //TODO: Move this function to a better place and rename it
  unsigned int cost2gridMapColor(double cost) {
    assert(abs(cost) <= 1);
    int r, g, b;
    if (cost == 0) {
      r = 0;
      g = 0;
      b = 0;
    } else if (cost == -1) {
      r = 127;
      g = r;
      b = r;
    } else if (cost == -0.5) {
      // Purple
      r = 160;
      g = 32;
      b = 240;
      // Blue
      r = 0;
      g = 0;
      b = 245;
    } else if (cost >= 0.5) {
      r = 255;
      g = 2.0 * (1 - cost) * 255.0;
      b = 0;
    } else {
      r = 2.0 * cost * 255.0;
      g = 255;
      b = 0;
    }
    return (r << 16) + (g << 8) + b;
  }

};

#endif //ROS_WS_MAP_SERVER_PLANEXP_H
