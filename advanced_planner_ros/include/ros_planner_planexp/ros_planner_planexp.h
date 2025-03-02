//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#ifndef ROS_PLANNER_PLANEXP_PLANNER_ROS_PLANEXP_H
#define ROS_PLANNER_PLANEXP_PLANNER_ROS_PLANEXP_H


#include <ctime>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/MarkerArray.h>

#include "active_3d_planning_core/planner/online_planner.h"
//#include "planner_modules_planexp/planner/planner_planexp.h"

namespace active_3d_planning {
  namespace ros {

    class RosPlannerPlanExp : public OnlinePlanner {
    public:
      RosPlannerPlanExp(const ::ros::NodeHandle& nh, const ::ros::NodeHandle& nh_private,
                 ModuleFactory* factory, Module::ParamMap* param_map);

      virtual ~RosPlannerPlanExp() = default;

      // ros callbacks
      void odomCallback(const nav_msgs::Odometry& msg);

      bool runSrvCallback(std_srvs::SetBool::Request& req,    // NOLINT
                          std_srvs::SetBool::Response& res);  // NOLINT

      bool cpuSrvCallback(std_srvs::SetBool::Request& req,    // NOLINT
                          std_srvs::SetBool::Response& res);  // NOLINT

      void publishVisualization(const VisualizationMarkers& markers) override;

      void planningLoop() override;

      // initialization helper
      static void setupFactoryAndParams(ModuleFactory* factory,
                                        Module::ParamMap* param_map,
                                        const ::ros::NodeHandle& nh_private);

      // logging and printing
      void printInfo(const std::string& text) override;

      void printWarning(const std::string& text) override;

      void printError(const std::string& text) override;

    protected:
      // ros
      ::ros::NodeHandle nh_;
      ::ros::NodeHandle nh_private_;
      ::ros::Subscriber odom_sub_;
      ::ros::Publisher target_pub_;
      ::ros::Publisher trajectory_vis_pub_;
      ::ros::ServiceServer run_srv_;
      ::ros::ServiceServer get_cpu_time_srv_;

      // variables
      ::ros::Time ros_timing_;      // track simulated time
      std::clock_t cpu_srv_timer_;  // To get CPU usage for service
      std::map<std::string, int>
          visualization_overwrite_counter_;  // store the previous number of
      // visualizations to overwrite in RVIZ

      // params
      double p_replan_pos_threshold_;  // m, when is the goal considered reached
      double p_replan_yaw_threshold_;  // rad

      // override/adapt planner methods
      void initializePlanning() override;

      bool requestNextTrajectory() override;

      void requestMovement(const EigenTrajectoryPointVector& trajectory) override;

      void setupFromParamMap(Module::ParamMap* param_map) override;
    };

  }  // namespace ros
}  // namespace active_3d_planning

#endif //ROS_PLANNER_PLANEXP_PLANNER_ROS_PLANEXP_H
