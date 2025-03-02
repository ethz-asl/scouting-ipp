//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//
#include <ros/ros.h>
#include <glog/logging.h>

#include "map_server_planexp/map_server_planexp.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_server_planexp");

  // Set logging to debug for testing
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Debug);
  google::InitGoogleLogging(argv[0]);

  // node handles
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  ROS_DEBUG("[map_server_planexp] Initializing!!");

  // Setup
  MapServerPlanExp map_server_(nh, nh_private);

  while(ros::ok()) {
    ros::spinOnce();
  }
}