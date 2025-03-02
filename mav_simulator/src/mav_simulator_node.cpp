//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//
#include "mav_simulator/mav_simulator.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "mav_simulator");

  // Set logging to debug for testing
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Debug);

  // node handles
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  auto mav_simulator = MavSimulator(nh, nh_private);

  mav_simulator.mainLoop();

  return 0;

};