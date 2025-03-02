//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#ifndef MAV_SIMULATOR_PLANEXP_TYPES_H
#define MAV_SIMULATOR_PLANEXP_TYPES_H
enum CostFunction_t {
  MapCost,
  DistanceCost,
  TimeCost
};

enum MapType_t {
  MT_UNKNOWN = 0,
  MT_REAL = 1,
  MT_ARTIFICIAL = 2
};

struct map_config_t {
  std::string name = "";
  MapType_t type = MT_UNKNOWN;
  std::vector<int> resolution = std::vector<int>(2, 0);
  double width = 0.0;
  double height = 0.0;
  int n_data_classes = 15;
  std::vector<int> present_ids = std::vector<int>();
  std::vector<int> obstacle_ids_mav = std::vector<int>();
  std::vector<int> obstacle_ids_rov = std::vector<int>();
  int unknown_id = 3615;
  std::vector<double> start_location = std::vector<double>(2, 0.0);
  std::vector<double> goal_location = std::vector<double>(2, 0.0);
  double explorable_free_space = 0.0;
};

#endif //MAV_SIMULATOR_PLANEXP_TYPES_H
