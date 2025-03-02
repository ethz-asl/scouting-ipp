#ifndef PLANEXP_BASE_PLANNERS_TYPES_H
#define PLANEXP_BASE_PLANNERS_TYPES_H

#include <Eigen/Eigen>

typedef Eigen::MatrixXi map_t;

enum MapSource {
  MS_RANDOM = 0,
  MS_OAISYS = 1,
  MS_IMAGE = 2
};

enum MapType {
  MT_UNKNOWN = 0,
  MT_REAL = 1,
  MT_ARTIFICIAL = 2
};

struct config_t {
  MapSource map_source = MS_IMAGE;
  std::string map_path = "";
  double map_width = 0.0;
  double map_height = 0.0;
  std::vector<int> intraversable_label_ids = std::vector<int>();
  int max_iterations = -1;
  std::string map_topic = "";
  double min_distance = 0.0;
  double sample_dist = 0.0;
  double step_range = 0.0;
  double optim_time = 0.0;
  double goal_bias = 0.0;
  double success_threshold = 0.05;
  bool verbose = true;
  bool save_output_map = true;
  bool save_tree_map = false;
  bool save_path = true;
};

struct map_config_t {
  std::string name = "";
  MapType type = MT_UNKNOWN;
  std::vector<int> resolution = std::vector<int>(2, 0);
  double width = 0.0;
  double height = 0.0;
  int n_data_classes = 15;
  std::vector<int> present_ids = std::vector<int>();
  std::vector<int> obstacle_ids_mav = std::vector<int>();
  std::vector<int> obstacle_ids_rov = std::vector<int>();
  int unknown_id = std::pow(n_data_classes, 3) + std::pow(n_data_classes, 2) - 1;
  std::vector<double> start_location = std::vector<double>(2, 0.0);
  std::vector<double> goal_location = std::vector<double>(2, 0.0);
  double explorable_free_space = 0.0;
};


#endif //PLANEXP_BASE_PLANNERS_TYPES_H