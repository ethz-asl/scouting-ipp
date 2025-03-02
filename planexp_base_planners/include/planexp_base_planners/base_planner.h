//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#ifndef PLANEXP_BASE_PLANNERS_BASE_PLANNER_H
#define PLANEXP_BASE_PLANNERS_BASE_PLANNER_H

#include <Eigen/Eigen>
#include <vector>

namespace planexp_base_planners {

  struct PlannerParams {
    int px_x;
    int px_y;

    double width;
    double height;

    double min_cost;
    double max_cost;
    double init_cost;

    Eigen::Vector2d start;
    Eigen::Vector2d goal;

    std::vector<int> obstacle_ids;
    int unknown_id;

    // RRT Specific
    double time_limit = 10.0;
    double goal_bias = 0.05;
    double step_range = 10.0;

    // PRM Specific

  };

  enum ResultStatus {
    VALID,
    INVALID,
    TERMINAL_VALID,
    TERMINAL_INVALID
  };

  enum PlannerType {
    UNKNOWN_PLANNER,
    RRT_STAR_PLANNER,
    PRM_STAR_PLANNER
  };

  enum CellStatus_t {
    UNEXPLORED,
    EXPLORED,
  };

  class PlannerBase {
  public:
    PlannerBase(PlannerParams params) : params_(params) {};
    virtual ~PlannerBase() = default;

    virtual void setup() = 0;
    virtual ResultStatus plan(std::vector<Eigen::Vector2d>* path, double& cost) = 0;
    virtual void store_path(const std::string file_name, const std::string path) = 0;
    virtual void update_states(Eigen::Vector2i location, Eigen::ArrayXXi map_data) = 0;
    virtual void update_unknown_cost(double new_cost) = 0;
    PlannerType get_planner_type() {return PlannerType::UNKNOWN_PLANNER;};

  protected:
    PlannerParams params_;
  };

}

#endif //PLANEXP_BASE_PLANNERS_BASE_PLANNER_H
