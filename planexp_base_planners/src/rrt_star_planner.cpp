//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//
#include "planexp_base_planners/rrt_star_planner.h"

namespace planexp_base_planners {

  RRTStarPlanner::RRTStarPlanner(PlannerParams params) : PlannerBase(params) {
    planner_ = nullptr;
    map_ = std::make_shared<Eigen::ArrayXXd>(params_.px_y, params_.px_x);
    map_->setConstant(params_.init_cost);
    explored_ = std::make_shared<Eigen::ArrayXXi>(params_.px_y, params_.px_x);
    explored_->setConstant(CellStatus_t::UNEXPLORED);
  }

  void RRTStarPlanner::setup() {

  }

  ResultStatus RRTStarPlanner::plan(std::vector<Eigen::Vector2d> *path, double& cost) {
    planner_ = std::make_unique<planexp_base_planners::RRTStarPlannerBase>(map_, params_);
    planner_->setup();
    planner_->plan(path, cost);
    if (cost <= 0.0) return ResultStatus::INVALID;
    else return ResultStatus::VALID;
  }

  void RRTStarPlanner::store_path(const std::string file_name, const std::string path) {
    planner_->store_path(file_name, path);
  }

  void RRTStarPlanner::update_states(Eigen::Vector2i location, Eigen::ArrayXXi map_data) {
    for (int i = 0; i < map_data.rows(); ++i) {
      for (int j = 0; j < map_data.cols(); ++j) {
        int ix = location.x() + j;
        int iy = location.y() + i;
        if (ix < 0 || ix >= params_.px_x || iy < 0 || iy >= params_.px_y) continue;
        if (idIsObstacle(map_data(i, j))) (*map_)(iy, ix) = 0.0;
        //else if (map_data(i, j) == params_.unknown_id) (*map_)(iy, ix) = params_.min_cost; //TODO: Check if that works and leads to the intended behaviour.
        else (*map_)(iy, ix) = map_data(i, j);
        (*explored_)(iy, ix) = CellStatus_t::EXPLORED;
      }
    }
  }

  void RRTStarPlanner::update_unknown_cost(double new_cost) {
    for (int i = 0; i < map_->rows(); ++i) {
      for (int j = 0; j < map_->cols(); ++j) {
        if ((*explored_)(i,j) == CellStatus_t::UNEXPLORED) (*map_)(i, j) = new_cost;
      }
    }
  }
}
