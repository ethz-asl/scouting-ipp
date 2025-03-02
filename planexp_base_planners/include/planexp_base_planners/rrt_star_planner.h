//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#ifndef PLANEXP_BASE_PLANNERS_RRT_STAR_PLANNER_H
#define PLANEXP_BASE_PLANNERS_RRT_STAR_PLANNER_H

#include "planexp_base_planners/rrt_star_ompl_base.h"

#include <Eigen/Eigen>
#include <memory>


#if __has_include(<filesystem>)
#include <filesystem>
  namespace fs = std::filesystem;
#elif __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
error "Missing the <filesystem> header."
#endif

namespace planexp_base_planners {
  class RRTStarPlanner : public PlannerBase {
  public:
    explicit RRTStarPlanner(PlannerParams params);

    void setup() override;

    ResultStatus plan(std::vector<Eigen::Vector2d> *path, double& cost) override;

    void store_path(const std::string file_name, const std::string path) override;

    void update_states(Eigen::Vector2i location, Eigen::ArrayXXi map_data) override;

    void update_unknown_cost(double new_cost) override;

    PlannerType get_planner_type() {return PlannerType::RRT_STAR_PLANNER;}

  protected:
    std::shared_ptr<Eigen::ArrayXXd> map_;
    std::shared_ptr<Eigen::ArrayXXi> explored_;
    std::unique_ptr<planexp_base_planners::RRTStarPlannerBase> planner_;

    inline bool idIsObstacle(const int id) {return (std::find(params_.obstacle_ids.begin(), params_.obstacle_ids.end(), id) != params_.obstacle_ids.end());};
  };
}

#endif //PLANEXP_BASE_PLANNERS_RRT_STAR_PLANNER_H
