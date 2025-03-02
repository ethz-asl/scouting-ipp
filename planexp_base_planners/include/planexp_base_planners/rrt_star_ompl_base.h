//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#ifndef PLANEXP_BASE_PLANNERS_RRT_STAR_OMPL_BASE_H
#define PLANEXP_BASE_PLANNERS_RRT_STAR_OMPL_BASE_H

#include "planexp_base_planners/base_planner.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/util/Console.h>
#include <ompl/config.h>

#include <Eigen/Eigen>
#include <vector>
#include <fstream>

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
  class RRTStarPlannerBase {
  public:
    RRTStarPlannerBase(std::shared_ptr<Eigen::ArrayXXd> map,  planexp_base_planners::PlannerParams params);
    ~RRTStarPlannerBase() = default;

    void setup();
    void plan(std::vector<Eigen::Vector2d>* path, double& cost);
    void store_path(const std::string file_name, const std::string path);

    class myValidityChecker : public ompl::base::StateValidityChecker
    {
    public:
      //myValidityChecker(const ompl::base::SpaceInformationPtr& si, Eigen::ArrayXXd* map, double width, double height, std::vector<int> obstacle_ids) :
      //    ompl::base::StateValidityChecker(si), map_(map), width_(width), height_(height), obstacle_ids_(obstacle_ids) {}
      myValidityChecker(const ompl::base::SpaceInformationPtr& si, std::shared_ptr<Eigen::ArrayXXd> map, double width, double height) :
          ompl::base::StateValidityChecker(si), map_(map), width_(width), height_(height) {};

      bool isValid(const ompl::base::State* state) const override;

    protected:
      std::shared_ptr<Eigen::ArrayXXd> map_;
      double width_, height_;
      //std::vector<int> obstacle_ids_;

      //inline bool idIsObstacle(const int id) const {return (std::find(obstacle_ids_.begin(), obstacle_ids_.end(), id) != obstacle_ids_.end());};
    };

    class myObjective : public ompl::base::StateCostIntegralObjective {
    public:
      myObjective(const ompl::base::SpaceInformationPtr &si, std::shared_ptr<Eigen::ArrayXXd> map, double width, double height,
                  double sample_dist) :
          ompl::base::StateCostIntegralObjective(si, true), map_(map), width_(width), height_(height),
          sample_dist_(sample_dist) {};

      ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const override;
    protected:
      std::shared_ptr<Eigen::ArrayXXd> map_;
      double width_, height_;
      double sample_dist_;
    };

  protected:
    double sample_distance_;
    planexp_base_planners::PlannerParams params_;
    std::shared_ptr<Eigen::ArrayXXd> map_;

    ompl::base::SpaceInformationPtr space_information_;
    ompl::base::ProblemDefinitionPtr problem_definition_;
    std::shared_ptr<ompl::geometric::RRTstar> rrt_star_planner_;
    ompl::base::PlannerDataPtr planner_data_;

    void setupGeneralProblemDefinition();
    void setupPlanner();
    void solve();
    void solve(double solve_time);
    void getResults(std::vector<Eigen::Vector2d>* path, double& cost);

    std::vector<Eigen::Vector2d> pathPtr_to_vector(ompl::base::PathPtr path_ptr);

    inline ompl::base::OptimizationObjectivePtr getObjective(const ompl::base::SpaceInformationPtr& si)
    {
      return (ompl::base::OptimizationObjectivePtr(new myObjective(si, map_, params_.width, params_.height, sample_distance_)));
    }

    inline ompl::base::StateValidityCheckerPtr getValidity(const ompl::base::SpaceInformationPtr& si)
    {
      return (ompl::base::StateValidityCheckerPtr(new myValidityChecker(si, map_, params_.width, params_.height)));
    }

  private:

  };
}

#endif // PLANEXP_BASE_PLANNERS_RRT_STAR_OMPL_BASE_H
