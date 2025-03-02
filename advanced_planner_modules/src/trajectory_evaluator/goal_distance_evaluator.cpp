//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//
#include "planner_modules_planexp/trajectory_evaluator/goal_distance_evaluator.h"

#include <algorithm>

namespace active_3d_planning {
  namespace trajectory_evaluator {

// Factory Registration
    ModuleFactoryRegistry::Registration<GoalDistanceEvaluator>
        GoalDistanceEvaluator::registration("GoalDistanceEvaluator");

    GoalDistanceEvaluator::GoalDistanceEvaluator(PlannerI& planner)
        : SimulatedSensorEvaluator(planner) {}

    void GoalDistanceEvaluator::setupFromParamMap(Module::ParamMap* param_map) {
      // setup parent
      SimulatedSensorEvaluator::setupFromParamMap(param_map);

      setParam<double>(param_map, "max_gain", &p_max_gain_, 100.0);

      // setup map
      map_ = dynamic_cast<map::MapPlanExp*>(&(planner_.getMap()));
      if (!map_) {
        planner_.printError(
            "'GoalDistanceEvaluator' requires a map of type 'MapPlanExp'!");
      }

    }

    bool GoalDistanceEvaluator::computeGainFromVisibleVoxels(TrajectorySegment* traj_in) {
      if (!traj_in->info) {
        traj_in->gain = 0.0;
        return false;
      }
      // remove all already observed voxels, count number of new voxels
      SimulatedSensorInfo* info =
          reinterpret_cast<SimulatedSensorInfo*>(traj_in->info.get());
      size_t i = 0;
      while (i < info->visible_voxels.size()) {
        if (planner_.getMap().isObserved(info->visible_voxels[i])) {
          info->visible_voxels.erase(info->visible_voxels.begin() + i);
        } else {
          ++i;
        }
      }

      Eigen::Vector3d goal = map_->getGoalPosition();
      double gain = 0.0;
      double map_extent = map_->getMapExtent();
      for (auto voxel:info->visible_voxels) {
        double distance = (goal-voxel).norm();
        //gain += (map_extent-distance) / map_extent;
        if (distance > 1.0/p_max_gain_) {
          gain += 1.0/distance;
        } else {
          gain += p_max_gain_;
        }
      }

      traj_in->gain = gain;
      return true;
    }

  }  // namespace trajectory_evaluator
}  // namespace active_3d_planning