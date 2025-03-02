//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//
#include "planner_modules_planexp/trajectory_evaluator/average_view_cost_evaluator.h"

#include <algorithm>

namespace active_3d_planning {
  namespace trajectory_evaluator {

// Factory Registration
    ModuleFactoryRegistry::Registration<AverageViewCostEvaluator>
        AverageViewCostEvaluator::registration("AverageViewCostEvaluator");

    AverageViewCostEvaluator::AverageViewCostEvaluator(PlannerI& planner)
        : SimulatedSensorEvaluator(planner) {}

    void AverageViewCostEvaluator::setupFromParamMap(Module::ParamMap* param_map) {
      // setup parent
      SimulatedSensorEvaluator::setupFromParamMap(param_map);

      // setup map
      map_ = dynamic_cast<map::MapPlanExp*>(&(planner_.getMap()));
      if (!map_) {
        planner_.printError(
            "'AverageViewCostEvaluator' requires a map of type 'MapPlanExp'!");
      }

    }

    bool AverageViewCostEvaluator::computeGainFromVisibleVoxels(TrajectorySegment* traj_in) {
      if (!traj_in->info) {
        traj_in->gain = 0.0;
        return false;
      }
      // remove all already observed voxels, count number of new voxels
      SimulatedSensorInfo* info =
          reinterpret_cast<SimulatedSensorInfo*>(traj_in->info.get());
      size_t i = 0;
      size_t n_observed_voxels = 0;
      double mean_view_cost = 0.0;
      while (i < info->visible_voxels.size()) {
        if (map_->isObserved(info->visible_voxels[i])) {
          if (map_->getVoxelState(info->visible_voxels[i]) == MSR_FREE) {
            ++n_observed_voxels;
            mean_view_cost += map_->getVoxelWeight(info->visible_voxels[i]);
          }
          info->visible_voxels.erase(info->visible_voxels.begin() + i);
        } else {
          ++i;
        }
      }

      if (n_observed_voxels > 0) {
        mean_view_cost /= n_observed_voxels;
        traj_in->gain = info->visible_voxels.size() * (1.0/mean_view_cost);
      } else {
        traj_in->gain = info->visible_voxels.size() * (1.0/map_->getMeanWeight());
      }
      return true;
    }

  }  // namespace trajectory_evaluator
}  // namespace active_3d_planning