//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//
#include "planner_modules_planexp/trajectory_evaluator/expand_low_cost_area_evaluator.h"

#include <algorithm>

namespace active_3d_planning {
  namespace trajectory_evaluator {

// Factory Registration
    ModuleFactoryRegistry::Registration<ExpandLowCostAreaEvaluator>
        ExpandLowCostAreaEvaluator::registration("ExpandLowCostAreaEvaluator");

    ExpandLowCostAreaEvaluator::ExpandLowCostAreaEvaluator(PlannerI& planner)
        : SimulatedSensorEvaluator(planner) {}

    void ExpandLowCostAreaEvaluator::setupFromParamMap(Module::ParamMap* param_map) {
      // setup parent
      SimulatedSensorEvaluator::setupFromParamMap(param_map);

      setParam<bool>(param_map, "augment_unknown_with_mean", &p_augment_unknown_with_mean_, false);

      // setup map
      map_ = dynamic_cast<map::MapPlanExp*>(&(planner_.getMap()));
      if (!map_) {
        planner_.printError(
            "'ExpandLowCostAreaEvaluator' requires a map of type 'MapPlanExp'!");
      }

    }

    bool ExpandLowCostAreaEvaluator::computeGainFromVisibleVoxels(TrajectorySegment* traj_in) {
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

      double gain = 0.0;
      if (p_augment_unknown_with_mean_) {
        for (auto voxel:info->visible_voxels) {
          double cost = map_->getVoxelClosestObservedWeight(voxel); //TODO: Handle edgecases for not computed weights.
          if (cost > 0.0) {
            gain += 1.0 / cost;
          } else {
            gain += 1.0 / map_->getMeanWeight();
          }
        }
      } else {
        for (auto voxel:info->visible_voxels) {
          double cost = map_->getVoxelClosestObservedWeight(voxel); //TODO: Handle edgecases for not computed weights.
          if (cost > 0.0) {
            gain += 1.0 / cost;
          } else {
            gain += 0.0;
          }
        }
      }
      traj_in->gain = gain;
      return true;
    }

  }  // namespace trajectory_evaluator
}  // namespace active_3d_planning