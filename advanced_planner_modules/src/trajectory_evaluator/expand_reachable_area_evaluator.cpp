//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//
#include "planner_modules_planexp/trajectory_evaluator/expand_reachable_area_evaluator.h"

#include <algorithm>

namespace active_3d_planning {
  namespace trajectory_evaluator {

// Factory Registration
    ModuleFactoryRegistry::Registration<ExpandReachableAreaEvaluator>
        ExpandReachableAreaEvaluator::registration("ExpandReachableAreaEvaluator");

    ExpandReachableAreaEvaluator::ExpandReachableAreaEvaluator(PlannerI& planner)
        : SimulatedSensorEvaluator(planner) {}

    void ExpandReachableAreaEvaluator::setupFromParamMap(Module::ParamMap* param_map) {
      // setup parent
      SimulatedSensorEvaluator::setupFromParamMap(param_map);

      setParam<double>(param_map, "discount_factor", &p_discount_factor_, 0.1);

      // setup map
      map_ = dynamic_cast<map::MapPlanExp*>(&(planner_.getMap()));
      if (!map_) {
        planner_.printError(
            "'ExpandReachableAreaEvaluator' requires a map of type 'MapPlanExp'!");
      }

    }

    bool ExpandReachableAreaEvaluator::computeGainFromVisibleVoxels(TrajectorySegment* traj_in) {
      if (!traj_in->info) {
        traj_in->gain = 0.0;
        return false;
      }
      // remove all already observed voxels, count number of new voxels
      SimulatedSensorInfo* info =
          reinterpret_cast<SimulatedSensorInfo*>(traj_in->info.get());
      size_t i = 0;
      while (i < info->visible_voxels.size()) {
        if (map_->isObserved(info->visible_voxels[i])) {
          info->visible_voxels.erase(info->visible_voxels.begin() + i);
        } else {
          ++i;
        }
      }

      if (i == 0) {
        traj_in->gain = 0.0;
        traj_in->gain_terminal = true;
        return true;
      }

      size_t n_reachable_voxels = 0;
      for (auto voxel:info->visible_voxels) {
        bool is_reachable = false;
        std::vector<Eigen::Vector3d> neighbour_voxels;
        map_->getVoxelArea(&neighbour_voxels, voxel, 1);
        for (auto neighbour_voxel:neighbour_voxels) is_reachable |= (map_->isReachable(neighbour_voxel));
        if (is_reachable) ++n_reachable_voxels;
      }

      traj_in->gain = n_reachable_voxels + (info->visible_voxels.size() - n_reachable_voxels) * p_discount_factor_;
      return true;
    }

  }  // namespace trajectory_evaluator
}  // namespace active_3d_planning