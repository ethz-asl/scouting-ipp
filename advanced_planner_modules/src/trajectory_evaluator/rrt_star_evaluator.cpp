//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//
#include "planner_modules_planexp/trajectory_evaluator/rrt_star_evaluator.h"

#include <algorithm>

namespace active_3d_planning {
  namespace trajectory_evaluator {

// Factory Registration
    ModuleFactoryRegistry::Registration<RRTStarEvaluator>
        RRTStarEvaluator::registration("RRTStarEvaluator");

    RRTStarEvaluator::RRTStarEvaluator(PlannerI& planner)
        : SimulatedSensorEvaluator(planner) {}

    void RRTStarEvaluator::setupFromParamMap(Module::ParamMap* param_map) {
      // setup parent
      SimulatedSensorEvaluator::setupFromParamMap(param_map);

      setParam<double>(param_map, "non_path_voxel_gain", &p_non_path_voxel_gain_, 0.0);
      setParam<bool>(param_map, "use_distance_discount", &p_use_distance_discount_, false);
      setParam<double>(param_map, "discount_offset", &p_discount_offset_, 0.1);
      setParam<bool>(param_map, "do_binary_check", &p_do_binary_check_, false);

      // setup map
      map_ = dynamic_cast<map::MapPlanExp*>(&(planner_.getMap()));
      if (!map_) {
        planner_.printError(
            "'RRTStarEvaluator' requires a map of type 'MapPlanExp'!");
      }

    }

    bool RRTStarEvaluator::computeGainFromVisibleVoxels(TrajectorySegment* traj_in) {
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

      double gain = 0.0;
      if (p_do_binary_check_) {
        if (p_use_distance_discount_) {
          Eigen::Vector3d goal = map_->getGoalPosition();
          for (auto voxel:info->visible_voxels) {
            if (map_->getPathMapStatusAtPosition(voxel) == 1) {
              gain = 1.0 / sqrt((goal-traj_in->trajectory[0].position_W).norm() + p_discount_offset_);
              break;
            }
          }
        } else {
          for (auto voxel:info->visible_voxels) {
            if (map_->getPathMapStatusAtPosition(voxel) == 1) {
              gain = 1.0;
              break;
            }
          }
        }
      } else {
        if (p_use_distance_discount_) {
          Eigen::Vector3d goal = map_->getGoalPosition();
          for (auto voxel:info->visible_voxels) {
            if (map_->getPathMapStatusAtPosition(voxel) == 1)
              gain += 1.0 / sqrt((goal-voxel).norm() + p_discount_offset_);
            else gain += p_non_path_voxel_gain_;
          }
        } else {
          for (auto voxel:info->visible_voxels) {
            if (map_->getPathMapStatusAtPosition(voxel) == 1) gain += 1.0;
            else gain += p_non_path_voxel_gain_;
          }
        }
      }

      traj_in->gain = gain;
      return true;
    }

  }  // namespace trajectory_evaluator
}  // namespace active_3d_planning