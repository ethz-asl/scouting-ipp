//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//
#include "planner_modules_planexp/cost_computer/cost_planexp_map_integral.h"

namespace active_3d_planning {
  namespace cost_computer {
// CostPlanExpMapIntegral
    ModuleFactoryRegistry::Registration<CostPlanExpMapIntegral> CostPlanExpMapIntegral::registration(
        "CostPlanExpMapIntegral");

    CostPlanExpMapIntegral::CostPlanExpMapIntegral(PlannerI& planner) : CostComputer(planner) {}

    void CostPlanExpMapIntegral::setupFromParamMap(Module::ParamMap* param_map) {
      setParam<bool>(param_map, "accumulate", &p_accumulate_, false);
      setParam<double>(param_map, "max_sample_distance", &max_sample_distance_, 0.05);

      map_ = dynamic_cast<map::MapPlanExp*>(&(planner_.getMap()));
    }

    bool CostPlanExpMapIntegral::computeCost(TrajectorySegment* traj_in) {
      if (traj_in->trajectory.size() < 2) {
        traj_in->cost = 0.0;
        return false;
      }
      // Linear path interpolation
      double distance = 0.0;
      double segment_distance = 0.0;
      double subsegment_distance = 0.0;
      double cost = 0.0;
      double segment_cost = 0.0;
      double subsegment_cost = 0.0;
      Eigen::Vector3d direction, start_subsegment, end_subsegment;

      Eigen::Vector3d last_point = traj_in->trajectory[0].position_W;
      for (int i = 1; i < traj_in->trajectory.size(); ++i) {
        segment_distance = (traj_in->trajectory[i].position_W - last_point).norm();
        if (segment_distance > 0.005) {
          direction = (traj_in->trajectory[i].position_W - last_point) / segment_distance;
          int n_sub_segments = (int)(segment_distance / max_sample_distance_);
          segment_cost = 0;
          start_subsegment = last_point;
          end_subsegment = last_point;
          for (int j = 0; j <= n_sub_segments; ++j) {
            subsegment_distance = max_sample_distance_;
            if (j == n_sub_segments) subsegment_distance = segment_distance - n_sub_segments * max_sample_distance_;
            end_subsegment += direction * subsegment_distance;
            subsegment_cost = subsegment_distance
                              * (map_->getVoxelWeight(start_subsegment) + map_->getVoxelWeight(end_subsegment)) / 2.0;
            if (map_->getVoxelWeight(start_subsegment) == 0 || map_->getVoxelWeight(end_subsegment) == 0)
              subsegment_cost = subsegment_distance * map_->getMeanWeight();
            segment_cost += subsegment_cost;
            start_subsegment = end_subsegment;
          }
        } else {
          if (map_->getVoxelWeight(traj_in->trajectory[i].position_W) == 0 ) {
            segment_cost = segment_distance * map_->getMeanWeight();
          } else {
            segment_cost = segment_distance * map_->getVoxelWeight(traj_in->trajectory[i].position_W);
          }
        }
        distance += segment_distance;
        cost += segment_cost;

        last_point = traj_in->trajectory[i].position_W;
      }
      traj_in->cost = cost;
      if (p_accumulate_ && traj_in->parent) {
        traj_in->cost += traj_in->parent->cost;
      }
      return true;
    }

  }  // namespace cost_computer
}  // namespace active_3d_planning
