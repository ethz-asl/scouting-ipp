//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#ifndef PLANNER_MODULES_PLANEXP_RRT_STAR_EVALUATOR_H
#define PLANNER_MODULES_PLANEXP_RRT_STAR_EVALUATOR_H

#include <memory>

#include "planner_modules_planexp/map/map_planexp.h"
#include "active_3d_planning_core/module/trajectory_evaluator/simulated_sensor_evaluator.h"

namespace active_3d_planning {
  namespace trajectory_evaluator {

    class RRTStarEvaluator : public SimulatedSensorEvaluator {
    public:
      explicit RRTStarEvaluator(PlannerI& planner);  // NOLINT

      void setupFromParamMap(Module::ParamMap* param_map) override;

    protected:
      static ModuleFactoryRegistry::Registration<RRTStarEvaluator> registration;
      map::MapPlanExp* map_;
      double p_non_path_voxel_gain_;
      bool p_use_distance_discount_;
      double p_discount_offset_;
      bool p_do_binary_check_;

      // Override virtual methods
      bool computeGainFromVisibleVoxels(TrajectorySegment* traj_in) override;

    };

  }  // namespace trajectory_evaluator
}  // namespace active_3d_planning
#endif //PLANNER_MODULES_PLANEXP_RRT_STAR_EVALUATOR_H
