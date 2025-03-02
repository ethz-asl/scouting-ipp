//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#ifndef PLANNER_MODULES_PLANEXP_EXPAND_REACHABLE_AREA_EVALUATOR_H
#define PLANNER_MODULES_PLANEXP_EXPAND_REACHABLE_AREA_EVALUATOR_H

#include <memory>

#include "planner_modules_planexp/map/map_planexp.h"
#include "active_3d_planning_core/module/trajectory_evaluator/simulated_sensor_evaluator.h"

namespace active_3d_planning {
  namespace trajectory_evaluator {

// ExpandReachableAreaEvaluator just counts the number of yet unobserved visible voxels which border reachable area.
    class ExpandReachableAreaEvaluator : public SimulatedSensorEvaluator {
    public:
      explicit ExpandReachableAreaEvaluator(PlannerI& planner);  // NOLINT

      void setupFromParamMap(Module::ParamMap* param_map) override;

    protected:
      static ModuleFactoryRegistry::Registration<ExpandReachableAreaEvaluator> registration;
      map::MapPlanExp* map_;
      double p_discount_factor_;

      // Override virtual methods
      bool computeGainFromVisibleVoxels(TrajectorySegment* traj_in) override;
    };

  }  // namespace trajectory_evaluator
}  // namespace active_3d_planning
#endif //PLANNER_MODULES_PLANEXP_EXPAND_REACHABLE_AREA_EVALUATOR_H
