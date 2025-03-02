//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#ifndef PLANNER_MODULES_PLANEXP_AVERAGE_VIEW_COST_H
#define PLANNER_MODULES_PLANEXP_AVERAGE_VIEW_COST_H

#include <memory>

#include "planner_modules_planexp/map/map_planexp.h"
#include "active_3d_planning_core/module/trajectory_evaluator/simulated_sensor_evaluator.h"

namespace active_3d_planning {
  namespace trajectory_evaluator {

// AverageViewCostEvaluator just counts the number of yet unobserved visible voxels weighted by their euclidean distance to the goal location.
    class AverageViewCostEvaluator : public SimulatedSensorEvaluator {
    public:
      explicit AverageViewCostEvaluator(PlannerI& planner);  // NOLINT

      void setupFromParamMap(Module::ParamMap* param_map) override;

    protected:
      static ModuleFactoryRegistry::Registration<AverageViewCostEvaluator> registration;
      map::MapPlanExp* map_;

      // Override virtual methods
      bool computeGainFromVisibleVoxels(TrajectorySegment* traj_in) override;
    };

  }  // namespace trajectory_evaluator
}  // namespace active_3d_planning
#endif //PLANNER_MODULES_PLANEXP_AVERAGE_VIEW_COST_H
