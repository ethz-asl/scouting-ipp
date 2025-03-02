//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#ifndef SENSOR_MODEL_PLANEXP_COST_PLANEXP_MAP_INTEGRAL_H
#define SENSOR_MODEL_PLANEXP_COST_PLANEXP_MAP_INTEGRAL_H

#include "active_3d_planning_core/module/trajectory_evaluator.h"
#include "planner_modules_planexp/map/map_planexp.h"

namespace active_3d_planning {
  namespace cost_computer {

// Travelled distance of a segment
    class CostPlanExpMapIntegral : public CostComputer {
    public:
      explicit CostPlanExpMapIntegral(PlannerI& planner);  // NOLINT

      void setupFromParamMap(Module::ParamMap* param_map) override;

      // override virtual functions
      bool computeCost(TrajectorySegment* traj_in) override;

    protected:
      static ModuleFactoryRegistry::Registration<CostPlanExpMapIntegral> registration;

      // params
      bool p_accumulate_;  // True: Use total length
      double max_sample_distance_;

      map::MapPlanExp* map_;

    };

  }  // namespace cost_computer
}  // namespace active_3d_planning

#endif //SENSOR_MODEL_PLANEXP_COST_PLANEXP_MAP_INTEGRAL_H
