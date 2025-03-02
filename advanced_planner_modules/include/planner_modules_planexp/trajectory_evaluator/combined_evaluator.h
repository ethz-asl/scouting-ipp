//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//
//TODO: WIP Has never been tested!
#ifndef PLANNER_MODULES_PLANEXP_COMBINED_EVALUATOR_H
#define PLANNER_MODULES_PLANEXP_COMBINED_EVALUATOR_H

#include <memory>
#include <string>
#include <vector>

#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
  namespace trajectory_evaluator {

// Base class for yaw adapting evaluators. Evaluates different yaws for the
// input trajectory and selects the best one. Evaluation and all other
// functionalities are delegated to the folowing evaluator.
    class CombinedEvaluator : public TrajectoryEvaluator {
    public:
      explicit CombinedEvaluator(PlannerI &planner);  // NOLINT

      // Override virtual functions
      bool computeGain(TrajectorySegment *traj_in) override;

      bool computeCost(TrajectorySegment *traj_in) override;

      bool computeValue(TrajectorySegment *traj_in) override;

      int selectNextBest(TrajectorySegment *traj_in) override;

      bool updateSegment(TrajectorySegment *segment) override;

      void visualizeTrajectoryValue(VisualizationMarkers *markers,
                                    const TrajectorySegment &trajectory) override;

      void setupFromParamMap(Module::ParamMap *param_map) override;

      bool checkParamsValid(std::string *error_message) override;

    protected:
      static ModuleFactoryRegistry::Registration<CombinedEvaluator> registration;
      // params
      std::string p_gain_combination_method_;
      std::string p_cost_combination_method_;

      // members
      std::vector<std::unique_ptr<TrajectoryEvaluator>> following_evaluators_;
    };
  }
}

#endif //PLANNER_MODULES_PLANEXP_COMBINED_EVALUATOR_H
