#ifndef PLANNER_MODULES_PLANEXP_EVALUATOR_UPDATER_RRT_STAR_UPDATER_H_
#define PLANNER_MODULES_PLANEXP_EVALUATOR_UPDATER_RRT_STAR_UPDATER_H_

#include <memory>

#include "active_3d_planning_core/module/module_factory.h"
#include "active_3d_planning_core/module/trajectory_evaluator.h"
#include "planner_modules_planexp/map/map_planexp.h"

namespace active_3d_planning {
namespace evaluator_updater {

// Update gain, only if conditions are met (min gain, nearby)
class RRTStarUpdater : public EvaluatorUpdater {
 public:
  explicit RRTStarUpdater(PlannerI& planner);  // NOLINT

  bool updateSegment(TrajectorySegment* segment) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<RRTStarUpdater> registration;

  // params
  double p_minimum_gain_;
  double p_update_range_;
  std::unique_ptr<EvaluatorUpdater> following_updater_;

  int last_rrt_path_id_{0};
  bool rrt_path_has_changed_{true};
  map::MapPlanExp* map_;
};

}  // namespace evaluator_updater
}  // namespace active_3d_planning
#endif  // PLANNER_MODULES_PLANEXP_EVALUATOR_UPDATER_RRT_STAR_UPDATER_H_
