#include "planner_modules_planexp/evaluator_updater/rrt_star_updater.h"

#include <string>

#include <active_3d_planning_core/module/module_factory.h>
#include <active_3d_planning_core/module/trajectory_evaluator.h>
#include <active_3d_planning_core/planner/planner_I.h>

namespace active_3d_planning {
namespace evaluator_updater {

// ConstrainedUpdater
ModuleFactoryRegistry::Registration<RRTStarUpdater>
      RRTStarUpdater::registration("RRTStarUpdater");

  RRTStarUpdater::RRTStarUpdater(PlannerI& planner)
    : EvaluatorUpdater(planner) {}

bool RRTStarUpdater::updateSegment(TrajectorySegment* segment) {
  // recursively update all segments from root to leaves (as in the planner)
  if (segment->parent) {
    if (segment->gain_terminal) return true;
    // Cannot update the root segment
    if (segment->gain > p_minimum_gain_ || rrt_path_has_changed_) {
      if ((planner_.getCurrentPosition() -
           segment->trajectory.back().position_W)
              .norm() <= p_update_range_ || rrt_path_has_changed_) {
        return following_updater_->updateSegment(segment);
      }
    }
  } else {
    if (last_rrt_path_id_ != map_->getPathMapId()) {
      rrt_path_has_changed_ = true;
      map_->updatePathMap();
    }
    else {
      rrt_path_has_changed_ = false;
    }
    last_rrt_path_id_ = map_->getPathMapId();
  }
  return true;
}

void RRTStarUpdater::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<double>(param_map, "minimum_gain", &p_minimum_gain_, -1e9);
  setParam<double>(param_map, "update_range", &p_update_range_, 1e9);

  // Create Following updater (default does nothing)
  std::string args;  // default args extends the parent namespace
  std::string param_ns = (*param_map)["param_namespace"];
  setParam<std::string>(param_map, "following_updater_args", &args,
                        param_ns + "/following_updater");
  following_updater_ = planner_.getFactory().createModule<EvaluatorUpdater>(
      args, planner_, verbose_modules_);

  // setup map
  map_ = dynamic_cast<map::MapPlanExp*>(&(planner_.getMap()));
  if (!map_) {
    planner_.printError(
        "'RRTStarEvaluator' requires a map of type 'MapPlanExp'!");
  }
}

}  // namespace evaluator_updater
}  // namespace active_3d_planning
