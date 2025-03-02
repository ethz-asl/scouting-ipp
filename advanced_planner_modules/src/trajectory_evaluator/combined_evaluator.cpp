//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//
//TODO: WIP Has never been tested!
#include "planner_modules_planexp/trajectory_evaluator/combined_evaluator.h"

#include <string>
#include <vector>

#include "active_3d_planning_core/module/module_factory.h"
#include "active_3d_planning_core/planner/planner_I.h"

namespace active_3d_planning {
  namespace trajectory_evaluator {

// YawPlanningEvaluator
    CombinedEvaluator::CombinedEvaluator(PlannerI& planner)
        : TrajectoryEvaluator(planner) {}

    void CombinedEvaluator::setupFromParamMap(Module::ParamMap* param_map) {
      setParam<std::string>(param_map, "gain_combination_method", &p_gain_combination_method_, "sum");
      setParam<std::string>(param_map, "cost_combination_method", &p_cost_combination_method_, "none");

      // Register link for yaw planning udpaters
      planner_.getFactory().registerLinkableModule("CombinedEvaluator", this);

      // Create following evaluators
      std::string param_ns = (*param_map)["param_namespace"];
      std::string evaluator_string = param_ns + "/following_evaluator";
      int n_evaluator = 1;
      while ((*param_map).find( evaluator_string + std::to_string(n_evaluator) ) != (*param_map).end()) {
        std::string args;  // default args extends the parent namespace
        setParam<std::string>(param_map, "following_evaluator" + std::to_string(n_evaluator) + "_args", &args,
                              evaluator_string + std::to_string(n_evaluator)); //TODO: Check if this is correct!
        following_evaluators_.push_back(planner_.getFactory().createModule<TrajectoryEvaluator>(args, planner_,
                                                                    verbose_modules_));
        ++n_evaluator;
      }
      // setup parent
      TrajectoryEvaluator::setupFromParamMap(param_map);
    }

    bool CombinedEvaluator::checkParamsValid(std::string* error_message) {
      if (p_gain_combination_method_ != "sum" && p_gain_combination_method_ != "average" && p_gain_combination_method_ != "product") {
        *error_message = "gain_combination_method must be either \"sum\", \"product\" or \"average\"";
        return false;
      }
      else if (p_cost_combination_method_ != "sum" && p_cost_combination_method_ != "average" && p_cost_combination_method_ != "product" && p_cost_combination_method_ != "none") {
        *error_message = "cost_combination_method must be either \"none\", \"sum\", \"product\" or \"average\"";
        return false;
      }
      else if (following_evaluators_.size() == 0) {
        *error_message = "At least one following evaluator has to be specified!";
        return false;
      }
      return TrajectoryEvaluator::checkParamsValid(error_message);
    }

    bool CombinedEvaluator::computeGain(TrajectorySegment* traj_in) {
      double gain = 0.0;
      if (p_gain_combination_method_ == "product") gain = 1.0;
      bool result = true;
      for (int i = 0; i < following_evaluators_.size(); ++i) {
        result &= following_evaluators_[i]->computeGain(traj_in);
        if (p_gain_combination_method_ == "sum" || p_gain_combination_method_ == "average") gain += traj_in->gain;
        else if (p_gain_combination_method_ == "product") gain *= traj_in->gain;
      }
      if (p_gain_combination_method_ == "average") {
        gain /= following_evaluators_.size();
      }
      traj_in->gain = gain;
      return result;
      //traj_in->info.reset(info);//TODO: Find out if this is needed
    }

    bool CombinedEvaluator::computeCost(TrajectorySegment* traj_in) {
      double cost = 0.0;
      bool result = true;
      int n_evaluators = following_evaluators_.size();
      if (p_cost_combination_method_ == "none") n_evaluators = 1;
      for (int i = 0; i < n_evaluators; ++i) {
        traj_in->cost = 0.0;
        result &= following_evaluators_[i]->computeCost(traj_in);
        if (p_cost_combination_method_ == "sum" || p_cost_combination_method_ == "average") cost += traj_in->cost;
        else if (p_cost_combination_method_ == "product") cost *= traj_in->cost;
      }
      if (p_cost_combination_method_ == "average") {
        cost /= following_evaluators_.size();
      }
      traj_in->cost = cost;
      return result;
    }

    bool CombinedEvaluator::computeValue(TrajectorySegment* traj_in) {
      return following_evaluators_.front()->computeValue(traj_in);
    }

    int CombinedEvaluator::selectNextBest(TrajectorySegment* traj_in) {
      return following_evaluators_.front()->selectNextBest(traj_in);
    }

    bool CombinedEvaluator::updateSegment(TrajectorySegment* segment) {
      return following_evaluators_.front()->updateSegment(segment);
    }

    void CombinedEvaluator::visualizeTrajectoryValue(
        VisualizationMarkers* markers, const TrajectorySegment& trajectory) {
      following_evaluators_.front()->visualizeTrajectoryValue(markers, trajectory);
    }
  }  // namespace trajectory_evaluator
}  // namespace active_3d_planning
