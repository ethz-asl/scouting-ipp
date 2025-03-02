//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//
#include "planner_modules_planexp/sensor_model/camera_model_planexp.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace active_3d_planning {
  namespace sensor_model {

    ModuleFactoryRegistry::Registration<CameraModelPlanExp>
        CameraModelPlanExp::registration("CameraModelPlanExp");

    CameraModelPlanExp::CameraModelPlanExp(PlannerI& planner) : SensorModelPlanExp(planner) {}

    void CameraModelPlanExp::setupFromParamMap(Module::ParamMap* param_map) {
      SensorModelPlanExp::setupFromParamMap(param_map);
      setParam<int>(param_map, "half_fov", &half_fov_, 10);
    }

    bool CameraModelPlanExp::getVisibleVoxels(std::vector<Eigen::Vector3d>* result,
                                           const Eigen::Vector3d& position,
                                           const Eigen::Quaterniond& orientation) {
      dynamic_cast<map::MapPlanExp*>(map_)->getVoxelArea(result, position, half_fov_);
      return true;
    }

  }  // namespace sensor_model
}  // namespace active_3d_planning
