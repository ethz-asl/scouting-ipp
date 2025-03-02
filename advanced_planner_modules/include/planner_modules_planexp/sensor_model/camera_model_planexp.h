//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//
#ifndef ROS_WS_CAMERA_MODEL_PLANEXP_H_
#define ROS_WS_CAMERA_MODEL_PLANEXP_H_

#include <vector>

#include "active_3d_planning_core/data/trajectory_segment.h"
#include "planner_modules_planexp/sensor_model/sensor_model_planexp.h"
#include "planner_modules_planexp/map/map_planexp.h"
#include "active_3d_planning_core/tools/defaults.h"

namespace active_3d_planning {
  namespace sensor_model {

    class CameraModelPlanExp : public SensorModelPlanExp {
    public:
      explicit CameraModelPlanExp(PlannerI& planner);  // NOLINT

      virtual ~CameraModelPlanExp() = default;

      // Override virutal functions
      bool getVisibleVoxels(std::vector<Eigen::Vector3d>* result,
                            const Eigen::Vector3d& position,
                            const Eigen::Quaterniond& orientation) override;

      void setupFromParamMap(Module::ParamMap* param_map) override;

      int getHalfFov() const { return half_fov_; }

    protected:
      static ModuleFactoryRegistry::Registration<CameraModelPlanExp> registration;

      // params
      int half_fov_ = 0; // half side length of the observable square
    };

  }  // namespace sensor_model
}  // namespace active_3d_planning
#endif  // ROS_WS_CAMERA_MODEL_PLANEXP_H_