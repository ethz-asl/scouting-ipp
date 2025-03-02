//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//
#ifndef ROS_WS_SENSOR_MODEL_PLANEXP_H_
#define ROS_WS_SENSOR_MODEL_PLANEXP_H_

#include <string>
#include <vector>

#include "active_3d_planning_core/data/trajectory_segment.h"
#include "active_3d_planning_core/data/visualization_markers.h"
#include "active_3d_planning_core/module/sensor_model/sensor_model.h"

namespace active_3d_planning {
  namespace sensor_model {

// Base class for camera models / Utility class that finds visible voxels.
// Available for all trajectory generators. (improve performance here.)
    class SensorModelPlanExp : public SensorModel {
    public:
      explicit SensorModelPlanExp(PlannerI& planner);  // NOLINT

      virtual ~SensorModelPlanExp() = default;

      // Return the voxel centers of all visible voxels for a camera model pointing
      // in x-direction
      virtual bool getVisibleVoxels(std::vector<Eigen::Vector3d>* result,
                                    const Eigen::Vector3d& position,
                                    const Eigen::Quaterniond& orientation) = 0;
      /*bool getVisibleVoxels(std::vector<Eigen::Vector3d>* result,
                            const Eigen::Vector3d& position,
                            const Eigen::Quaterniond& orientation);*/

      // Return the voxel centers of all visible voxels, sampling camera poses from
      // a trajectory segment
      bool getVisibleVoxelsFromTrajectory(
          std::vector<Eigen::Vector3d>* result,
          const TrajectorySegment& traj_in) override;

      // Display camera view bounds
      void visualizeSensorView(VisualizationMarkers* markers,
                               const TrajectorySegment& traj_in) override;

      void setupFromParamMap(Module::ParamMap* param_map) override;

      bool checkParamsValid(std::string* error_message) override;

    protected:
      // parameters
      double p_ray_length_;  // params for camera model
      double p_focal_length_;
      int p_resolution_x_;
      int p_resolution_y_;
      double p_sampling_time_;  // sample camera poses from segment, use 0 for last
      // only
      double p_pixel_size_;

      // constants
      double c_field_of_view_x_;
      double c_field_of_view_y_;

      // For performance timing of different implementations
      bool p_test_;
      std::vector<double> time_count_;

      // methods
      // return the indices of the trajectory points of traj_in for which a view is
      // needed
      void sampleViewpoints(std::vector<int>* result,
                            const TrajectorySegment& traj_in);

      void visualizeSingleView(VisualizationMarkers* markers,
                               const Eigen::Vector3d& position,
                               const Eigen::Quaterniond& orientation);

      // get the direction vector for camera pointing in x_direction at pixel with
      // relative x, y position [0, 1]
      void getDirectionVector(Eigen::Vector3d* result, double relative_x,
                              double relative_y);
    };

  }  // namespace sensor_model
}  // namespace active_3d_planning
#endif  // ROS_WS_SENSOR_MODEL_PLANEXP_H_