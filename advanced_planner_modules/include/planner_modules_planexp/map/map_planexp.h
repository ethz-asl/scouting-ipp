//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#ifndef ROS_WS_MAP_PLANEXP_H
#define ROS_WS_MAP_PLANEXP_H

#include <memory>
#include <string>
#include <vector>

#include <active_3d_planning_core/module/module_factory_registry.h>

#include "active_3d_planning_core/map/tsdf_map.h"
#include "map_server_planexp/map_server_planexp.h"

#define AM(x_) (x_)[1], (x_)[0]

namespace active_3d_planning {
  namespace map {

// My map representation
    class MapPlanExp : public TSDFMap {
    public:
      explicit MapPlanExp(PlannerI& planner);  // NOLINT

      // implement virtual methods
      void setupFromParamMap(Module::ParamMap* param_map) override;

      // check collision for a single pose
      bool isTraversable(const Eigen::Vector3d& position,
                         const Eigen::Quaterniond& orientation) override;

      // check whether point is part of the map
      bool isObserved(const Eigen::Vector3d& point) override;

      // get occupancy
      unsigned char getVoxelState(const Eigen::Vector3d& point) override;

      // get voxel size
      double getVoxelSize() override;

      // get the center of a voxel from input point
      bool getVoxelCenter(Eigen::Vector3d* center,
                          const Eigen::Vector3d& point) override;

      // get the stored distance
      double getVoxelDistance(const Eigen::Vector3d& point) override;

      // get the stored weight
      double getVoxelWeight(const Eigen::Vector3d& point) override;

      // get the maximum allowed weight (return 0 if using uncapped weights)
      double getMaximumWeight() override;

      // get the center of all voxels in a box of side length 2*halfsize+1 from input point
      bool getVoxelArea(std::vector<Eigen::Vector3d>* center_points,
                        const Eigen::Vector3d& point,
                        const int half_size);

      double getMeanWeight() const;

      double getMapExtent() const;

      double getMapWidth() const;

      double getMapHeight() const;

      Eigen::Vector3d getStartPosition() const;
      Eigen::Vector3d getGoalPosition() const;

      bool getClosestObservedAtPosition(const Eigen::Vector3d& position, Eigen::Vector3d& position_closest);

      // get the weight of the closest observed voxel
      double getVoxelClosestObservedWeight(const Eigen::Vector3d& point);

      bool isReachable(const Eigen::Vector3d& point);

      int getPathMapStatusAtPosition(const Eigen::Vector3d position);

      int getPathMapStatusAtLocation(const Eigen::Vector2i location);

      int getPathMapId();

      void updatePathMap();

      void targetReachedCallback() {map_server_->request_path();};

      template<typename T>
      void forwardParam(ros::NodeHandle& nh, Module::ParamMap* param_map, std::string name, T default_value);


    protected:
      static ModuleFactoryRegistry::Registration<MapPlanExp> registration;

      std::unique_ptr<MapServerPlanExp> map_server_;

      // cache constants
      double c_voxel_size_;
      double c_block_size_;
      double c_maximum_weight_;

    private:
      double height_{20.0};
      double width_{20.0};

      unsigned int vertical_units_{2000};
      unsigned int horizontal_units_{2000};

      //Eigen::ArrayXXd map_cost_;
      //Eigen::ArrayXXd map_uncertainty_;
      //Eigen::ArrayXXi map_state_;

      Eigen::ArrayXXi current_path_map_estimate_;

      Eigen::Vector2i position22pixelLocation(const Eigen::Vector2d& loc);
      Eigen::Vector2i position32pixelLocation(const Eigen::Vector3d& loc);
      Eigen::Vector2d pixelLocation2position(const Eigen::Vector2i& loc);
      Eigen::Vector2d pixelLocation2position(const Eigen::Vector2d& loc);

      Eigen::Vector2i getNthSpiralCoordinate(const Eigen::Vector2i& start, int &n);

      bool getDistanceAtPosition(const Eigen::Vector3d& position, double &distance);

      bool isEmpty(const Eigen::Vector3d& point);

      bool isOnMap3(const Eigen::Vector3d loc);

      bool isOnMap2(const Eigen::Vector2d loc);

    };

  }  // namespace map
}  // namespace active_3d_planning

#endif //ROS_WS_MAP_PLANEXP_H
