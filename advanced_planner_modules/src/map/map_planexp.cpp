//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#include "planner_modules_planexp/map/map_planexp.h"

//#include <voxblox_ros/ros_params.h>

#include "active_3d_planning_core/data/system_constraints.h"

namespace active_3d_planning {
  namespace map {

    ModuleFactoryRegistry::Registration<MapPlanExp> MapPlanExp::registration(
        "MapPlanExp");

    MapPlanExp::MapPlanExp(PlannerI& planner) : TSDFMap(planner) {}

    void MapPlanExp::setupFromParamMap(Module::ParamMap* param_map) {
      // create an esdf server
      //TODO: Check if the ros code below is necessary and where to get the parameters from;
      ros::NodeHandle nh("");
      ros::NodeHandle nh_private("~");

      forwardParam(nh_private, param_map, "use_uncertainty_map", false);
      forwardParam(nh_private, param_map, "use_height_map", false);
      forwardParam(nh_private, param_map, "compute_reachability", true);
      forwardParam(nh_private, param_map, "compute_closest_observed", false);
      forwardParam(nh_private, param_map, "compute_path", false);
      forwardParam(nh_private, param_map, "unknown_init_cost", std::string("min"));
      forwardParam(nh_private, param_map, "planner_type", std::string("prm_star"));
      map_server_.reset(new MapServerPlanExp(nh, nh_private));

      // cache constants
      setParam<unsigned int>(param_map, "horizontal_units", &horizontal_units_, map_server_->getMapResolutionH());
      setParam<unsigned int>(param_map, "vertical_units", &vertical_units_, map_server_->getMapResolutionV());
      setParam<double>(param_map, "width", &width_, map_server_->getMapWidth());
      setParam<double>(param_map, "height", &height_, map_server_->getMapHeight());
      setParam<double>(param_map, "max_cost", &c_maximum_weight_, map_server_->getMaxCost());
      bool compute_path = false;
      setParam<bool>(param_map, "compute_path", &compute_path, compute_path);

      c_voxel_size_ = width_/horizontal_units_;
      c_block_size_ = width_/horizontal_units_;
      if (compute_path) updatePathMap();
    }

    bool MapPlanExp::isTraversable(const Eigen::Vector3d& position,
                                   const Eigen::Quaterniond& orientation) {
      /*double distance = 0.0;
      if (getDistanceAtPosition(position, distance)) {
        // This means the voxel is observed
        return (distance > planner_.getSystemConstraints().collision_radius);
      }
      return false;*/
      return isEmpty(position);
    }

    bool MapPlanExp::isObserved(const Eigen::Vector3d& point) {
      return MapPlanExp::UNKNOWN != map_server_->getStatusAtLocation(point.head(2));
    }

// get occupancy
    unsigned char MapPlanExp::getVoxelState(const Eigen::Vector3d& point) {
      return map_server_->getStatusAtLocation(point.head(2));
    }

// get voxel size
    double MapPlanExp::getVoxelSize() { return c_voxel_size_; }

// get the center of a voxel from input point
    bool MapPlanExp::getVoxelCenter(Eigen::Vector3d* center,
                                    const Eigen::Vector3d& point) {
      Eigen::Vector2i loc = position32pixelLocation(point);
      Eigen::Vector2d center_loc(loc[0] + 0.5, loc[1] + 0.5);
      center->head(2) = pixelLocation2position(center_loc);
      center->z() = 0;
      return isOnMap2(center_loc);
    }
    // get the center of a voxel from input point
    bool MapPlanExp::getVoxelArea(std::vector<Eigen::Vector3d>* center_points,
                                     const Eigen::Vector3d& point,
                                     const int half_size) {
      Eigen::Vector3d voxel_center, new_voxel;
      Eigen::Vector2d new_voxel_loc;
      Eigen::Vector2i center_voxel_loc;
      if (!getVoxelCenter(&voxel_center,point)) return false;
      center_points->push_back(voxel_center);
      center_voxel_loc = position32pixelLocation(voxel_center);
      for (int i = -half_size; i<=half_size; ++i) {
        for (int j = -half_size; j<=half_size; ++j) {
          new_voxel_loc[0] = center_voxel_loc[0] + i;
          new_voxel_loc[1] = center_voxel_loc[1] + j;
          //new_voxel_loc[2] = 0;
          new_voxel.head(2) = pixelLocation2position(new_voxel_loc);
          new_voxel[2] = 0.0;
          if (isOnMap2( new_voxel_loc)) center_points->push_back(new_voxel);
        }
      }
      return true;
    }

// get the stored TSDF distance
    double MapPlanExp::getVoxelDistance(const Eigen::Vector3d& point) {
      std::cout << "ERROR" << std::endl;
      return 0.0;
    }

// get the stored weight
    double MapPlanExp::getVoxelWeight(const Eigen::Vector3d& point) {
      return map_server_->getCostAtLocation(point.head(2));
    }

// get the maximum allowed weight (return 0 if using uncapped weights)
    double MapPlanExp::getMaximumWeight() { return std::max(c_maximum_weight_, map_server_->getMaxCost()); }

    double MapPlanExp::getMeanWeight() const { return map_server_->getMeanCost(); }

    double MapPlanExp::getMapExtent() const { return width_ * height_; }

    double MapPlanExp::getMapWidth() const { return width_; }

    double MapPlanExp::getMapHeight() const { return height_; }

    Eigen::Vector3d MapPlanExp::getStartPosition() const { return map_server_->getStartPosition(); }

    Eigen::Vector3d MapPlanExp::getGoalPosition() const { return map_server_->getGoalPosition(); }

    bool MapPlanExp::isEmpty(const Eigen::Vector3d& point) {
      return MSR_FREE == map_server_->getStatusAtLocation(point.head(2));
    }

    Eigen::Vector2i MapPlanExp::position22pixelLocation(const Eigen::Vector2d& loc) {
      return Eigen::Vector2i((int)(loc[0] * (1/c_voxel_size_)), (int)(loc[1] * (1/c_voxel_size_)));
    }
    Eigen::Vector2i MapPlanExp::position32pixelLocation(const Eigen::Vector3d& loc) {
      return Eigen::Vector2i((int)(loc[0] * (1/c_voxel_size_)), (int)(loc[1] * (1/c_voxel_size_)));
    }
    Eigen::Vector2d MapPlanExp::pixelLocation2position(const Eigen::Vector2i& loc) {
      return Eigen::Vector2d(((double)loc[0] * c_voxel_size_), ((double)loc[1] * c_voxel_size_));
    }
    Eigen::Vector2d MapPlanExp::pixelLocation2position(const Eigen::Vector2d& loc) {
      return Eigen::Vector2d(((double)loc[0] * c_voxel_size_), ((double)loc[1] * c_voxel_size_));
    }

    bool MapPlanExp::getDistanceAtPosition(const Eigen::Vector3d& position, double &distance) {
      if (!isObserved(position)) return false;
      if (!isEmpty(position)) {
        distance = 0.0;
        return true;
      }
      int n = 1;
      double max_distance = 10.0;
      double mpp = c_voxel_size_;
      while (n<=(max_distance/mpp)){
        distance = n * mpp;
        for (int i = 0; i <= n; ++i) {
          for (int j = 0; j <= n; ++j) {
            if (!((i*i + j*j < (n-1)*(n-1))||(i*i + j*j > n*n))) {
              if ((isEmpty(Eigen::Vector3d(position[0] + i*mpp, position[1] + j*mpp, 0.0)) != MSR_FREE) ||
                  (isEmpty(Eigen::Vector3d(position[0] + i*mpp, position[1] - j*mpp, 0.0)) != MSR_FREE) ||
                  (isEmpty(Eigen::Vector3d(position[0] - i*mpp, position[1] - j*mpp, 0.0)) != MSR_FREE) ||
                  (isEmpty(Eigen::Vector3d(position[0] - i*mpp, position[1] + j*mpp, 0.0)) != MSR_FREE)) return true;
            }
          }
        }
      }
      distance = 0.0;
      return false;
    }

    bool MapPlanExp::getClosestObservedAtPosition(const Eigen::Vector3d& position, Eigen::Vector3d& position_closest) {
      getVoxelCenter(&position_closest, position);
      if (isObserved(position)) return true;

      int n = 1;
      Eigen::Vector3d pos_temp = position_closest;
      double max_distance = 10.0; //TODO: Set from parameter file
      double mpp = c_voxel_size_;
      while (n<=(max_distance/mpp)){
        for (int i = 0; i <= n; ++i) {
          for (int j = 0; j <= n; ++j) {
            if (!((i*i + j*j < (n-1)*(n-1))||(i*i + j*j > n*n))) {

              position_closest = pos_temp  +  Eigen::Vector3d(i*mpp, j*mpp, 0.0);
              Eigen::Vector2i test = position32pixelLocation(position_closest);
              if (isOnMap2(position32pixelLocation(position_closest).cast<double>()))
                if (isObserved(position_closest))
                  return true;
              position_closest = pos_temp  +  Eigen::Vector3d(i*mpp, -j*mpp, 0.0);
              if (isOnMap2(position32pixelLocation(position_closest).cast<double>()))
                if (isObserved(position_closest))
                  return true;
              position_closest = pos_temp  +  Eigen::Vector3d(-i*mpp, j*mpp, 0.0);
              if (isOnMap2(position32pixelLocation(position_closest).cast<double>()))
                if (isObserved(position_closest))
                  return true;
              position_closest = pos_temp  +  Eigen::Vector3d(-i*mpp, -j*mpp, 0.0);
              if (isOnMap2(position32pixelLocation(position_closest).cast<double>()))
                if (isObserved(position_closest))
                  return true;
            }
          }
        }
      }
      position_closest = pos_temp;
      return false;
    }

    double MapPlanExp::getVoxelClosestObservedWeight(const Eigen::Vector3d& point) {
      return map_server_->getClosestCostAtLocation(point.head(2));
    }

    bool MapPlanExp::isReachable(const Eigen::Vector3d& point) {
      return MSR_REACHABLE == (map_server_->getReachabilityAtLocation(point.head(2)));
    }

    bool MapPlanExp::isOnMap3(const Eigen::Vector3d loc) {
      return (0 <= (int)loc[0]) && (0 <= (int)loc[1]) && (horizontal_units_ > (int)loc[0]) && (vertical_units_ > (int)loc[1]);
    }

    bool MapPlanExp::isOnMap2(const Eigen::Vector2d loc) {
      return (0 <= (int)loc[0]) && (0 <= (int)loc[1]) && (horizontal_units_ > (int)loc[0]) && (vertical_units_ > (int)loc[1]);
    }

    int MapPlanExp::getPathMapStatusAtPosition(const Eigen::Vector3d position) {
      return current_path_map_estimate_(AM(position32pixelLocation(position)));
      //return map_server_->getCurrentPathMapStatusAtPosition(position);
    }

    int MapPlanExp::getPathMapStatusAtLocation(const Eigen::Vector2i location) {
      return current_path_map_estimate_(AM(location));
      //return map_server_->getCurrentPathMapStatusAtLocation(location);
    }

    int MapPlanExp::getPathMapId() {
      return map_server_->getCurrentPathMapId();
    }

    void MapPlanExp::updatePathMap() {
      current_path_map_estimate_ = map_server_->getCurrentPathMap();
    }

    template<typename T>
    void MapPlanExp::forwardParam(ros::NodeHandle& nh, Module::ParamMap* param_map, std::string name, T default_value) {
      T storage;
      setParam<T>(param_map, name, &storage, default_value);
      nh.setParam(name, storage);
    }
  }  // namespace map
}  // namespace active_3d_planning
