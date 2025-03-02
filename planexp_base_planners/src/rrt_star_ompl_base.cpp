//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//
#include "planexp_base_planners/rrt_star_ompl_base.h"

namespace planexp_base_planners {
  RRTStarPlannerBase::RRTStarPlannerBase(std::shared_ptr<Eigen::ArrayXXd> map,  planexp_base_planners::PlannerParams params): map_(map), params_(params){
    sample_distance_ = params_.width / (3.0 * params_.px_x);
    ompl::msg::LogLevel currentLogLevel = ompl::msg::getLogLevel();
    ompl::msg::setLogLevel(std::max(ompl::msg::LOG_ERROR, currentLogLevel));
  }

  void RRTStarPlannerBase::setup() {
    setupGeneralProblemDefinition();
    setupPlanner();
  }

  void RRTStarPlannerBase::plan(std::vector<Eigen::Vector2d>* path, double& cost) {
    solve();
    getResults(path, cost);
  }

  bool RRTStarPlannerBase::myValidityChecker::isValid(const ompl::base::State* state) const
  {
    Eigen::Vector2d spacing(width_ / (*map_).cols(), height_ / (*map_).rows());
    // We know we're working with a RealVectorStateSpace in this
    // example, so we downcast state into the specific type.
    const auto* state2D =
        state->as<ompl::base::RealVectorStateSpace::StateType>();

    // Extract the robot's (x,y) position from its state
    double x = state2D->values[0];
    double y = state2D->values[1];

    int pos_x = (int)(x / spacing[0]);
    int pos_y = (int)(y / spacing[1]);

    //return !idIsObstacle((*map_)(pos_y, pos_x));
    return ((*map_)(pos_y, pos_x)) != 0.0;
  }

  ompl::base::Cost RRTStarPlannerBase::myObjective::motionCost(const ompl::base::State *s1,
                                                           const ompl::base::State *s2) const
  {
    double total_cost = 0.0;
    Eigen::Vector2d spacing(width_ / (*map_).cols(), height_ / (*map_).rows());

    const ompl::base::RealVectorStateSpace::StateType* state12D =
        s1->as<ompl::base::RealVectorStateSpace::StateType>();
    const ompl::base::RealVectorStateSpace::StateType* state22D =
        s2->as<ompl::base::RealVectorStateSpace::StateType>();

    // Extract the robot's (x,y) position from its state
    Eigen::Vector2d pose1(state12D->values[0], state12D->values[1]);
    Eigen::Vector2d pose2(state22D->values[0], state22D->values[1]);

    double distance = si_->distance(s1, s2);
    int samples = (int)(distance/sample_dist_);
    if (samples > 0) {
    Eigen::Vector2d dpose = (pose2-pose1) / samples;

    double dt = distance / samples;

    for (int i = 0; i<samples; i++) {
    Eigen::Vector2d inter_pos = pose1 + i * dpose;

    int pos_x = (int)(inter_pos[0] / spacing[0]);
    int pos_y = (int)(inter_pos[1] / spacing[1]);

    total_cost += abs((*map_)(pos_y, pos_x) * dt);
    }
  } else {
    int pos1_x = (int)(pose1[0] / spacing[0]);
    int pos1_y = (int)(pose1[1] / spacing[1]);
    int pos2_x = (int)(pose2[0] / spacing[0]);
    int pos2_y = (int)(pose2[1] / spacing[1]);
    double cost1 = (*map_)(pos1_y, pos1_x);
    double cost2 = (*map_)(pos2_y, pos2_x);
    total_cost = distance * (cost1 + cost2) / 2;
  }

  return ompl::base::Cost(total_cost);
  }

  void RRTStarPlannerBase::setupGeneralProblemDefinition() {
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(2));

    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, 0);
    bounds.setHigh(0, params_.width);

    bounds.setLow(1, 0);
    bounds.setHigh(1, params_.height);
    space->setBounds(bounds);

    space->setLongestValidSegmentFraction((sample_distance_)/space->getMaximumExtent());
    space->allocStateSampler();


    space_information_ = std::make_shared<ompl::base::SpaceInformation>(space);
    ompl::base::StateValidityCheckerPtr validity_checker_ptr = getValidity(space_information_);
    space_information_->setStateValidityChecker(validity_checker_ptr);
    space_information_->setup();

    ompl::base::ScopedState<> start(space);
    start[0] = params_.start[0];
    start[1] = params_.start[1];

    ompl::base::ScopedState<> goal(space);
    goal[0] = params_.goal[0];
    goal[1] = params_.goal[1];

    problem_definition_ = (std::make_shared<ompl::base::ProblemDefinition>(space_information_));
    problem_definition_->setStartAndGoalStates(start, goal);

    ompl::base::OptimizationObjectivePtr objective_ptr = getObjective(space_information_);
    problem_definition_->setOptimizationObjective(objective_ptr);
  }

  void RRTStarPlannerBase::setupPlanner() {

    // Construct our optimizing planner using the RRTstar algorithm.
    rrt_star_planner_ = std::make_shared<ompl::geometric::RRTstar>(space_information_);

    // Set the problem instance for our planner to solve
    rrt_star_planner_->setProblemDefinition(problem_definition_);
    rrt_star_planner_->setGoalBias(params_.goal_bias);
    rrt_star_planner_->setRange(params_.step_range);
    rrt_star_planner_->setup();
  }

  void RRTStarPlannerBase::solve() {
    rrt_star_planner_->solve(ompl::base::timedPlannerTerminationCondition(params_.time_limit));
  }

  void RRTStarPlannerBase::solve(double solve_time) {
    rrt_star_planner_->solve(ompl::base::timedPlannerTerminationCondition(solve_time));
  }

  void RRTStarPlannerBase::getResults(std::vector<Eigen::Vector2d>* path, double& cost) {
    //rrt_star_planner_->getPlannerData(*planner_data_);
    ompl::base::PathPtr path_ptr = problem_definition_->getSolutionPath();
    (*path).clear();
    (*path) = pathPtr_to_vector(path_ptr);
    cost = problem_definition_->getSolutions().front().cost_.value();
    if (!problem_definition_->hasSolution() || !problem_definition_->hasExactSolution()) cost = -1.0;
    //return std::vector<Eigen::Vector2d>();
  }

  std::vector<Eigen::Vector2d> RRTStarPlannerBase::pathPtr_to_vector(ompl::base::PathPtr path_ptr)
  {
    ompl::geometric::PathGeometric path( dynamic_cast< const ompl::geometric::PathGeometric& >( *path_ptr));
    const std::vector< ompl::base::State* > &states = path.getStates();
    ompl::base::State *state;
    std::vector<Eigen::Vector2d> path_vector;

    for( size_t i = 0 ; i < states.size( ) ; ++i )
    {
      state = states[ i ]->as< ompl::base::State >( );

      Eigen::Vector2d state_vec;
      state_vec(0) = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
      state_vec(1) = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
      path_vector.push_back(state_vec);
    }
    return path_vector;
  }

  void RRTStarPlannerBase::store_path(const std::string file_name, const std::string path) {
    bool store_paths = false;
    fs::path output_dir;

    if (!path.empty()) {
      output_dir = fs::path(path);

      if (!fs::exists(output_dir)) {
        if (fs::exists(output_dir.parent_path())) {
          fs::create_directories(path);
          store_paths = true;
        } else {
          store_paths = false;
        }
      } else {
        store_paths = true;
      }
    }

    if (!store_paths) {
      std::cout << "[RRTStarPlanner] store_path was requested but not setup properly!" << std::endl;
      return;
    }
    std::string output_file_path = output_dir / (file_name + ".csv");
    std::ofstream output_file;
    output_file.open(output_file_path, std::ios::out | std::ios::trunc);
    output_file << "PositionX" << "," << "PositionY" << std::endl;
    output_file << "m" << "," << "m" << std::endl;
    for (auto point:pathPtr_to_vector(problem_definition_->getSolutionPath())) output_file << point[0] << "," << point[1] << std::endl;
    output_file.close();
  }
}
