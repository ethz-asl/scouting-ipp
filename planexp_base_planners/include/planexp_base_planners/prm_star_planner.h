//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#ifndef PLANEXP_BASE_PLANNERS_PRM_STAR_PLANNER_H
#define PLANEXP_BASE_PLANNERS_PRM_STAR_PLANNER_H

#include <memory>
#include <fstream>
#include <mutex>

#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <math.h>

#if __has_include(<filesystem>)
#include <filesystem>
  namespace fs = std::filesystem;
#elif __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
error "Missing the <filesystem> header."
#endif

#include "planexp_base_planners/base_planner.h"

typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;
typedef boost::adjacency_list<boost::listS, boost::vecS,boost::undirectedS,boost::no_property,EdgeWeightProperty> UndirectedGraph;
typedef boost::graph_traits<UndirectedGraph>::edge_iterator edge_iterator;
typedef boost::graph_traits<UndirectedGraph>::edge_descriptor edge_descriptor;

namespace planexp_base_planners {
  class PRMStarPlanner : public PlannerBase {
  public:
    explicit PRMStarPlanner(PlannerParams params);

    void setup() override;

    ResultStatus plan(std::vector<Eigen::Vector2d> *path, double& cost) override;

    void store_path(const std::string file_name, const std::string path) override;

    void update_states(Eigen::Vector2i location, Eigen::ArrayXXi map_data) override;

    void update_unknown_cost(double new_cost) override;

    PlannerType get_planner_type() {return PlannerType::PRM_STAR_PLANNER;}

  protected:
    std::shared_ptr<Eigen::ArrayXXd> map_;
    std::vector<CellStatus_t> explored_;

    int n_nodes_x_;
    int n_nodes_y_;
    int n_nodes_;

    double offset_x_;
    double offset_y_;
    double spacing_;

    UndirectedGraph graph_;
    std::mutex graph_mutex_;

    double max_dist_;

    Eigen::Vector2i start_node_;
    Eigen::Vector2i goal_node_;

    std::vector<std::vector<int>> neighbours_;
    std::vector<std::vector<edge_descriptor>> edges_;

    std::vector<int> current_best_path_;


    inline Eigen::Vector2i node_id2xy(const int id) {return Eigen::Vector2i(id/n_nodes_y_, id%n_nodes_y_ );};
    inline int xy2node_id(const Eigen::Vector2i xy) {return xy.x() * n_nodes_y_ + xy.y();};
    inline Eigen::Vector2d node_id2pose(const int id) {return Eigen::Vector2d(offset_x_ + spacing_ * (id/n_nodes_y_), offset_y_ + spacing_ * (id%n_nodes_y_ ));};
    inline Eigen::Vector2i closest_xy(const Eigen::Vector2d pos) {return Eigen::Vector2i(std::round((pos.x() - offset_x_)/ spacing_), std::round((pos.y() - offset_y_)/ spacing_));};

  private:
    void set_derived_params();
    void build_graph();
    void delete_connections(const int id);
    void update_cost(const int id);
    bool check_termination();
    double compute_edge_cost(const int start, const int end);
    inline bool is_new(const int id) {return explored_[id] == CellStatus_t::UNEXPLORED;};
    inline bool is_obstacle(const int id) {return (*map_)(node_id2xy(id)[1], node_id2xy(id)[0]) == 0.0;};
    inline bool idIsObstacle(const int id) {return (std::find(params_.obstacle_ids.begin(), params_.obstacle_ids.end(), id) != params_.obstacle_ids.end());};
    inline void set_explored(const int id) {explored_[id] = CellStatus_t::EXPLORED;};
    inline double get_cost(const int id) {return (*map_)(node_id2xy(id)[1], node_id2xy(id)[0]);};
  };

  struct found_goal {};

  template <class Graph, class CostType>
  class distance_heuristic : public boost::astar_heuristic<Graph, CostType>
  {
  public:
    distance_heuristic(int goal_id, double spacing, int n_nodes_y, CostType min_cost)
        : spacing_(spacing), n_nodes_y_(n_nodes_y), min_cost_(min_cost) {

      goal_ = node_id2xy(goal_id);
    }
    CostType operator()(int node_id)
    {
      Eigen::Vector2i dist = (goal_ - node_id2xy(node_id));
      return ::sqrt(static_cast<double>(dist.x() * dist.x() + dist.y() * dist.y())) * min_cost_ * spacing_;
    }
  private:
    Eigen::Vector2i goal_;
    double spacing_;
    int n_nodes_y_;
    double min_cost_;
    inline Eigen::Vector2i node_id2xy(const int id) {return Eigen::Vector2i(id/n_nodes_y_, id%n_nodes_y_ );};
  };

  // visitor that terminates when we find the goal
  template <class Vertex>
  class astar_goal_visitor : public boost::default_astar_visitor
  {
  public:
    astar_goal_visitor(Vertex goal) : goal_(goal) {
    }
    template <class Graph>
    void examine_vertex(Vertex u, Graph& g) {
      if(u == goal_){
        throw found_goal();
      }

    }
  private:
    Vertex goal_;
  };

}

#endif //PLANEXP_BASE_PLANNERS_PRM_STAR_PLANNER_H
