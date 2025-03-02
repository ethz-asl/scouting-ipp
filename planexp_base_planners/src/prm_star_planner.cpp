//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//
#include "planexp_base_planners/prm_star_planner.h"

namespace planexp_base_planners {
  PRMStarPlanner::PRMStarPlanner(PlannerParams params) : PlannerBase(params) {
    set_derived_params();
  }

  void PRMStarPlanner::setup() {
    set_derived_params();
    build_graph();
  }

  void PRMStarPlanner::set_derived_params() {
    n_nodes_x_ = params_.px_x;
    n_nodes_y_ = params_.px_y;

    n_nodes_ = n_nodes_x_ * n_nodes_y_;

    neighbours_ = std::vector<std::vector<int>>(n_nodes_);
    edges_ = std::vector<std::vector<edge_descriptor>>(n_nodes_);
    explored_ = std::vector<CellStatus_t>(n_nodes_, CellStatus_t::UNEXPLORED);

    map_ = std::make_shared<Eigen::ArrayXXd>(params_.px_y, params_.px_x);
    map_->setConstant(params_.init_cost);

    spacing_ = params_.width / (n_nodes_x_+1);
    max_dist_ = spacing_ * sqrt(2.0);
    offset_x_ = (params_.width - (n_nodes_x_+1) * spacing_) / 2.0;
    offset_y_ = (params_.height - (n_nodes_y_+1) * spacing_) / 2.0;

    start_node_ = closest_xy(params_.start);
    goal_node_ = closest_xy(params_.goal);
  }

  ResultStatus PRMStarPlanner::plan(std::vector<Eigen::Vector2d> *path, double& cost) {
    cost = -1.0;

    std::lock_guard<std::mutex> guard(graph_mutex_);

    std::vector<UndirectedGraph::vertex_descriptor> p(num_vertices(graph_));
    std::vector<double> d(num_vertices(graph_));
    try {
      boost::astar_search_tree
          (graph_, xy2node_id(start_node_),
           distance_heuristic<UndirectedGraph, double>
               (xy2node_id(goal_node_), spacing_, n_nodes_y_, params_.min_cost),
           predecessor_map(make_iterator_property_map(p.begin(), get(boost::vertex_index, graph_))).
               distance_map(make_iterator_property_map(d.begin(), get(boost::vertex_index, graph_))).
               visitor(astar_goal_visitor<int>(xy2node_id(goal_node_))));

    } catch(found_goal fg) { // found a path to the goal
      std::list<int> shortest_path;
      for (int v = xy2node_id(goal_node_);; v = p[v]) {
        shortest_path.push_front(v);
        if (p[v] == v)
          break;
      }
      current_best_path_ = std::vector<int>(shortest_path.begin(), shortest_path.end());
      for (auto node:current_best_path_) path->push_back(node_id2pose(node));
      cost = d[xy2node_id(goal_node_)];
      if (check_termination()) return ResultStatus::TERMINAL_VALID;
      return ResultStatus::VALID;
    } catch (...) {
      return ResultStatus::INVALID;
    }
    return ResultStatus::TERMINAL_INVALID;
  }

  void PRMStarPlanner::store_path(const std::string file_name, const std::string path) {
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
      std::cout << "[PRMStarPlanner] store_path was requested but not setup properly! (" << path << "/" << file_name << ")" << std::endl;
      return;
    }
    std::string output_file_path = output_dir / (file_name + ".csv");
    std::ofstream output_file;
    output_file.open(output_file_path, std::ios::out | std::ios::trunc);
    output_file << "PositionX" << "," << "PositionY" << std::endl;
    output_file << "m" << "," << "m" << std::endl;
    for (auto node:current_best_path_) output_file << node_id2pose(node)[0] << "," << node_id2pose(node)[1] << std::endl;
    output_file.close();
  }

  void PRMStarPlanner::update_states(Eigen::Vector2i location, Eigen::ArrayXXi map_data) {
    std::lock_guard<std::mutex> guard(graph_mutex_);
    for (int i = 0; i<map_data.cols(); ++i) {
      for (int j = 0; j<map_data.rows(); ++j) {
        Eigen::Vector2i curr_loc = location + Eigen::Vector2i(i, j);
        if (curr_loc.x() < 0 || curr_loc.x() >= params_.px_x || curr_loc.y() < 0 || curr_loc.y() >= params_.px_y) continue;
        int curr_id = xy2node_id(curr_loc);
        if (is_new(curr_id)) {
          (*map_)(curr_loc.y(), curr_loc.x()) = map_data(j,i);
          if (idIsObstacle(map_data(j,i))) delete_connections(curr_id);
          else update_cost(curr_id);
          set_explored(curr_id);
        }
      }
    }
  }

  void PRMStarPlanner::update_unknown_cost(double new_cost) {
    std::cout << "[PRMStarPlanner] Updating unknown cost to: " << new_cost << std::endl;
    for (int id = 0; id < explored_.size(); ++id) {
      if (explored_[id] == CellStatus_t::UNEXPLORED) {
        Eigen::Vector2i location = node_id2xy(id);
        (*map_)(location.y(), location.x()) = new_cost;
        update_cost(id);
      }
    }
  }

  void PRMStarPlanner::build_graph() {
    for (int node_idx=0; node_idx<n_nodes_; ++node_idx){
      Eigen::Vector2i xy = node_id2xy(node_idx);
      const int box_size = std::ceil(max_dist_/spacing_);
      for (int i = std::max<int>(xy.x()-box_size, 0); i<=std::min<int>(xy.x()+box_size, n_nodes_x_-1); ++i){
        for (int j = std::max<int>(xy.y()-box_size, 0); j<=std::min<int>(xy.y()+box_size, n_nodes_y_-1); ++j){
          if((node_id2pose(node_idx)-node_id2pose(i*n_nodes_y_ + j)).norm() < max_dist_ && node_idx != i*n_nodes_y_ + j){
            neighbours_[node_idx].push_back(i*n_nodes_y_ + j);
            if (!std::binary_search(neighbours_[i*n_nodes_y_ + j].begin(), neighbours_[i*n_nodes_y_ + j].end(), node_idx)){
              if ((offset_y_ + spacing_ * j)*(1/spacing_) >= 480 || (offset_x_ + spacing_ * i)*(1/spacing_) >= 640 || (offset_y_ + spacing_ * xy.y())*(1/spacing_) >= 480 || (offset_x_ + spacing_ * xy.x())*(1/spacing_) >= 640){
                std::cout << "ERROR at location: (" << xy.x() << ", " << xy.y() << ")" << std::endl;
                std::cout << "i_x1: " << (offset_x_ + spacing_ * i)*(1/spacing_) << std::endl;
                std::cout << "i_y1: " << (offset_y_ + spacing_ * j)*(1/spacing_) << std::endl;
                std::cout << "i_x2: " << (offset_x_ + spacing_ * xy.x())*(1/spacing_) << std::endl;
                std::cout << "i_y2: " << (offset_y_ + spacing_ * xy.y())*(1/spacing_) << std::endl;
              }
              auto ed = boost::add_edge (node_idx, i*n_nodes_y_ + j, (node_id2pose(node_idx)-node_id2pose(i*n_nodes_y_ + j)).norm() * params_.init_cost, graph_);
              edges_[node_idx].push_back(ed.first);
              edges_[i*n_nodes_y_ + j].push_back(ed.first);
            }
          }
        }
      }
    }
  }

  void PRMStarPlanner::delete_connections(const int id) {
    std::vector<int> neighbours = neighbours_[id];
    for (int neighbour:neighbours) {
      int idx = find(neighbours_[neighbour].begin(), neighbours_[neighbour].end(), id) - neighbours_[neighbour].begin();
      neighbours_[neighbour].erase(neighbours_[neighbour].begin()+idx);
      edges_[neighbour].erase(edges_[neighbour].begin()+idx);
    }
    for (auto edge:edges_[id]) boost::remove_edge(edge, graph_);
    edges_[id].clear();
    neighbours_[id].clear();
  }

  void PRMStarPlanner::update_cost(const int id) {
    std::vector<int> neighbours = neighbours_[id];
    for (int neighbour:neighbours) {
      int idx = find(neighbours_[neighbour].begin(), neighbours_[neighbour].end(), id) - neighbours_[neighbour].begin();
      boost::put(boost::edge_weight_t(), graph_, edges_[neighbour][idx], compute_edge_cost(id, neighbour));
    }
  }

  double PRMStarPlanner::compute_edge_cost(const int start_id, const int end_id) {
    double dist = (node_id2pose(start_id)-node_id2pose(end_id)).norm();
    return dist * (get_cost(start_id) + get_cost(end_id)) / 2.0;
  }

  bool PRMStarPlanner::check_termination() {
    for (auto id: current_best_path_) if (explored_[id] == CellStatus_t::UNEXPLORED) return false;
    return true;
  }
}
