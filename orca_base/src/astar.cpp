#include "orca_base/astar.hpp"

namespace astar
{

  std::ostream &operator<<(std::ostream &os, CandidateNode const &c)
  {
    return os << "{marker: " << c.node << ", g_score: " << c.g_score << ", f_score: " << c.f_score << "}";
  }

  // Return the neighbors of this node
  std::vector<Neighbor> Graph::get_neighbors(node_type node)
  {
    std::vector<Neighbor> neighbors;

    for (auto edge : edges_) {
      if (edge.a == node) {
        neighbors.emplace_back(edge.b, edge.distance);
      } else if (edge.b == node) {
        neighbors.emplace_back(edge.a, edge.distance);
      }
    }

    return neighbors;
  }

  void Solver::print_open_set()
  {
    auto temp = open_set_;

    std::cout << "queue: ";
    while (!temp.empty()) {
      auto c = temp.top();
      temp.pop();
      std::cout << c << ", ";
    }
    std::cout << std::endl;
  }

  // Follow the parent links to reconstruct the shortest path
  void Solver::reconstruct_path(node_type node, std::vector<node_type> &path)
  {
    if (best_parents_.find(node) != best_parents_.end()) {
      reconstruct_path(best_parents_[node], path);
    }
    std::cout << node << ", ";
    path.push_back(node);
  }

  void Solver::reset()
  {
    // Clear everything
    best_g_score_.clear();
    best_parents_.clear();
    open_set_ = std::priority_queue<CandidateNode, std::vector<CandidateNode>, CandidateNode>();

    // Initialize best_g_score_ with ~infinity
    for (auto edge : graph_.edges_) {
      best_g_score_[edge.a] = std::numeric_limits<double>::max();
      best_g_score_[edge.b] = std::numeric_limits<double>::max();
    }
  }

  // Find the best path from start to destination, return true if successful
  bool Solver::find_shortest_path(node_type start, node_type destination, std::vector<node_type> &result)
  {
    reset();

    // Best distance to start node is always 0
    best_g_score_[start] = 0;

    // Add start node to open set
    open_set_.push(CandidateNode(start, 0, h_(start, destination)));

    while (!open_set_.empty()) {
      // print_open_set();

      // Pop the path with the best f_score
      auto current = open_set_.top();
      open_set_.pop();
      // std::cout << "pop " << current << std::endl;

      // Are we done?
      if (current.node == destination) {
        result.clear();
        std::cout << "reconstructed path: ";
        reconstruct_path(destination, result);
        std::cout << std::endl;
        return true;
      }

      // If this path to current.node is worse than the best one we've seen, drop it
      if (current.g_score > best_g_score_[current.node]) {
        continue;
      }

      // Loop through neighbors
      std::vector<Neighbor> neighbors = graph_.get_neighbors(current.node);
      // std::cout << "considering " << neighbors.size() << " neighbors" << std::endl;
      for (auto neighbor : neighbors) {
        // tentative_g_score is the distance from start through current to the neighbor
        double tentative_g_score = best_g_score_[current.node] + neighbor.distance;

        // Does this beat the current best path?
        if (tentative_g_score < best_g_score_[neighbor.node]) {
          // Yes! Remember this path
          best_g_score_[neighbor.node] = tentative_g_score;
          best_parents_[neighbor.node] = current.node;

          // Add the neighbor to the open set
          auto c = CandidateNode(neighbor.node, tentative_g_score, h_(neighbor.node, destination));
          // std::cout << "push " << c << std::endl;
          open_set_.push(c);
        }
      }
    }

    return false;
  }


} // namespace astar
