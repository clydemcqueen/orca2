#ifndef ORCA_BASE_ASTAR_HPP
#define ORCA_BASE_ASTAR_HPP

#include <vector>
#include <cassert>
#include <iostream>
#include <queue>
#include <map>
#include <limits>
#include <functional>

namespace astar
{

  // Nodes
  using node_type = int;

  // Edges
  struct Edge
  {
    node_type a;
    node_type b;

    // Distance between a and b, in either direction
    double distance;

    Edge(node_type _a, node_type _b, double _distance) : a{_a}, b{_b}, distance{_distance}
    {}
  };

  // Used for get_neighbors
  struct Neighbor
  {
    node_type node;

    // Distance to this neighbor
    double distance;

    Neighbor(node_type _node, double _distance) : node{_node}, distance{_distance}
    {}
  };

  // Graph
  struct Graph
  {
    std::vector<Edge> edges_;

    explicit Graph(std::vector<Edge> graph) : edges_{std::move(graph)}
    {}

    std::vector<Neighbor> get_neighbors(node_type node);
  };

  // Nodes in the open_set_ contain additional state
  struct CandidateNode
  {
    node_type node;

    // Cost of cheapest path from start to this node
    double g_score;

    // Heuristic for cost of cheapest path from start through this node to the destination
    // f_score = g_score + h_(this)
    double f_score;

    // Must be default constructible to be used in a priority queue
    CandidateNode() = default;

    // Useful constructor
    CandidateNode(node_type _node, double _g_score, double h) : node{_node}, g_score{_g_score}, f_score{_g_score + h}
    {}

    // Priority queue uses value operator as comparator
    bool operator()(const CandidateNode &a, const CandidateNode &b)
    {
      return a.f_score > b.f_score;
    }
  };

  std::ostream &operator<<(std::ostream &os, CandidateNode const &c);

  // Find the shortest path in a graph
  class Solver
  {
    Graph graph_;

    // Heuristic function provides a rough guess of distance between 2 nodes, must be < than actual distance
    using HeuristicFn = std::function<double(node_type a, node_type b)>;

    HeuristicFn h_;

    // Best g_score from start to a node
    std::map<node_type, double> best_g_score_;

    // Parent nodes, used to reconstruct the path at the end
    std::map<node_type, node_type> best_parents_;

    // Set of candidate nodes
    std::priority_queue<CandidateNode, std::vector<CandidateNode>, CandidateNode> open_set_;

    void print_open_set();

    void reconstruct_path(node_type node, std::vector<node_type> &path);

    void reset();

  public:

    explicit Solver(std::vector<Edge> edges, HeuristicFn h) : graph_{Graph(std::move(edges))}, h_{std::move(h)}
    {}

    // Find the best path from start to destination, return true if successful
    bool find_shortest_path(node_type start, node_type destination, std::vector<node_type> &result);
  };

} // namespace astar

#endif //ORCA_BASE_ASTAR_HPP
