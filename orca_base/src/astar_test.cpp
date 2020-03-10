#include "orca_base/astar.hpp"

void astar_test()
{
  std::cout << "=== ASTAR TEST ===" << std::endl;

  using astar::Edge;

  std::vector<Edge> edges = std::vector<Edge>{
    Edge(0, 1, 10),
    Edge(0, 2, 15),
    Edge(1, 3, 10),
    Edge(1, 2, 5),
    Edge(2, 3, 15)
  };

  // A* heuristic: how far from node to the destination?
  auto heuristic = [](astar::node_type a, astar::node_type b) -> double
  {
    if (a == 0) return 18;
    else if (a == 1) return 9;
    else if (a == 2) return 12;
    else if (a == 3) return 0;
    else
      assert(false);

  };

  auto solver = astar::Solver(edges, heuristic);

  std::vector<int> path;
  std::cout << (solver.find_shortest_path(0, 3, path) ? "success" : "failure") << std::endl;

  for (auto item : path) {
    std::cout << item << ", ";
  }
  std::cout << std::endl;
}
