/*
 * instance definition
 */
#pragma once
#include <random>

#include "graph.hpp"
#include "utils.hpp"

namespace lacam {

struct InstanceLa {
  Graph *G;       // graph
  Config starts;  // initial configuration
  Config goals;   // goal configuration
  const uint N;   // number of agents
  bool delete_graph_after_used;

  InstanceLa(Graph *_G, const Config &_starts, const Config &_goals, uint _N);
  InstanceLa(const std::string &map_filename,
           const std::vector<int> &start_indexes,
           const std::vector<int> &goal_indexes);
  // for MAPF benchmark
  InstanceLa(const std::string &scen_filename, const std::string &map_filename,
           const int _N = 1);
  // random instance generation
  InstanceLa(const std::string &map_filename, const int _N = 1,
           const int seed = 0);
  ~InstanceLa();

  // simple feasibility check of instance
  bool is_valid(const int verbose = 0) const;
};

// solution: a sequence of configurations
using Solution = std::vector<Config>;

}
