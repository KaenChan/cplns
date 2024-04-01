/*
 * fast collision checking, used in SUO and refinner
 */
#pragma once

#include "graph.hpp"
#include "instance_la.hpp"
#include "utils.hpp"

namespace lacam {

struct CollisionTable {
  // vertex, time, agents
  std::vector<std::vector<std::vector<int>>> body;
  std::vector<std::vector<int>> body_last;
  int collision_cnt;
  int N;

  CollisionTable(const InstanceLa *ins);
  ~CollisionTable();

  int getCollisionCost(const Vertex *v_from, const Vertex *v_to,
                       const int t_from);
  void enrollPath(const int i, Path &path);
  void clearPath(const int i, Path &path);
  void shrink();
};

}