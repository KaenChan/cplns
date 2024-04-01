/*
 * heuristic definition
 */

#pragma once
#include "dist_table.hpp"
#include "graph.hpp"
#include "instance_la.hpp"

namespace lacam {

struct Heuristic {
  const InstanceLa *ins;
  DistTable *D;

  Heuristic(const InstanceLa *_ins, DistTable *_D);
  int get(const Config &C);
};

}