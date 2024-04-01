/*
 * implementation of PIBTa
 *
 * references:
 * Priority Inheritance with Backtracking for Iterative Multi-agent Path
 * Finding. Keisuke Okumura, Manao Machida, Xavier Défago & Yasumasa Tamura.
 * Artificial Intelligence (AIJ). 2022.
 */
#pragma once
#include "dist_table.hpp"
#include "graph.hpp"
#include "instance_la.hpp"
#include "scatter.hpp"
#include "utils.hpp"

namespace lacam {

struct PIBTa {
  const InstanceLa *ins;
  std::mt19937 MT;

  // solver utils
  const int N;  // number of agents
  const int V_size;
  DistTable *D;

  // specific to PIBTa
  const int NO_AGENT_LA;
  std::vector<int> occupied_now;                // for quick collision checking
  std::vector<int> occupied_next;               // for quick collision checking
  std::vector<std::array<Vertex *, 5>> C_next;  // next location candidates
  std::vector<float> tie_breakers;              // random values, used in PIBTa

  // swap, used in the LaCAM* paper
  bool flg_swap;

  // scatter
  Scatter *scatter;

  PIBTa(const InstanceLa *_ins, DistTable *_D, int seed = 0, bool _flg_swap = true,
       Scatter *_scatter = nullptr);
  ~PIBTa();

  bool set_new_config(const Config &Q_from, Config &Q_to,
                      const std::vector<int> &order);
  bool funcPIBT(const int i, const Config &Q_from, Config &Q_to);
  int is_swap_required_and_possible(const int ai, const Config &Q_from,
                                    Config &Q_to);
  bool is_swap_required(const int pusher, const int puller,
                        Vertex *v_pusher_origin, Vertex *v_puller_origin);
  bool is_swap_possible(Vertex *v_pusher_origin, Vertex *v_puller_origin);
};

}