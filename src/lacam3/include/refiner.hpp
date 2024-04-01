/*
 * Implementation of refiners
 *
 * references:
 * Iterative Refinement for Real-Time Multi-Robot Path Planning.
 * Keisuke Okumura, Yasumasa Tamura, and Xavier Défago.
 * In Proceedings of IEEE/RSJ International Conference on Intelligent Robots and
 * Systems (IROS). 2021.
 *
 * Anytime multi-agent path finding via large neighborhood search.
 * Jiaoyang Li, Zhe Chen, Daniel Harabor, P Stuckey, and Sven Koenig.
 * In Proceedings of International Joint Conference on Artificial Intelligence
 * (IJCAI). 2021.
 */

#pragma once

#include "collision_table.hpp"
#include "dist_table.hpp"
#include "graph.hpp"
#include "instance_la.hpp"
#include "metrics.hpp"
#include "sipp.hpp"
#include "translator.hpp"
#include "utils.hpp"

namespace lacam {

Solution refine(const InstanceLa *ins, const Deadline *deadline,
                const Solution &solution, DistTable *D, const int seed = 0,
                const int verbose = 0);
}
