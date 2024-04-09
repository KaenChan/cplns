#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "instance_la.hpp"
#include "planner.hpp"
#include "post_processing.hpp"
#include "sipp.hpp"
#include "utils.hpp"

lacam::Solution
lacam_solve(const lacam::InstanceLa &ins, const int verbose = 0,
            const lacam::Deadline *deadline = nullptr, int seed = 0,
            lacam::DistTable *D = nullptr,
            std::function<int(lacam::Solution &)> func = nullptr,
            std::atomic<bool> *interrupt = nullptr, 
            int* is_running = nullptr);
