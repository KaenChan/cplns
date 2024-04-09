#pragma once
#include "PathTable.h"
#include "SolutionShared.h"
#include "Threading.h"
#include "common.h"

class ShareDataManager {
   public:
    ShareDataManager(int screen, bool use_share, bool use_share_wc,
                     vector<int> &group_list, int np, int np_init);

    int screen;
    int numGroupPP;
    int np;
    int np_init;
    vector<SolutionShared *> solutions;
    // int best_group_index{-1};
    std::atomic<int> best_group_index{-1};

    std::atomic<bool> interrupte_mem_out{false};

    vector<Path> g_agents_path;  // for the path in Agent class
    int g_num_of_colliding_pairs{MAX_COST};
    int g_sum_of_costs = MAX_COST;

    double time_phase_collision_start = 0;

    pthread_barrier_t barrier_stop_all_init_solver;
    pthread_barrier_t barrier_all_threads_restart1;
    pthread_barrier_t barrier_all_threads_restart2;

    spinlock groupLock;

    SolutionShared *get_current_best_group_solution();
    int getCurrentBestGroupId(vector<SolutionShared *> &solutions);
    void print(double mem_use_rate);

    void destory_barrier();
};
