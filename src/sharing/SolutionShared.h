#pragma once
#include "Agent.h"
#include "PathTable.h"
#include "Threading.h"
#include "common.h"
#include "rw_lock.hpp"

class SolutionShared {
   public:
    SolutionShared(int screen, bool use_share, bool use_share_wc)
        : screen(screen),
          use_share_phase_cost(use_share),
          use_share_phase_collision(use_share_wc) {}

    // avoid making copies of this variable as much as possible
    // 用来validate solution
    const Instance* instance; 

    int screen = 0;
    bool use_share_phase_collision = 0;
    bool use_share_phase_cost = 1;
    int myGroup = 0;
    std::atomic<int> numLocalSolver{1};

    bool has_eecbs_solver = false;

    int sum_of_distances = MAX_COST;
    double time_phase_collision_start = 0;
    double time_phase_cost_start = 0;
    double runtime_phase_collision = 0;
    double runtime_phase_cost = 0;
    double num_of_consecutive_failure_rate = 0; // for print info

    std::atomic<int> num_restart{0};

    std::atomic<int> num_of_init_remaining_agents{MAX_COST};
    std::atomic<int> idForInitialSolutionMethod3{-1};
    std::atomic<int> nRemainForInitialSolutionMethod3{MAX_COST};
    std::atomic<int> numDoneForInitialSolutionMethod3{0};

    vector<int> method3_remains;
    vector<int> method3_select_agents;

    std::atomic<int> num_iter_update_phase_collision{8};
    std::atomic<int> num_iter_update_phase_cost{8};

    // bool asynch_interrupt = false;
    std::atomic<bool> phase_collision_asynch_interrupt{false};
    std::atomic<bool> phase_cost_asynch_interrupt{false};

    // for share solution in phase_collision
    std::atomic<bool> phase_collision_need_restart{false};
    std::atomic<bool> phase_cost_need_restart{false};

    // for path solution
    int makespan = MAX_COST;
    int sum_of_costs = MAX_COST;
    vector<int> goals_global;  // this stores the goal locatons of the paths:
                               // key is the location, while value is the
                               // timestep when the agent reaches the goal
    vector<vector<int>>
        table_global;  // this stores the collision-free paths, the value is the
                       // id of the agent. 2d:(location, time)
    vector<Path> agents_path;  // for the path in Agent class
    vector<double> destroy_weights_global;

    vector<PathState> agents_path_state;  // state of the path in Agent class

    // for path solution with collion
    int makespan_wc = MAX_COST;
    int sum_of_costs_wc = MAX_COST;
    std::atomic<int> num_of_colliding_pairs{MAX_COST};
    std::atomic<int> colliding_pairs_4_print{MAX_COST};
    // int num_of_colliding_pairs = MAX_COST;
    table_wc_t
        table_wc_global;  // this stores the paths with collision, the value is
                          // the id of the agent. 3d:(location, time, agent_id)
    vector<int> goals_wc_global;  // this stores the goal locatons of the paths:
                                  // key is the location, while value is the
                                  // timestep when the agent reaches the goal
    vector<set<int>> collision_graph_global;
    vector<double> destroy_weights_wc_global;

    ReadWriteLock rw_mutex;

    spinlock pathTableUpdateLock;

    // start barrier
    pthread_barrier_t barrier_phase_cost_start1;  
    pthread_barrier_t barrier_phase_cost_start2;  
    pthread_barrier_t barrier_phase_cost_start3;  
    // restart barrier
    pthread_barrier_t barrier_phase_cost_restart1;
    pthread_barrier_t barrier_phase_cost_restart2;

    pthread_barrier_t barrier_phase_collision_restart;
    pthread_barrier_t barrier_phase_collision_result;
    // 利用多个barrier来保证在restart时能够同步，如果一个barrier可能会有意外情况
    // 用std::condition_variable可能更好一些，但目前也能用，就不改了
    pthread_barrier_t barrier_phase_collision_result_31;  
    pthread_barrier_t barrier_phase_collision_result_32;
    pthread_barrier_t barrier_phase_collision_result_33;
    pthread_barrier_t barrier_phase_collision_result_34;

    int last_cost_for_print = 0;

    void read_global_data(PathTable &path_table, vector<Agent> &agents);
    void write_global_path(vector<PathState> &agents_path_state,
                           vector<Agent> &agents);
    void do_share_phase_cost(int pe_id, PathTable &path_table_local,
                         vector<Agent> &agents,
                         vector<double> &destroy_weights);
    void do_share_phase_collision(int pe_id, PathTableWC &path_table_local,
                         vector<Agent> &agents,
                         vector<set<int>> &collision_graph,
                         vector<double> &destroy_weights,
                         bool force_share = false);

    string print();
    void writeResultToFile(const string &file_name,
                           const string &inst_name) const;
    void reset();
};
