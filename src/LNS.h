#pragma once
#include "BasicLNS.h"
#include "InitLNS.h"

// pibt related
#include "mapf.h"
#include "pibt.h"
#include "pibt_agent.h"
#include "pps.h"
#include "problem.h"
#include "simplegrid.h"
#include "winpibt.h"

#include "lacam3/include/lacam.hpp"
#include "lacam3/include/instance_la.hpp"

enum destroy_heuristic {
    RANDOMAGENTS,
    RANDOMWALK,
    INTERSECTION,
    DESTORY_COUNT
};

// TODO: adaptively change the neighbor size, that is,
// increase it if no progress is made for a while
// decrease it if replanning fails to find any solutions for several times

class LNS : public BasicLNS {
   public:
    vector<Agent> agents;
    double preprocessing_time = 0;
    double initial_solution_runtime = 0;
    int initial_sum_of_costs = -1;
    int sum_of_costs_lowerbound = -1;
    int sum_of_distances = -1;
    int restart_times = 0;

    bool stop_costlns_only_solver = false;

    string group_mode_phase_collision = "evenly";
    string group_mode_phase_cost = "one";
    bool all_run_next_if_one_solution_found = false;

    bool succ_phase_collision = false;

    int is_running = 0;

    int num_init_iteration = 0;

    LNS(const Instance& instance, double time_limit,
        const string& init_algo_name, const string& init_replan_algo_name,
        const string& replan_algo_name, const string& destory_name,
        int neighbor_size, int num_of_iterations, bool init_lns,
        const string& init_destory_name, int use_sipp, int screen,
        PIBTPPS_option pipp_option, SolutionShared* solution_shared,
        bool preprocess_LL_engines = true);
    ~LNS() { delete init_lns; }
    bool getInitialSolution();
    bool run_with_restart();
    bool run();
    void run_initial_solver();
    void run_collision_lns();
    bool regroup_solvers();
    void run_cost_lns();
    void validateSolution() const;
    void writeIterStatsToFile(const string& file_name) const;
    void writeResultToFile(const string& file_name) const;
    void writePathsToFile(const string& file_name) const;
    string getSolverName() const override {
        return "LNS(" + init_algo_name + ";" + replan_algo_name + ")";
    }
    virtual void setSolverInterrupt();
    virtual void unsetSolverInterrupt();

   public:
    InitLNS* init_lns = nullptr;
    string init_algo_name;
    string init_replan_algo_name;
    string replan_algo_name;
    bool use_init_lns;  // use LNS to find initial solutions
    destroy_heuristic destroy_strategy = RANDOMWALK;
    int max_num_of_iterations;
    string init_destory_name;
    PIBTPPS_option pipp_option;

    PathTable
        path_table;  // 1. stores the paths of all agents in a time-space table;
    // 2. avoid making copies of this variable as much as possible.
    unordered_set<int> tabu_list;  // used by randomwalk strategy
    list<int> intersections;

    bool runEECBS();
    bool runCBS();
    bool runPP();
    bool runPIBT();
    bool runPPS();
    bool runWinPIBT();

    bool runlacam();
    void callbackLacamResult(const lacam::Solution &solution, vector<int>& shuffled_agents);
    void updateLacamResult(const lacam::Solution &solution, vector<int>& shuffled_agents);

    MAPF preparePIBTProblem(vector<int>& shuffled_agents);
    void updatePIBTResult(const PIBT_Agents& A, vector<int>& shuffled_agents);

    void chooseDestroyHeuristicbyALNS();

    bool generateNeighborByRandomWalk();
    bool generateNeighborByIntersection();

    int findMostDelayedAgent();
    int findRandomAgent() const;
    void randomWalk(int agent_id, int start_location, int start_timestep,
                    set<int>& neighbor, int neighbor_size, int upperbound);
};
