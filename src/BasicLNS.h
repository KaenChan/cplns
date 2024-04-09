#pragma once
#include "common.h"
#include "Threading.h"
#include "Agent.h"
#include "SolutionShared.h"
#include "ShareDataManager.h"

class BasicLNS
{
public:
    int myId = 0;
    int myGroup = 0;
    int threadId = 0;
    int numLocalSolver = 1;
    int numGroupPP = 1;
    bool use_sync_mode_in_iteration = false;
    // bool use_same_point_phase_collision = false;

    // portfolio parameter
    float single_agent_solver_f_w = 1;
    int lacam_pibt_num = 1;
    int lacam_star = 0;

    bool test_speed_mode = false;

    bool run_costlns_only = false;
    
    double runtime_share_data = 0;

    // statistics
    int num_of_failures = 0; // #replanning that fails to find any solutions
    int num_of_consecutive_failures = 0; // #replanning that fails to find any solutions
    int num_of_iteration = 0; // #replanning that fails to find any solutions
    int num_of_ll_search = 0; 
    list<IterationStats> iteration_stats; //stats about each iteration
    int num_of_restart = -1;
    double runtime = 0;
    double average_group_size = -1;
    int sum_of_costs = 0;

    BasicLNS(const Instance &instance, double time_limit, int neighbor_size,
             int screen);
    virtual string getSolverName() const = 0;
    
	virtual void setSolverInterrupt() = 0;
	virtual void unsetSolverInterrupt() = 0;

    void setPathTableShared(PathTable *path_table);
    //由每个solver执行完一次迭代后进行调用
    void updatePathTableShared();

    // 当前线程的共享数据
    SolutionShared *solution_shared = nullptr;

    // 所有的共享数据
    ShareDataManager *share_manager;

    double start_time;

protected:
    // input params
    const Instance& instance; // avoid making copies of this variable as much as possible
    double time_limit;
    double replan_time_limit; // time limit for replanning
    int neighbor_size;
    int screen;

    // adaptive LNS
    bool ALNS = false;
    double decay_factor = -1;
    double reaction_factor = -1;
    vector<double> destroy_weights;
    vector<double> min_destroy_weights;
    int selected_neighbor;

    vector<int> agent_hit_weights;

    // helper variables
    Neighbor neighbor;

    void rouletteWheel();
};

