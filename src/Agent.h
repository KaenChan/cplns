#pragma once
#include "common.h"
#include "SpaceTimeAStar.h"
#include "SIPP.h"
#include "Threading.h"
#include "SippParaHD.h"


// 在共享通信时，当状态改变时，才需要进行copy，减少通信成本
struct PathState {
    int tid;
    int iter;

    PathState() {
        tid = -1;
        iter = -1;
    }

    bool equal(PathState &o) {
        return tid == o.tid && iter == o.iter;
    }

    void clear() {
        tid = -1;
        iter = -1;
    }
};

struct Agent
{
    int id;
    SingleAgentSolver* path_planner = nullptr; // start, goal, and heuristics are stored in the path planner
    Path path;
    PathState state;

    Agent(const Instance& instance, int id, int sipp, bool pre_process=true) : id(id)
    {
        if(sipp == 1)
            path_planner = new SIPP(instance, id, pre_process);
        else if(sipp == 2)
            path_planner = new SippParaHD(instance, id, pre_process);
        else
            path_planner = new SpaceTimeAStar(instance, id, pre_process);
    }
    ~Agent(){ delete path_planner; }

    int getNumOfDelays() const
    {
        return (int) path.size() - 1 - path_planner->p_my_heuristic->at(path_planner->start_location);
    }
};

struct Neighbor
{
    vector<int> agents;
    int sum_of_costs;
    int old_sum_of_costs;
    set<pair<int, int>> colliding_pairs;  // id1 < id2
    set<pair<int, int>> old_colliding_pairs;  // id1 < id2
    vector<Path> old_paths;
    vector<PathState> old_state;

    void clear() {
        sum_of_costs = 0;
        old_sum_of_costs = 0;
        agents.clear();
        old_paths.clear();
        colliding_pairs.clear();
        old_colliding_pairs.clear();
    }
};
