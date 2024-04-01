#include "SolutionShared.h"

#include "common.h"

void SolutionShared::read_global_data(PathTable &path_table,
                                      vector<Agent> &agents) {
    pathTableUpdateLock.lock();
    for (int i = 0; i < agents.size(); i++) {
        agents[i].path = agents_path[i];
        agents[i].state = agents_path_state[i];
    }
    path_table.reset();
    for (const auto &agent : agents) {
        path_table.insertPath(agent.id, agent.path);
    }
    path_table.sum_of_costs = sum_of_costs;
    // assert(path_table.sum_of_costs == sum_of_costs);
    pathTableUpdateLock.unlock();
}

void SolutionShared::write_global_path(vector<PathState> &agents_path_state,
                                       vector<Agent> &agents) {
    if (agents_path.size() != agents.size()) {
        agents_path.resize(agents.size());
        agents_path_state.resize(agents.size());
    }
    for (int i = 0; i < agents.size(); i++) {
        if (numLocalSolver > 1 && agents_path_state[i].equal(agents[i].state))
            continue;
        // agents_path[i].clear();
        agents_path[i] = agents[i].path;
        agents_path_state[i] = agents[i].state;
        // for(auto o : agents[i].path)
        //     agents_path[i].emplace_back(PathEntry(o.location));
    }
    // printf("w cnt %d %d\n", cnt, agents.size());
}

void SolutionShared::do_share_phase_cost(int pe_id, PathTable &path_table_local,
                                     vector<Agent> &agents,
                                     vector<double> &destroy_weights) {
    num_iter_update_phase_cost++;
    // if(path_table_local.sum_of_costs < sum_of_costs &&
    // path_table_local.sum_of_costs > sum_of_costs - 10)
    //     if(!pathTableUpdateLock.tryLock())
    //         return;
    // else
    if (time_phase_cost_start == -1) time_phase_cost_start = getTime();
    runtime_phase_cost = getTime() - time_phase_cost_start;
    if (!use_share_phase_cost) {  // write
        pathTableUpdateLock.lock();
        if (path_table_local.sum_of_costs < sum_of_costs) {
            sum_of_costs = path_table_local.sum_of_costs;
            makespan = path_table_local.makespan;
            num_of_colliding_pairs = 0;
            // w
            write_global_path(agents_path_state, agents);
        }
        pathTableUpdateLock.unlock();
        return;
    }
    if (path_table_local.sum_of_costs < sum_of_costs) {
        pathTableUpdateLock.lock();
        // double check
        if (path_table_local.sum_of_costs < sum_of_costs) {
            if (screen > 1)
                std::cout << "    g " << myGroup << " pe " << pe_id << " w costs "
                        << path_table_local.sum_of_costs << " -> global "
                        << sum_of_costs << std::endl;
            // table_global = path_table_local.table; //当使用partialupdate时，不需要保存table
            goals_global = path_table_local.goals;
            destroy_weights_global = destroy_weights;
            sum_of_costs = path_table_local.sum_of_costs;
            makespan = path_table_local.makespan;
            num_of_colliding_pairs = 0;
            write_global_path(agents_path_state, agents);
        }
        pathTableUpdateLock.unlock();
    } else if (path_table_local.sum_of_costs > sum_of_costs) {
        if(goals_global.empty()) {
            printf("goals_global.empty()\n");
            exit(0);
        }
        // 查看哪些路径需要更新
        vector<int> changes;
        pathTableUpdateLock.lock();

        if (path_table_local.sum_of_costs > sum_of_costs) {
            for (int i = 0; i < agents.size(); i++) {
                if (agents_path_state[i].equal(agents[i].state)) continue;
                changes.push_back(i);
                // for update path_table
                if(path_table_local.sum_of_costs < MAX_COST)
                    path_table_local.deletePath(agents[i].id, agents[i].path);
            }

            if (screen > 1)
                std::cout << "    g " << myGroup << " pe " << pe_id << " r costs "
                        << path_table_local.sum_of_costs << " <- global "
                        << sum_of_costs << std::endl;
            // path_table_local.table = table_global;
            path_table_local.goals = goals_global;
            path_table_local.sum_of_costs = sum_of_costs;
            destroy_weights = destroy_weights_global;
            num_of_colliding_pairs = 0;
            assert(agents_path.size() == agents.size());

            for (auto i : changes) {
                // 读取global_path
                agents[i].path = agents_path[i];
                agents[i].state = agents_path_state[i];
            }

            pathTableUpdateLock.unlock();
        }

        // update the path_table
        for (auto a : changes) {
            path_table_local.insertPath(agents[a].id, agents[a].path);
        }
    }
}

void SolutionShared::do_share_phase_collision(int pe_id, PathTableWC &path_table_local,
                                     vector<Agent> &agents,
                                     vector<set<int>> &collision_graph,
                                     vector<double> &destroy_weights,
                                     bool force_share) {
    num_iter_update_phase_collision++;
    pathTableUpdateLock.lock();
    // if(!pathTableUpdateLock.tryLock())   // todo:
    // 当collision=0时,需要等待获得
    //     return;
    runtime_phase_collision = getTime() - time_phase_collision_start;
    if (!use_share_phase_collision && !force_share) {
        if (path_table_local.num_of_colliding_pairs < num_of_colliding_pairs ||
            (path_table_local.num_of_colliding_pairs ==
                 num_of_colliding_pairs &&
             path_table_local.sum_of_costs < sum_of_costs_wc)) {
            if (screen > 1)
                std::cout << "-->" 
                          << " g " << myGroup << " pe " << pe_id
                          << " w colliding_pairs " 
                          << path_table_local.num_of_colliding_pairs
                          << " -> global " << num_of_colliding_pairs
                          << std::endl;
            sum_of_costs_wc = path_table_local.sum_of_costs;
            makespan_wc = path_table_local.makespan;
            num_of_colliding_pairs = path_table_local.num_of_colliding_pairs;

            table_wc_global = path_table_local.table;
            collision_graph_global = collision_graph;
            destroy_weights_wc_global = destroy_weights;
            goals_wc_global = path_table_local.goals;
            if (agents_path.size() != agents.size()) {
                agents_path.resize(agents.size());
                agents_path_state.resize(agents.size());
            }
            for (int i = 0; i < agents.size(); i++)
                agents_path[i] = agents[i].path;
        }
        pathTableUpdateLock.unlock();
        return;
    }
    if (path_table_local.num_of_colliding_pairs < num_of_colliding_pairs ||
        (path_table_local.num_of_colliding_pairs == num_of_colliding_pairs &&
         path_table_local.sum_of_costs < sum_of_costs_wc)) {
        if (screen > 1)
            std::cout << "-->"
                      << " g " << myGroup << " pe " << pe_id
                      << " w colliding_pairs "
                      << path_table_local.num_of_colliding_pairs
                      << " -> global " << num_of_colliding_pairs << std::endl;
        table_wc_global = path_table_local.table;
        collision_graph_global = collision_graph;
        destroy_weights_wc_global = destroy_weights;
        goals_wc_global = path_table_local.goals;
        sum_of_costs_wc = path_table_local.sum_of_costs;
        makespan_wc = path_table_local.makespan;
        num_of_colliding_pairs = path_table_local.num_of_colliding_pairs;
        if (agents_path.size() != agents.size()) {
            agents_path.resize(agents.size());
            agents_path_state.resize(agents.size());
        }
        for (int i = 0; i < agents.size(); i++) agents_path[i] = agents[i].path;
    } else if (path_table_local.num_of_colliding_pairs >
                   num_of_colliding_pairs ||
               (path_table_local.num_of_colliding_pairs ==
                    num_of_colliding_pairs &&
                path_table_local.sum_of_costs > sum_of_costs_wc)) {
        assert(agents_path.size() == agents.size());
        if (screen > 1)
            std::cout << "-->"
                      << " g " << myGroup << " pe " << pe_id
                      << " r colliding_pairs "
                      << path_table_local.num_of_colliding_pairs
                      << " <- global " << num_of_colliding_pairs << std::endl;
        path_table_local.table = table_wc_global;
        collision_graph = collision_graph_global;
        destroy_weights = destroy_weights_wc_global;
        path_table_local.goals = goals_wc_global;
        path_table_local.sum_of_costs = sum_of_costs_wc;
        path_table_local.num_of_colliding_pairs = num_of_colliding_pairs;
        for (int i = 0; i < agents.size(); i++) {
            agents[i].path = agents_path[i];
            path_table_local.paths[i] = &agents[i].path;
        }
    }
    pathTableUpdateLock.unlock();
}

string SolutionShared::print() {
    stringstream sout;
    if (colliding_pairs_4_print == 0) num_of_init_remaining_agents = 0;
    float subopt = sum_of_costs * 1.0 / sum_of_distances;
    // printf("[%.2f] ", getTime());
    sout << "group " << myGroup << " colliding " << colliding_pairs_4_print
         << " costs " << sum_of_costs
         // << " " << last_cost_for_print - sum_of_costs
         << " costs_wc " << sum_of_costs_wc << " restart " << num_restart
         << " iter " << num_iter_update_phase_collision << " " << num_iter_update_phase_cost
        << " cfail " << std::round(num_of_consecutive_failure_rate * 10) / 10 
         << " np " << numLocalSolver 
        << " subopt " << std::round(subopt * 10000) / 10000 
        << " remain " << num_of_init_remaining_agents 
        << " runtime " << (int)runtime_phase_collision 
        << " + " << (int)runtime_phase_cost
        << " = " << (int)(getTime() - time_phase_collision_start);
    last_cost_for_print = sum_of_costs;
    return sout.str();
}

void SolutionShared::writeResultToFile(const string &file_name,
                                       const string &inst_name) const {
    string name = file_name;
    cout << "writeResultToFile " << name << endl;
    std::ifstream infile(name);
    bool exist = infile.good();
    infile.close();
    if (!exist) {
        ofstream addHeads(name);
        addHeads << "costs,cost_wc,colliding_pairs,inst_name" << endl;
        addHeads.close();
    }
    ofstream stats(name, std::ios::app);

    stats << sum_of_costs << "," << sum_of_costs_wc << ","
          << num_of_colliding_pairs << "," << inst_name << endl;

    stats.close();
}

void SolutionShared::reset() {
    phase_cost_need_restart = false;
    phase_collision_asynch_interrupt = false;
    phase_cost_asynch_interrupt = false;
    num_of_colliding_pairs = MAX_COST;
    colliding_pairs_4_print = MAX_COST;
    sum_of_costs = MAX_COST;
    makespan = MAX_COST;
    sum_of_costs_wc = MAX_COST;
    makespan_wc = MAX_COST;
    goals_global.clear();
    table_global.clear();
    agents_path.clear();
    destroy_weights_global.clear();
    agents_path_state.clear();

    table_wc_global.clear();
    goals_wc_global.clear();
    collision_graph_global.clear();
    destroy_weights_wc_global.clear();
}