#include <algorithm>
#include <queue>

#include "InitLNS.h"
#include "GCBS.h"
#include "PBS.h"
#include "common.h"
#include "defines.h"

extern int screen_const;

InitLNS::InitLNS(const Instance& instance, vector<Agent>& agents,
                 double time_limit, const string& replan_algo_name,
                 const string& init_destory_name, int neighbor_size, int screen)
    : BasicLNS(instance, time_limit, neighbor_size, screen),
      agents(agents),
      replan_algo_name(replan_algo_name),
      path_table(instance.map_size, agents.size()),
      collision_graph(agents.size()),
      goal_table(instance.map_size, -1) {
    replan_time_limit = time_limit;
    if (init_destory_name == "Adaptive") {
        ALNS = true;
        destroy_weights.assign(INIT_COUNT, 1);
        decay_factor = 0.05;
        reaction_factor = 0.05;
        // decay_factor = 0.01;
        // reaction_factor = 0.01;
    } else if (init_destory_name == "Target")
        init_destroy_strategy = TARGET_BASED;
    else if (init_destory_name == "Collision")
        init_destroy_strategy = COLLISION_BASED;
    else if (init_destory_name == "Random")
        init_destroy_strategy = RANDOM_BASED;
    else {
        cerr << "Init Destroy heuristic " << init_destory_name
             << " does not exists. " << endl;
        exit(-1);
    }

    for (auto& i : agents) {
        goal_table[i.path_planner->goal_location] = i.id;
    }
}

bool InitLNS::run() {
    start_time = getTime();
    num_of_colliding_pairs = MAX_COST;
    vector<Path*> paths(agents.size());
    bool need_break = false;
    while (runtime < time_limit && num_of_colliding_pairs > 0 && !need_break) {
        *is_running = 210;
        if (solution_shared->use_share_phase_collision) {
            if (myId == 0) solution_shared->num_restart++;
        } else
            solution_shared->num_restart++;
        num_of_restart++;

        simulated_annealing::reset_temp(param_simulated_annealing_T,
                                        param_sa_restart_coolfactor,
                                        param_sa_iteration_coolfactor);
        num_of_failures = 0;
        num_of_iteration = 1;
        num_of_consecutive_failures = 0;

        // if (num_of_restart == 0 || !param_sa_restart_resume) 
        {
            *is_running = 220;
            if (num_of_restart > 0) {
                for (int i = 0; i < (int)agents.size(); i++)
                    agents[i].path.clear();
                iteration_stats.clear();
                path_table.reset();
                for (int i = 0; i < (int)agents.size(); i++)
                    collision_graph[i].clear();
            }
            bool succ = getInitialSolution();
            if(test_speed_mode) return false;
            // bool succ = getInitialSolutionMethod3();
            runtime = getTime() - start_time;
            iteration_stats.emplace_back(neighbor.agents.size(), sum_of_costs,
                                         runtime, "PP", 0,
                                         num_of_colliding_pairs);
            if (screen >= 3) printPath();
            if (screen >= 1 || true) {
                stringstream sout;
                cout << "g " << myGroup << " pe " << myId << " Iteration "
                     << iteration_stats.size() << ", "
                     << "group size = " << neighbor.agents.size() << ", "
                     << "colliding pairs = " << num_of_colliding_pairs << ", "
                     << "solution cost = " << sum_of_costs << ", "
                     << "remaining time = " << time_limit - runtime << endl;
                printf("%s", sout.str().c_str());
            }
            // cout << "InitLNS line52 " << endl;
            if (runtime > time_limit && !succ ||
                solution_shared->phase_collision_asynch_interrupt) {
                // printResult();
                break;
                ;
            }

            for (auto i = 0; i < agents.size(); i++) paths[i] = &agents[i].path;
        }

        while (true) {
            *is_running = 230;
            runtime = getTime() - start_time;
            if (solution_shared->num_of_colliding_pairs == 0 &&
                solution_shared->use_share_phase_cost) {
                if (screen > 0)
                    printf("\t--> g %d pe %d initlns:180 set collision_interrupt 1\n", 
                        myGroup, myId);
                solution_shared->phase_collision_asynch_interrupt = true;
                *is_running = 231;
                need_break = true;
            }
            if (runtime > time_limit || num_of_colliding_pairs == 0 ||
                solution_shared->phase_collision_asynch_interrupt) {
                *is_running = 232;
                if (screen > 0)
                    printf("\t--> g %d pe %d initlns timeout %f\n", 
                        myGroup, myId, runtime);
                need_break = true;
            }
            if (need_break) {
                break;
            }
            if (solution_shared->use_share_phase_collision &&
                use_sync_mode_in_iteration) {
                // cout << "g " << myGroup << " pe " << myId << " " <<
                // "Iteration " << iteration_stats.size() << " "
                //     << solution_shared->num_iter_update_phase_collision
                //     << " numLocalSolver " << numLocalSolver
                //     << endl;
                if (iteration_stats.size() * numLocalSolver >
                    solution_shared->num_iter_update_phase_collision)
                    continue;
                // if(iteration_stats.size() > 1)
                //     path_table.num_of_colliding_pairs = MAX_COST;
                solution_shared->do_share_phase_collision(myId, path_table, agents,
                                                 collision_graph,
                                                 destroy_weights, true);
                sum_of_costs = path_table.sum_of_costs;
                num_of_colliding_pairs = path_table.num_of_colliding_pairs;
                if (num_of_colliding_pairs == 0) continue;
            }
            assert(instance.validateSolution(paths, sum_of_costs,
                                             num_of_colliding_pairs));
            if(num_of_colliding_pairs == 0) {
                cout << "errrrrr" << endl;
                exit(0);
            }
            if (ALNS) chooseDestroyHeuristicbyALNS();

            bool succ;
            switch (init_destroy_strategy) {
                case TARGET_BASED:
                    succ = generateNeighborByTarget();
                    break;
                case COLLISION_BASED:
                    succ = generateNeighborByCollisionGraph();
                    break;
                case RANDOM_BASED:
                    succ = generateNeighborRandomly();
                    break;
                default:
                    cerr << "Wrong neighbor generation strategy" << endl;
                    exit(-1);
            }
            if (!succ || neighbor.agents.empty()) continue;

            *is_running = 240;
            // get colliding pairs
            neighbor.old_colliding_pairs.clear();
            for (int a : neighbor.agents) {
                for (auto j : collision_graph[a]) {
                    neighbor.old_colliding_pairs.emplace(min(a, j), max(a, j));
                }
            }
            if (neighbor.old_colliding_pairs.empty())  // no need to replan
            {
                assert(init_destroy_strategy == RANDOM_BASED);
                if (ALNS)  // update destroy heuristics
                {
                    destroy_weights[selected_neighbor] =
                        (1 - decay_factor) * destroy_weights[selected_neighbor];
                }
                continue;
            }
            *is_running = 241;

            // store the neighbor information
            neighbor.old_paths.resize(neighbor.agents.size());
            neighbor.old_sum_of_costs = 0;
            for (int i = 0; i < (int)neighbor.agents.size(); i++) {
                int a = neighbor.agents[i];
                if (replan_algo_name == "PP" || neighbor.agents.size() == 1)
                    neighbor.old_paths[i] = agents[a].path;
                path_table.deletePath(neighbor.agents[i]);
                neighbor.old_sum_of_costs += (int)agents[a].path.size() - 1;
            }
            if (screen >= 2) {
                cout << "Neighbors: ";
                for (auto a : neighbor.agents) cout << a << ", ";
                cout << endl;
                cout << "Old colliding pairs ("
                     << neighbor.old_colliding_pairs.size() << "): ";
                for (const auto& p : neighbor.old_colliding_pairs) {
                    cout << "(" << p.first << "," << p.second << "), ";
                }
                cout << endl;
            }
            // if(solution_shared->num_of_init_remaining_agents < 3)
            //     screen_const = 1;
            *is_running = 242;
            // cout << "InitLNS " << replan_algo_name << "
            // neighbor.agents.size() " << neighbor.agents.size() << endl;
            if ((replan_algo_name == "PP" || neighbor.agents.size() == 1))
                succ = runPP();
            else if (replan_algo_name == "GCBS")
                succ = runGCBS();
            else if (replan_algo_name == "PBS")
                succ = runPBS();
            else {
                cerr << "InitLNS Wrong replanning strategy " << replan_algo_name
                     << endl;
                cerr << "Need be in GCBS, PBS, PP" << endl;
                exit(-1);
            }
            simulated_annealing::update_temp(num_of_iteration);
            *is_running = 243;

            if (ALNS)  // update destroy heuristics
            {
                if (neighbor.colliding_pairs.size() < neighbor.old_colliding_pairs.size())
                    destroy_weights[selected_neighbor] =
                        reaction_factor * (double)(neighbor.old_colliding_pairs.size() -
                        neighbor.colliding_pairs.size()) // / neighbor.agents.size()
                        + (1 - reaction_factor) * destroy_weights[selected_neighbor];
                else
                    destroy_weights[selected_neighbor] =
                        (1 - decay_factor) * destroy_weights[selected_neighbor];
                double sum = 0;
                for (auto v : destroy_weights) sum += v;
                for (int i = 0; i < destroy_weights.size(); i++) {
                    destroy_weights[i] /= (1e-8 + sum);
                    destroy_weights[i] += 1e-4;
                }
            }
            *is_running = 244;
            if (screen >= 2)
                cout << "New colliding pairs = "
                     << neighbor.colliding_pairs.size() << endl;
            if (succ)  // update collision graph
            {
                num_of_colliding_pairs +=
                    (int)neighbor.colliding_pairs.size() -
                    (int)neighbor.old_colliding_pairs.size();
                for (const auto& agent_pair : neighbor.old_colliding_pairs) {
                    collision_graph[agent_pair.first].erase(agent_pair.second);
                    collision_graph[agent_pair.second].erase(agent_pair.first);
                }
                for (const auto& agent_pair : neighbor.colliding_pairs) {
                    collision_graph[agent_pair.first].emplace(
                        agent_pair.second);
                    collision_graph[agent_pair.second].emplace(
                        agent_pair.first);
                }
                if (screen >= 2) printCollisionGraph();
            }
            if (solution_shared->colliding_pairs_4_print > num_of_colliding_pairs)
                solution_shared->colliding_pairs_4_print = num_of_colliding_pairs;

            *is_running = 245;
            runtime = getTime() - start_time;
            sum_of_costs += neighbor.sum_of_costs - neighbor.old_sum_of_costs;
            float fail_rate = num_of_consecutive_failures * 1. / num_of_iteration;
            if(myId == 0)
                solution_shared->num_of_consecutive_failure_rate = fail_rate;
            if (screen >= 1 && num_of_colliding_pairs == 0)
                cout << "------> success initLNS pe " << myId << " Iteration "
                     << iteration_stats.size() << endl;
            bool flag_share = false;
            if (solution_shared != 0) {
                if (num_of_iteration % param_share_step_phase_collision == 0)
                    flag_share = true;
                if (iter_of_run_after_sa_accept > 0) {
                    iter_of_run_after_sa_accept--;
                    if (iter_of_run_after_sa_accept > 0) flag_share = false;
                }
                if (num_of_colliding_pairs == 0) flag_share = true;
            }
            if (screen >= 1)
                if (iteration_stats.size() < 1 || num_of_colliding_pairs == 0 ||
                    neighbor.colliding_pairs.size() <
                        neighbor.old_colliding_pairs.size() ||
                    iter_of_run_after_sa_accept > 0)
                    cout << "g " << myGroup << " pe " << myId 
                         << " Restart " << num_of_restart 
                         << " Iteration " << iteration_stats.size() 
                         << " neighbor " << neighbor.agents.size() 
                         << " colliding " << num_of_colliding_pairs 
                         << " failed iter " << num_of_failures << " "
                         << num_of_failures * 1. / (1e-5 + iteration_stats.size())
                         << " cons_fail " << num_of_consecutive_failures 
                         << " " << fail_rate 
                         << " iterDD " << iter_of_run_after_sa_accept 
                         << " saT " << simulated_annealing::sa_iteration_T 
                         << " cost " << sum_of_costs 
                         << " remaining " << time_limit - runtime << endl;
            path_table.sum_of_costs = sum_of_costs;
            path_table.num_of_colliding_pairs = num_of_colliding_pairs;
            if (flag_share) {
                bool force_update = false;
                if (solution_shared->use_share_phase_cost &&
                    num_of_colliding_pairs == 0)
                    force_update = true;
                double t1 = getTime();
                solution_shared->do_share_phase_collision(
                    myId, path_table, agents, collision_graph, destroy_weights,
                    force_update);
                runtime_share_data += getTime() - t1;
                sum_of_costs = path_table.sum_of_costs;
                if (path_table.num_of_colliding_pairs < num_of_colliding_pairs)
                    num_of_consecutive_failures = 0;
                num_of_colliding_pairs = path_table.num_of_colliding_pairs;
            }
            iteration_stats.emplace_back(neighbor.agents.size(), sum_of_costs,
                                         runtime, replan_algo_name, 0,
                                         num_of_colliding_pairs);
            neighbor.clear();

            *is_running = 246;
            bool bb = fail_rate > param_sa_max_con_fail;
            // 当使用多线程共享时，fail_rate要大一些才可以
            // if(solution_shared->use_share_phase_collision && false)
            //     bb = fail_rate > 0.4;
            if (num_of_consecutive_failures > 100 && num_of_iteration > 300 && bb ||
                solution_shared->phase_collision_need_restart) {
                if (solution_shared->use_share_phase_collision) {
                    solution_shared->phase_collision_need_restart = true; 
                    //todo:这里隐藏一个bug，需要同一个group中所有线程都进入到collisionlns过程
                    //不过一般需要重启的实例，都会进入到这里的，所以先不解决
                    pthread_barrier_wait(
                        &solution_shared->barrier_phase_collision_restart);  
                    solution_shared->phase_collision_need_restart = false;
                    solution_shared->num_of_colliding_pairs = MAX_COST;
                    solution_shared->sum_of_costs_wc = MAX_COST;
                }
                if (screen >= 1)
                    cout << "g " << myGroup << " pe " << myId << " Restart "
                         << num_of_restart << " Iteration "
                         << iteration_stats.size() << " neighbor "
                         << neighbor.agents.size() << " colliding "
                         << num_of_colliding_pairs << " failed iter "
                         << num_of_failures << " "
                         << num_of_failures * 1. / (1e-5 + num_of_iteration)
                         << " cons_fail " << num_of_consecutive_failures << " "
                         << fail_rate << " cost " << sum_of_costs
                         << " remaining " << time_limit - runtime << endl;
                break;
            }
        }
        *is_running = 250;
        float fail_rate = num_of_consecutive_failures * 1. / (1e-8 + num_of_iteration);
        if (screen > 0)
            cout << "\tg " << myGroup << " pe " << myId << " Restart "
                 << num_of_restart << " Iteration " << num_of_iteration
                 << " colliding pairs " << num_of_colliding_pairs
                 << " solution cost " << sum_of_costs << " cons_fail "
                 << num_of_consecutive_failures << " " << fail_rate
                 << " T_init " << simulated_annealing::sa_iteration_T_init
                 << " remaining time " << time_limit - runtime << endl;
    }

    *is_running = 260;
    printResult();

    if (solution_shared->use_share_phase_cost) {
        if (screen > 0)
            cout << "\tg " << myGroup << " pe " << myId
                 << "use_share_phase_cost get the best solution" << endl;
        double t1 = getTime();
        pthread_barrier_wait(&solution_shared->barrier_phase_collision_result);
        solution_shared->do_share_phase_collision(
            myId, path_table, agents, collision_graph, destroy_weights, true);
        runtime_share_data += getTime() - t1;
        sum_of_costs = path_table.sum_of_costs;
        num_of_colliding_pairs = path_table.num_of_colliding_pairs;
        if (solution_shared->num_of_colliding_pairs == 0 &&
            solution_shared->use_share_phase_cost) {
            if (screen > 0)
                printf("\t--> g %d pe %d initLNS set phase_collision_asynch_interrupt 0\n", 
                    myGroup, myId);
            solution_shared->phase_collision_asynch_interrupt = false;
        }
    }

    // printResult();

    return (num_of_colliding_pairs == 0);
}

bool InitLNS::runWait() {
    if (solution_shared->use_share_phase_cost)
        pthread_barrier_wait(&solution_shared->barrier_phase_collision_result);
    return true;
}

bool InitLNS::runGCBS() {
    vector<SingleAgentSolver*> search_engines;
    search_engines.reserve(neighbor.agents.size());
    for (int i : neighbor.agents) {
        search_engines.push_back(agents[i].path_planner);
    }

    // build path tables
    vector<PathTable> path_tables(neighbor.agents.size(),
                                  PathTable(instance.map_size));
    for (int i = 0; i < (int)neighbor.agents.size(); i++) {
        int agent_id = neighbor.agents[i];
        for (int j = 0; j < instance.getDefaultNumberOfAgents(); j++) {
            if (j != agent_id and collision_graph[agent_id].count(j) == 0)
                path_tables[i].insertPath(j, agents[j].path);
        }
    }

    GCBS gcbs(search_engines, screen - 1, &path_tables);
    gcbs.setDisjointSplitting(false);
    gcbs.setBypass(true);
    gcbs.setTargetReasoning(true);

    runtime = getTime() - start_time;
    double T = time_limit - runtime;
    if (!iteration_stats.empty())  // replan
        T = min(T, replan_time_limit);
    gcbs.solve(T);
    if (gcbs.best_node->colliding_pairs <
        (int)neighbor.old_colliding_pairs.size())  // accept new paths
    {
        auto id = neighbor.agents.begin();
        neighbor.colliding_pairs.clear();
        for (size_t i = 0; i < neighbor.agents.size(); i++) {
            agents[*id].path = *gcbs.paths[i];
            updateCollidingPairs(neighbor.colliding_pairs, agents[*id].id,
                                 agents[*id].path);
            path_table.insertPath(agents[*id].id, agents[*id].path);
            ++id;
        }
        neighbor.sum_of_costs = gcbs.best_node->sum_of_costs;
        return true;
    } else  // stick to old paths
    {
        if (!neighbor.old_paths.empty()) {
            for (int id : neighbor.agents) {
                path_table.insertPath(agents[id].id, agents[id].path);
            }
            neighbor.sum_of_costs = neighbor.old_sum_of_costs;
        }
        num_of_failures++;
        return false;
    }
}
bool InitLNS::runPBS() {
    vector<SingleAgentSolver*> search_engines;
    search_engines.reserve(neighbor.agents.size());
    vector<const Path*> initial_paths;
    initial_paths.reserve(neighbor.agents.size());
    for (int i : neighbor.agents) {
        search_engines.push_back(agents[i].path_planner);
        initial_paths.push_back(&agents[i].path);
    }

    PBS pbs(search_engines, path_table, screen - 1);
    // pbs.setInitialPath(initial_paths);
    runtime = getTime() - start_time;
    double T = time_limit - runtime;
    if (!iteration_stats.empty())  // replan
        T = min(T, replan_time_limit);
    bool succ = pbs.solve(T, (int)neighbor.agents.size(),
                          neighbor.old_colliding_pairs.size());
    if (succ and
        pbs.best_node->getCollidingPairs() <
            (int)neighbor.old_colliding_pairs.size())  // accept new paths
    {
        auto id = neighbor.agents.begin();
        neighbor.colliding_pairs.clear();
        for (size_t i = 0; i < neighbor.agents.size(); i++) {
            agents[*id].path = *pbs.paths[i];
            updateCollidingPairs(neighbor.colliding_pairs, agents[*id].id,
                                 agents[*id].path);
            path_table.insertPath(agents[*id].id);
            ++id;
        }
        assert(neighbor.colliding_pairs.size() ==
               pbs.best_node->getCollidingPairs());
        neighbor.sum_of_costs = pbs.best_node->sum_of_costs;
        return true;
    } else  // stick to old paths
    {
        if (!neighbor.old_paths.empty()) {
            for (int id : neighbor.agents) {
                path_table.insertPath(agents[id].id);
            }
            neighbor.sum_of_costs = neighbor.old_sum_of_costs;
        }
        num_of_failures++;
        return false;
    }
}

bool InitLNS::runPP() {
    num_of_iteration++;
    auto shuffled_agents = neighbor.agents;
    std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());
    if (screen >= 2) {
        cout << "Neighbors_set: ";
        for (auto id : shuffled_agents) cout << id << ", ";
        cout << endl;
    }
    int remaining_agents = (int)shuffled_agents.size();
    auto p = shuffled_agents.begin();
    neighbor.sum_of_costs = 0;
    neighbor.colliding_pairs.clear();
    runtime = getTime() - start_time;
    double T = min(time_limit - runtime, replan_time_limit);
    auto time = getTime();
    ConstraintTable constraint_table(instance.num_of_cols, instance.map_size,
                                     nullptr, &path_table);
    while (p != shuffled_agents.end() &&
           (getTime() - start_time) < time_limit &&
           !solution_shared->phase_collision_asynch_interrupt) {
        if (solution_shared->num_of_colliding_pairs == 0 &&
            solution_shared->use_share_phase_cost)
            break;
        int id = *p;
        agents[id].path = agents[id].path_planner->findPath(constraint_table);
        // while(getTime() - time < T) { // for test sipp speed
        //     num_of_ll_search++;
        //     agents[id].path = agents[id].path_planner->findPath(constraint_table);
        // }
        if (solution_shared->phase_collision_asynch_interrupt) break;
        if (agents[id].path.empty()) { //ck add
            // 重新绑定interrupt的问题会导致这个错误
            printf("    g %d pe %d PP agents[id].path.empty()\n", myGroup, myId);
            exit(0);
        }
        num_of_ll_search++;
        assert(!agents[id].path.empty() &&
               agents[id].path.back().location ==
                   agents[id].path_planner->goal_location);
        if (agents[id].path_planner->num_collisions > 0)
            updateCollidingPairs(neighbor.colliding_pairs, agents[id].id,
                                 agents[id].path);
        assert(agents[id].path_planner->num_collisions > 0 or
               !updateCollidingPairs(neighbor.colliding_pairs, agents[id].id,
                                     agents[id].path));
        neighbor.sum_of_costs += (int)agents[id].path.size() - 1;
        remaining_agents--;
        if (screen >= 3) {
            runtime = getTime() - start_time;
            cout << "After agent " << id
                 << ": Remaining agents = " << remaining_agents
                 << ", colliding pairs = " << neighbor.colliding_pairs.size()
                 << ", LL nodes = " << agents[id].path_planner->getNumExpanded()
                 << ", remaining time = " << time_limit - runtime
                 << " seconds. " << endl;
        }
        if (!tl_use_simulated_annealing)  // || neighbor_size <= 4)
        {
            if (neighbor.colliding_pairs.size() >=
                neighbor.old_colliding_pairs.size())
                break;
        }
        path_table.insertPath(agents[id].id, agents[id].path);
        ++p;
    }
    if (p == shuffled_agents.end() &&
        neighbor.colliding_pairs.size() <=
            neighbor.old_colliding_pairs.size())  // accept new paths
    {
        if (neighbor.colliding_pairs.size() <
            neighbor.old_colliding_pairs.size())
            num_of_consecutive_failures = 0;
        return true;
    } else  // stick to old paths
    {
        num_of_failures++;
        if (neighbor_size > 6) num_of_consecutive_failures++;
        if (tl_use_simulated_annealing && p == shuffled_agents.end())  // && neighbor_size > 4)
        {
            double dt = neighbor.colliding_pairs.size() -
                        neighbor.old_colliding_pairs.size();
            bool b =
                simulated_annealing::can_accept(dt, screen, num_of_iteration);
            if (b) {
                if (iter_of_run_after_sa_accept == 0)
                    iter_of_run_after_sa_accept =
                        param_num_of_run_after_sa_accept;
                return true;
            }
        }
        auto p2 = shuffled_agents.begin();
        while (p2 != p) {
            int a = *p2;
            path_table.deletePath(agents[a].id);
            ++p2;
        }
        if (!neighbor.old_paths.empty()) {
            p2 = neighbor.agents.begin();
            for (int i = 0; i < (int)neighbor.agents.size(); i++) {
                int a = *p2;
                agents[a].path = neighbor.old_paths[i];
                path_table.insertPath(agents[a].id);
                ++p2;
            }
            neighbor.sum_of_costs = neighbor.old_sum_of_costs;
        }
        return false;
    }
}

bool InitLNS::getInitialSolution() {
    neighbor.agents.clear();
    neighbor.agents.reserve(agents.size());
    sum_of_costs = 0;
    for (int i = 0; i < (int)agents.size(); i++) {
        if (agents[i].path.empty())
            neighbor.agents.push_back(i);
        else {
            sum_of_costs += (int)agents[i].path.size() - 1;
            path_table.insertPath(agents[i].id, agents[i].path);
        }
    }
    int remaining_agents = (int)neighbor.agents.size();
    if (screen > 0)
        cout << "g " << myGroup << " pe " << myId << " "
             << "initLNS getInitialSolution remaining_agents "
             << remaining_agents << endl;
    std::random_shuffle(neighbor.agents.begin(), neighbor.agents.end());
    ConstraintTable constraint_table(instance.num_of_cols, instance.map_size,
                                     nullptr, &path_table);
    set<pair<int, int>> colliding_pairs;
    if(test_speed_mode) num_of_ll_search = 0;
    for (auto id : neighbor.agents) {
        agents[id].path = agents[id].path_planner->findPath(constraint_table);
        // if (agents[id].path.empty()) { //问题太难找不到解，ck add
        //     printf("    g %d pe %d getinit agents[id].path.empty()\n", myGroup, myId);
        //     solution_shared->phase_collision_asynch_interrupt = true;
        //     break;
        // }
        if(test_speed_mode) {
            for(auto i=0; i<50; i++)
                agents[id].path = agents[id].path_planner->findPath(constraint_table);
            if(myId == 0 && num_of_ll_search >= 200 || getTime()-start_time >= 3) {
                solution_shared->phase_collision_asynch_interrupt = true;
                return false;
            }
        }
        if (solution_shared->phase_collision_asynch_interrupt) break;
        if (agents[id].path.empty()) 
            break;
        num_of_ll_search++;
        assert(!agents[id].path.empty() &&
               agents[id].path.back().location ==
                   agents[id].path_planner->goal_location);
        if (agents[id].path_planner->num_collisions > 0)
            updateCollidingPairs(colliding_pairs, agents[id].id,
                                 agents[id].path);
        sum_of_costs += (int)agents[id].path.size() - 1;
        remaining_agents--;
        path_table.insertPath(agents[id].id, agents[id].path);
        runtime = getTime() - start_time;
        if (screen >= 3) {
            cout << "After agent " << id
                 << ": Remaining agents = " << remaining_agents
                 << ", colliding pairs = " << colliding_pairs.size()
                 << ", LL nodes = " << agents[id].path_planner->getNumExpanded()
                 << ", remaining time = " << time_limit - runtime
                 << " seconds. " << endl;
        }
        if (agents.size() >= 300) {
            if (solution_shared->num_of_init_remaining_agents > remaining_agents)
                solution_shared->num_of_init_remaining_agents = remaining_agents;
        }
        if (runtime > time_limit) {
            printf("\t--> g %d pe %d initlns getInitialSolution break timeout %d", runtime);
            break;
        }
        if (solution_shared->phase_collision_asynch_interrupt) {
            printf("\t--> g %d pe %d initlns getInitialSolution break phase_collision_asynch_interrupt");
            break;
        }
    }

    num_of_colliding_pairs = colliding_pairs.size();
    for (const auto& agent_pair : colliding_pairs) {
        collision_graph[agent_pair.first].emplace(agent_pair.second);
        collision_graph[agent_pair.second].emplace(agent_pair.first);
    }
    if(remaining_agents > 0) {
        sum_of_costs = MAX_COST;
        num_of_colliding_pairs = MAX_COST;
    }
    // assert(path_table.sum_of_costs == sum_of_costs);
    path_table.sum_of_costs = sum_of_costs;
    path_table.num_of_colliding_pairs = num_of_colliding_pairs;
    if (screen >= 2) printCollisionGraph();
    return remaining_agents == 0;
}

/**
 * @brief
 * 每个线程负责一部分的初始化，然后合并成一个solution。
 * 但是这种方法会产生大量的冲突，后续的耗时会更长
 *
 * @return true
 * @return false
 */
bool InitLNS::getInitialSolutionMethod2() {
    neighbor.agents.clear();
    neighbor.agents.reserve(agents.size());
    sum_of_costs = 0;
    int batch_size = agents.size() / numLocalSolver;
    if (batch_size * numLocalSolver < agents.size()) batch_size++;
    int i_start = batch_size * myId;
    int i_end = batch_size * (myId + 1);
    if (i_end > agents.size()) i_end = agents.size();
    for (int i = i_start; i < i_end; i++) {
        if (agents[i].path.empty())
            neighbor.agents.push_back(i);
        else {
            sum_of_costs += (int)agents[i].path.size() - 1;
            path_table.insertPath(agents[i].id, agents[i].path);
        }
    }
    int remaining_agents = (int)neighbor.agents.size();
    if (screen > 0)
        cout << "g " << myGroup << " pe " << myId << " "
             << "initLNS getInitialSolution0 remaining_agents "
             << remaining_agents << endl;
    std::random_shuffle(neighbor.agents.begin(), neighbor.agents.end());
    set<pair<int, int>> colliding_pairs;
    ConstraintTable constraint_table(instance.num_of_cols, instance.map_size,
                                     nullptr, &path_table);
    for (auto id : neighbor.agents) {
        agents[id].path = agents[id].path_planner->findPath(constraint_table);
        num_of_ll_search++;
        if (solution_shared->phase_collision_asynch_interrupt) break;
        assert(!agents[id].path.empty() &&
               agents[id].path.back().location ==
                   agents[id].path_planner->goal_location);
        if (agents[id].path_planner->num_collisions > 0)
            updateCollidingPairs(colliding_pairs, agents[id].id,
                                 agents[id].path);
        sum_of_costs += (int)agents[id].path.size() - 1;
        remaining_agents--;
        path_table.insertPath(agents[id].id, agents[id].path);
        runtime = getTime() - start_time;
        if (screen >= 3) {
            cout << "After agent " << id
                 << ": Remaining agents = " << remaining_agents
                 << ", colliding pairs = " << colliding_pairs.size()
                 << ", LL nodes = " << agents[id].path_planner->getNumExpanded()
                 << ", remaining time = " << time_limit - runtime
                 << " seconds. " << endl;
        }
        if (runtime > time_limit || solution_shared->phase_collision_asynch_interrupt)
            break;
    }

    if (remaining_agents != 0) {
        if (screen > 0)
            printf("\t--> g %d pe %d initlns:738 set collision_interrupt 1\n", 
                myGroup, myId);
        solution_shared->phase_collision_asynch_interrupt = true;
    }
    else {
        // 先写在这里，如果效果好，再移到share类中
        solution_shared->pathTableUpdateLock.lock();

        if (solution_shared->agents_path.size() != agents.size())
            solution_shared->agents_path.resize(agents.size());
        for (auto i : neighbor.agents)
            solution_shared->agents_path[i] = agents[i].path;

        solution_shared->pathTableUpdateLock.unlock();
    }

    // 等待所有完成
    pthread_barrier_wait(&solution_shared->barrier_phase_collision_result);

    if (solution_shared->phase_collision_asynch_interrupt) return false;

    // 由myId=0的线程来更新path_table，collision_graph
    if (myId == 0) {
        // 不需要加锁
        for (int id = 0; id < (int)agents.size(); id++)
            agents[id].path = solution_shared->agents_path[id];

        sum_of_costs = 0;
        path_table.reset();
        colliding_pairs.clear();
        for (int id = 0; id < (int)agents.size(); id++) {
            assert(!agents[id].path.empty());
            if (id > 0)
                updateCollidingPairs(colliding_pairs, agents[id].id,
                                     agents[id].path);
            path_table.insertPath(agents[id].id, agents[id].path);
            sum_of_costs += (int)agents[id].path.size() - 1;
            if (screen >= 3) {
                cout << "After agent " << id
                     << ", colliding pairs = " << colliding_pairs.size()
                     << endl;
            }
        }

        for (int i = 0; i < (int)agents.size(); i++) collision_graph[i].clear();
        num_of_colliding_pairs = colliding_pairs.size();

        path_table.num_of_colliding_pairs = num_of_colliding_pairs;
        path_table.sum_of_costs = sum_of_costs;

        for (const auto& agent_pair : colliding_pairs) {
            collision_graph[agent_pair.first].emplace(agent_pair.second);
            collision_graph[agent_pair.second].emplace(agent_pair.first);
        }
        if (screen >= 4) printCollisionGraph();

        solution_shared->do_share_phase_collision(
            myId, path_table, agents, collision_graph, destroy_weights, true);
    }

    // 等待所有完成
    pthread_barrier_wait(&solution_shared->barrier_phase_collision_result);

    if (myId > 0) {
        path_table.num_of_colliding_pairs = MAX_COST;
        solution_shared->do_share_phase_collision(
            myId, path_table, agents, collision_graph, destroy_weights, true);
        num_of_colliding_pairs = path_table.num_of_colliding_pairs;
        sum_of_costs = path_table.sum_of_costs;
    }

    return true;
}

/**
 * @brief
 * 在每一步中，多个线程一起竞争性的求解
 *
 * @return true
 * @return false
 */
bool InitLNS::getInitialSolutionMethod3() {
    neighbor.agents.clear();
    neighbor.agents.reserve(agents.size());
    sum_of_costs = 0;
    for (int i = 0; i < (int)agents.size(); i++) {
        if (agents[i].path.empty())
            neighbor.agents.push_back(i);
        else {
            sum_of_costs += (int)agents[i].path.size() - 1;
            path_table.insertPath(agents[i].id, agents[i].path);
        }
    }
    int remaining_agents = (int)neighbor.agents.size();
    if (screen > 0)
        cout << "g " << myGroup << " pe " << myId << " "
             << "initLNS getInitialSolution remaining_agents "
             << remaining_agents << endl;

    // 初始化golbal path
    solution_shared->pathTableUpdateLock.lock();
    if (solution_shared->agents_path.size() != agents.size())
        solution_shared->agents_path.resize(agents.size());
    // for (auto i : neighbor.agents)
    //     solution_shared->agents_path[i] = agents[i].path;
    if (myId == 0) {
        solution_shared->method3_remains.resize(numLocalSolver,
                                                remaining_agents);
        solution_shared->method3_select_agents.resize(numLocalSolver, -1);
    }

    solution_shared->pathTableUpdateLock.unlock();

    // 先不shuffle了，这样每个线程的id都是一样的。
    // std::random_shuffle(neighbor.agents.begin(), neighbor.agents.end());

    if (myId == 0)
        solution_shared->nRemainForInitialSolutionMethod3 =
            remaining_agents;  //*numLocalSolver;
    pthread_barrier_wait(&solution_shared->barrier_phase_collision_result);

    ConstraintTable constraint_table(instance.num_of_cols, instance.map_size,
                                     nullptr, &path_table);
    set<pair<int, int>> colliding_pairs;

    while (true) {
        *is_running = 301;
        // 获取下一个agent
        solution_shared->method3_remains[myId] = remaining_agents;
        solution_shared->method3_select_agents[myId] = -1;
        int id = -1;
        for (auto i : neighbor.agents) {
            if (solution_shared->agents_path[i].empty()) {
                id = i;
                break;
            }
        }
        if (id == -1) {
            *is_running = 302;
            break;
        }
        solution_shared->method3_select_agents[myId] = id;

        if (screen > 1)
            printf(
                "  0-- g %d pe %d remain_agents %d * %d = %d g_remain_agents "
                "%d\n",
                myGroup, myId, remaining_agents, numLocalSolver,
                remaining_agents * numLocalSolver,
                (int)solution_shared->nRemainForInitialSolutionMethod3);

        if (myId == 0) {
            solution_shared->phase_cost_asynch_interrupt = false;
            solution_shared->idForInitialSolutionMethod3 = -1;
            solution_shared->numDoneForInitialSolutionMethod3 = 0;
        }
        pthread_barrier_wait(&solution_shared->barrier_phase_collision_result_31);

        if (screen > 1)
            printf("  1-- g %d pe %d global (%d, %ld) cur-id (%d, %ld)\n",
                   myGroup, myId,
                   (int)solution_shared->idForInitialSolutionMethod3,
                   solution_shared->agents_path[id].size(), id,
                   agents[id].path.size());

        // 规划agent
        agents[id].path = agents[id].path_planner->findPath(constraint_table);
        num_of_ll_search++;
        if (solution_shared->phase_collision_asynch_interrupt) {
            *is_running = 303;
            break;
        }
        solution_shared->phase_cost_asynch_interrupt = true;
        solution_shared->numDoneForInitialSolutionMethod3++;
        if (screen > 1)
            printf(
                "  2-- g %d pe %d global (%d, %ld) cur-id (%d, %ld) done %d\n",
                myGroup, myId,
                (int)solution_shared->idForInitialSolutionMethod3,
                solution_shared->agents_path[id].size(), id,
                agents[id].path.size(),
                int(solution_shared->numDoneForInitialSolutionMethod3));

        //等待都结束
        while (solution_shared->numDoneForInitialSolutionMethod3 <
                   numLocalSolver &&
               !solution_shared->phase_collision_asynch_interrupt)
            *is_running = 3031;

        if (screen > 1)
            printf("  3-- g %d pe %d global (%d, %ld) cur-id (%d, %ld)\n",
                   myGroup, myId,
                   (int)solution_shared->idForInitialSolutionMethod3,
                   solution_shared->agents_path[id].size(), id,
                   agents[id].path.size());

        // set the global path
        solution_shared->pathTableUpdateLock.lock();
        if (solution_shared->idForInitialSolutionMethod3 == -1 &&
            !agents[id].path.empty()) {
            solution_shared->idForInitialSolutionMethod3 = id;
            solution_shared->agents_path[id] = agents[id].path;
            // 先用phase_cost_asynch_interrupt来停止sipp的运行
        }
        solution_shared->pathTableUpdateLock.unlock();

        if (screen > 1)
            printf("--4-- g %d pe %d global (%d, %ld) cur-id (%d, %ld)\n",
                   myGroup, myId,
                   (int)solution_shared->idForInitialSolutionMethod3,
                   solution_shared->agents_path[id].size(), id,
                   agents[id].path.size());

        pthread_barrier_wait(&solution_shared->barrier_phase_collision_result_32);
        assert(!solution_shared
                    ->agents_path[solution_shared->idForInitialSolutionMethod3]
                    .empty());
        assert(solution_shared->idForInitialSolutionMethod3 != -1);

        // get the global path
        id = solution_shared->idForInitialSolutionMethod3;
        agents[id].path = solution_shared->agents_path[id];

        assert(!agents[id].path.empty() &&
               agents[id].path.back().location ==
                   agents[id].path_planner->goal_location);
        updateCollidingPairs(colliding_pairs, agents[id].id, agents[id].path);
        sum_of_costs += (int)agents[id].path.size() - 1;

        path_table.insertPath(agents[id].id, agents[id].path);

        solution_shared->pathTableUpdateLock.lock();
        // solution_shared->nRemainForInitialSolutionMethod3--;
        remaining_agents--;
        if (screen > 1)
            printf(
                "============ g %d pe %d remain_agents %d * %d = %d "
                "g_remain_agents %d\n",
                myGroup, myId, remaining_agents, numLocalSolver,
                remaining_agents * numLocalSolver,
                (int)solution_shared->nRemainForInitialSolutionMethod3);
        solution_shared->pathTableUpdateLock.unlock();

        //等待都结束了
        // while(solution_shared->nRemainForInitialSolutionMethod3 >
        // last_nRemainForInitialSolutionMethod3 - numLocalSolver
        //     && !solution_shared->phase_collision_asynch_interrupt)
        //     *is_running = 305;
        // while(true)
        // {
        //     *is_running = 3012;
        //     if(solution_shared->nRemainForInitialSolutionMethod3 - 1 ==
        //     remaining_agents)
        //         break;
        // }
        // *is_running = 3013;
        pthread_barrier_wait(&solution_shared->barrier_phase_collision_result_33);

        runtime = getTime() - start_time;
        if (screen >= 3) {
            cout << "After agent " << id
                 << ": Remaining agents = " << remaining_agents
                 << ", colliding pairs = " << colliding_pairs.size()
                 << ", LL nodes = " << agents[id].path_planner->getNumExpanded()
                 << ", remaining time = " << time_limit - runtime
                 << " seconds. " << endl;
        }
        if (agents.size() >= 300) {
            if (solution_shared->num_of_init_remaining_agents >
                remaining_agents)
                solution_shared->num_of_init_remaining_agents =
                    remaining_agents;
        }
        *is_running = 306;
        if (runtime > time_limit || solution_shared->phase_collision_asynch_interrupt)
            break;

        // 结束处理
        if (myId == 0) solution_shared->nRemainForInitialSolutionMethod3--;
        while (true) {
            *is_running = 3011;
            if (solution_shared->nRemainForInitialSolutionMethod3 ==
                remaining_agents)
                break;
        }
        pthread_barrier_wait(&solution_shared->barrier_phase_collision_result_34);
    }

    solution_shared->phase_collision_asynch_interrupt = false;
    solution_shared->phase_cost_asynch_interrupt = false;
    num_of_colliding_pairs = colliding_pairs.size();
    for (const auto& agent_pair : colliding_pairs) {
        collision_graph[agent_pair.first].emplace(agent_pair.second);
        collision_graph[agent_pair.second].emplace(agent_pair.first);
    }
    if (screen >= 2) printCollisionGraph();
    return remaining_agents == 0;
}

// return true if the new p[ath has collisions;
bool InitLNS::updateCollidingPairs(set<pair<int, int>>& colliding_pairs,
                                   int agent_id, const Path& path) const {
    bool succ = false;
    if (path.size() < 2) return succ;
    for (int t = 1; t < (int)path.size(); t++) {
        int from = path[t - 1].location;
        int to = path[t].location;
        if ((int)path_table.table[to].size() > t)  // vertex conflicts
        {
            for (auto id : path_table.table[to][t]) {
                succ = true;
                colliding_pairs.emplace(min(agent_id, id), max(agent_id, id));
            }
        }
        if (from != to && path_table.table[to].size() >= t &&
            path_table.table[from].size() > t)  // edge conflicts
        {
            for (auto a1 : path_table.table[to][t - 1]) {
                for (auto a2 : path_table.table[from][t]) {
                    if (a1 == a2) {
                        succ = true;
                        colliding_pairs.emplace(min(agent_id, a1),
                                                max(agent_id, a1));
                        break;
                    }
                }
            }
        }
        // auto id = getAgentWithTarget(to, t);
        // if (id >= 0) // this agent traverses the target of another agent
        //     colliding_pairs.emplace(min(agent_id, id), max(agent_id, id));
        if (!path_table.goals.empty() &&
            path_table.goals[to] < t)  // target conflicts
        {  // this agent traverses the target of another agent
            for (auto id :
                 path_table
                     .table[to][path_table.goals[to]])  // look at all agents at
                                                        // the goal time
            {
                if (agents[id].path.back().location ==
                    to)  // if agent id's goal is to, then this is the agent we
                         // want
                {
                    succ = true;
                    colliding_pairs.emplace(min(agent_id, id),
                                            max(agent_id, id));
                    break;
                }
            }
        }
    }
    int goal = path.back().location;  // target conflicts - some other agent
                                      // traverses the target of this agent
    for (int t = (int)path.size(); t < path_table.table[goal].size(); t++) {
        for (auto id : path_table.table[goal][t]) {
            succ = true;
            colliding_pairs.emplace(min(agent_id, id), max(agent_id, id));
        }
    }
    return succ;
}

void InitLNS::chooseDestroyHeuristicbyALNS() {
    rouletteWheel();
    switch (selected_neighbor) {
        case 0:
            init_destroy_strategy = TARGET_BASED;
            break;
        case 1:
            init_destroy_strategy = COLLISION_BASED;
            break;
        case 2:
            init_destroy_strategy = RANDOM_BASED;
            break;
        default:
            cerr << "ERROR" << endl;
            exit(-1);
    }
}

bool InitLNS::generateNeighborByCollisionGraph() {
    vector<int> all_vertices;
    all_vertices.reserve(collision_graph.size());
    for (int i = 0; i < (int)collision_graph.size(); i++) {
        if (!collision_graph[i].empty()) all_vertices.push_back(i);
    }
    unordered_map<int, set<int>> G;
    if(all_vertices.empty()) {
        printf("1342\n"); exit(0);
    }
    auto v =
        all_vertices[rand() % all_vertices.size()];  // pick a random vertex
    findConnectedComponent(collision_graph, v, G);
    assert(G.size() > 1);

    assert(neighbor_size <= (int)agents.size());
    set<int> neighbors_set;
    if ((int)G.size() <= neighbor_size) {
        for (const auto& node : G) neighbors_set.insert(node.first);
        int count = 0;
        while ((int)neighbors_set.size() < neighbor_size && count < 10) {
            if(neighbors_set.empty()) {
                printf("1343\n"); exit(0);
            }
            int a1 = *std::next(neighbors_set.begin(),
                                rand() % neighbors_set.size());
            int a2 = randomWalk(a1);
            if (a2 != NO_AGENT)
                neighbors_set.insert(a2);
            else
                count++;
        }
    } else {
        if(G.empty()) {
            printf("1344\n"); exit(0);
        }
        int a = std::next(G.begin(), rand() % G.size())->first;
        neighbors_set.insert(a);
        while ((int)neighbors_set.size() < neighbor_size) {
            if(G[a].empty()) {
                printf("13441\n"); exit(0);
            }
            a = *std::next(G[a].begin(), rand() % G[a].size());
            neighbors_set.insert(a);
        }
    }
    neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
    if (screen >= 2)
        cout << "Generate " << neighbor.agents.size()
             << " neighbors by collision graph" << endl;
    return true;
}
bool InitLNS::generateNeighborByTarget() {
    int a = -1;
    if(num_of_colliding_pairs == 0) {
        printf("1345\n"); exit(0);
    }
    auto r = rand() % (num_of_colliding_pairs * 2);
    int sum = 0;
    for (int i = 0; i < (int)collision_graph.size(); i++) {
        sum += (int)collision_graph[i].size();
        if (r <= sum and !collision_graph[i].empty()) {
            a = i;
            break;
        }
    }
    assert(a != -1 and !collision_graph[a].empty());
    set<pair<int, int>> A_start;  // an ordered set of (time, id) pair.
    set<int> A_target;

    for (int t = 0;
         t < path_table.table[agents[a].path_planner->start_location].size();
         t++) {
        for (auto id :
             path_table.table[agents[a].path_planner->start_location][t]) {
            if (id != a) A_start.insert(make_pair(t, id));
        }
    }

    agents[a].path_planner->findMinimumSetofColldingTargets(
        goal_table, A_target);  // generate non-wait path and collect A_target

    if (screen >= 3) {
        cout << "     Selected a : " << a << endl;
        cout << "     Select A_start: ";
        for (auto e : A_start)
            cout << "(" << e.first << "," << e.second << "), ";
        cout << endl;
        cout << "     Select A_target: ";
        for (auto e : A_target) cout << e << ", ";
        cout << endl;
    }

    set<int> neighbors_set;

    neighbors_set.insert(a);

    if (A_start.size() + A_target.size() >= neighbor_size - 1) {
        if (A_start.empty()) {
            vector<int> shuffled_agents;
            shuffled_agents.assign(A_target.begin(), A_target.end());
            std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());
            neighbors_set.insert(shuffled_agents.begin(),
                                 shuffled_agents.begin() + neighbor_size - 1);
        } else if (A_target.size() >= neighbor_size) {
            vector<int> shuffled_agents;
            shuffled_agents.assign(A_target.begin(), A_target.end());
            std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());
            neighbors_set.insert(shuffled_agents.begin(),
                                 shuffled_agents.begin() + neighbor_size - 2);

            neighbors_set.insert(A_start.begin()->second);
        } else {
            neighbors_set.insert(A_target.begin(), A_target.end());
            for (auto e : A_start) {
                // A_start is ordered by time.
                if (neighbors_set.size() >= neighbor_size) break;
                neighbors_set.insert(e.second);
            }
        }
    } else if (!A_start.empty() || !A_target.empty()) {
        neighbors_set.insert(A_target.begin(), A_target.end());
        for (auto e : A_start) {
            neighbors_set.insert(e.second);
        }

        set<int> tabu_set;
        while (neighbors_set.size() < neighbor_size) {
            if(neighbors_set.empty()) {
                printf("1346\n"); exit(0);
            }
            int rand_int = rand() % neighbors_set.size();
            auto it = neighbors_set.begin();
            std::advance(it, rand_int);
            a = *it;
            tabu_set.insert(a);

            if (tabu_set.size() == neighbors_set.size()) break;

            vector<int> targets;
            for (auto p : agents[a].path) {
                if (goal_table[p.location] > -1) {
                    targets.push_back(goal_table[p.location]);
                }
            }

            if (targets.empty()) continue;
            rand_int = rand() % targets.size();
            neighbors_set.insert(*(targets.begin() + rand_int));
        }
    }

    neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
    if (screen >= 2)
        cout << "Generate " << neighbor.agents.size() << " neighbors by target"
             << endl;
    return true;
}
bool InitLNS::generateNeighborRandomly() {
    if (neighbor_size >= agents.size()) {
        neighbor.agents.resize(agents.size());
        for (int i = 0; i < (int)agents.size(); i++) neighbor.agents[i] = i;
        return true;
    }
    set<int> neighbors_set;
    auto total = num_of_colliding_pairs * 2 + agents.size();
    if(total == 0) {
        printf("1347\n"); exit(0);
    }
    while (neighbors_set.size() < neighbor_size) {
        vector<int> r(neighbor_size - neighbors_set.size());
        for (auto i = 0; i < neighbor_size - neighbors_set.size(); i++)
            r[i] = rand() % total;
        std::sort(r.begin(), r.end());
        int sum = 0;
        for (int i = 0, j = 0; i < agents.size() and j < r.size(); i++) {
            sum += (int)collision_graph[i].size() + 1;
            if (sum >= r[j]) {
                neighbors_set.insert(i);
                while (j < r.size() and sum >= r[j]) j++;
            }
        }
    }
    neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
    if (screen >= 2)
        cout << "Generate " << neighbor.agents.size() << " neighbors randomly"
             << endl;
    return true;
}

// Random walk; return the first agent that the agent collides with
int InitLNS::randomWalk(int agent_id) {
    if (agents[agent_id].path.size() == 0) {
        printf("1348\n"); exit(0);
    }
    int t = rand() % agents[agent_id].path.size();
    int loc = agents[agent_id].path[t].location;
    while (t <= path_table.makespan and
           (path_table.table[loc].size() <= t or
            path_table.table[loc][t].empty() or
            (path_table.table[loc][t].size() == 1 and
             path_table.table[loc][t].front() == agent_id))) {
        auto next_locs = instance.getNeighbors(loc);
        next_locs.push_back(loc);
        if (next_locs.size() == 0) {
            printf("1349\n"); exit(0);
        }
        int step = rand() % next_locs.size();
        auto it = next_locs.begin();
        loc = *std::next(next_locs.begin(), rand() % next_locs.size());
        t = t + 1;
    }
    if (t > path_table.makespan)
        return NO_AGENT;
    else {
        if (path_table.table[loc][t].size() == 0) {
            printf("1350\n"); exit(0);
        }
        return *std::next(path_table.table[loc][t].begin(),
                          rand() % path_table.table[loc][t].size());
    }
}

void InitLNS::writeIterStatsToFile(const string& file_name) const {
    std::ofstream output;
    output.open(file_name);
    // header
    output <<  //"num of agents," <<
        "sum of costs,"
           << "num of colliding pairs,"
           << "runtime" <<  //"," <<
        //"MAPF algorithm" <<
        endl;

    for (const auto& data : iteration_stats) {
        output <<  // data.num_of_agents << "," <<
            data.sum_of_costs << "," << data.num_of_colliding_pairs << ","
               << data.runtime <<  //"," <<
            // data.algorithm <<
            endl;
    }
    output.close();
}

void InitLNS::writeResultToFile(const string& file_name, int sum_of_distances,
                                double preprocessing_time) const {
    std::ifstream infile(file_name);
    bool exist = infile.good();
    infile.close();
    if (!exist) {
        ofstream addHeads(file_name);
        addHeads << "runtime,num of collisions,solution cost,initial "
                    "collisions,initial solution cost,"
                 << "sum of distances,iterations,group size,"
                 << "runtime of initial solution,area under curve,"
                 << "LL expanded nodes,LL generated,LL reopened,LL runs,"
                 << "preprocessing runtime,solver name,instance name" << endl;
        addHeads.close();
    }
    uint64_t num_LL_expanded = 0, num_LL_generated = 0, num_LL_reopened = 0,
             num_LL_runs = 0;
    for (auto& agent : agents) {
        agent.path_planner->reset();
        num_LL_expanded += agent.path_planner->accumulated_num_expanded;
        num_LL_generated += agent.path_planner->accumulated_num_generated;
        num_LL_reopened += agent.path_planner->accumulated_num_reopened;
        num_LL_runs += agent.path_planner->num_runs;
    }
    double auc = 0;
    if (!iteration_stats.empty()) {
        auto prev = iteration_stats.begin();
        auto curr = prev;
        ++curr;
        while (curr != iteration_stats.end() && curr->runtime < time_limit) {
            auc +=
                prev->num_of_colliding_pairs * (curr->runtime - prev->runtime);
            prev = curr;
            ++curr;
        }
        auc += prev->num_of_colliding_pairs * (time_limit - prev->runtime);
    }

    ofstream stats(file_name, std::ios::app);
    stats << runtime << "," << iteration_stats.back().num_of_colliding_pairs
          << "," << sum_of_costs << ","
          << iteration_stats.front().num_of_colliding_pairs << ","
          << iteration_stats.front().sum_of_costs << "," << sum_of_distances
          << "," << iteration_stats.size() << "," << average_group_size << ","
          << iteration_stats.front().runtime << "," << auc << ","
          << num_LL_expanded << "," << num_LL_generated << ","
          << num_LL_reopened << "," << num_LL_runs << "," << preprocessing_time
          << "," << getSolverName() << "," << instance.getInstanceName()
          << endl;
    stats.close();
}

void InitLNS::printCollisionGraph() const {
    cout << "Collision graph: ";
    int edges = 0;
    for (size_t i = 0; i < collision_graph.size(); i++) {
        for (int j : collision_graph[i]) {
            if (i < j) {
                cout << "(" << i << "," << j << "),";
                edges++;
            }
        }
    }
    cout << endl
         << "|V|=" << collision_graph.size() << ", |E|=" << edges << endl;
}

unordered_map<int, set<int>>& InitLNS::findConnectedComponent(
    const vector<set<int>>& graph, int vertex,
    unordered_map<int, set<int>>& sub_graph) {
    std::queue<int> Q;
    Q.push(vertex);
    sub_graph.emplace(vertex, graph[vertex]);
    while (!Q.empty()) {
        auto v = Q.front();
        Q.pop();
        for (const auto& u : graph[v]) {
            auto ret = sub_graph.emplace(u, graph[u]);
            if (ret.second)  // insert successfully
                Q.push(u);
        }
    }
    return sub_graph;
}

void InitLNS::printPath() const {
    for (const auto& agent : agents)
        cout << "Agent " << agent.id << ": " << agent.path << endl;
}

void InitLNS::printResult() {
    // average_group_size = - iteration_stats.front().num_of_agents;
    // for (const auto& data : iteration_stats)
    //     average_group_size += data.num_of_agents;
    // if (average_group_size > 0)
    //     average_group_size /= (double)(iteration_stats.size() - 1);
    assert(!iteration_stats.empty());
    stringstream sout;
    sout << "\t"
         << "g " << solution_shared->myGroup << " "
         << "pe " << myId << " " << getSolverName() << ": "
         << "runtime " << runtime << " "
         << "iterations " << num_of_iteration << " "
         << "colliding " << num_of_colliding_pairs << " "
         << "initial colliding "
         << iteration_stats.front().num_of_colliding_pairs << " "
         << "cost " << sum_of_costs << " "
         << "initial cost " << iteration_stats.front().sum_of_costs << " "
         << "failed " << num_of_failures << " "
         << num_of_failures * 1. / (1e-6 + num_of_iteration) << " T_init "
         << simulated_annealing::sa_iteration_T_init << " "
         << "restart " << num_of_restart << " " << endl;
    printf("%s", sout.str().c_str());
}

void InitLNS::clear() {
    path_table.clear();
    collision_graph.clear();
    goal_table.clear();
    num_of_colliding_pairs = MAX_COST;
    sum_of_costs = MAX_COST;
    path_table.sum_of_costs = MAX_COST;
    path_table.num_of_colliding_pairs = MAX_COST;
}

void InitLNS::reset() {
    // goal_table不用clear
    iteration_stats.clear();
    path_table.reset();
    for (int i = 0; i < (int)collision_graph.size(); i++)
        collision_graph[i].clear();
    sum_of_costs = MAX_COST;
    num_of_colliding_pairs = MAX_COST;
    path_table.sum_of_costs = MAX_COST;
    path_table.num_of_colliding_pairs = MAX_COST;
}

bool InitLNS::validatePathTable() const {
    for (auto i = 0; i < agents.size(); i++)
        assert(path_table.getPath(i) == &agents[i].path);
    return true;
}

// Interrupt the solving, so it can be started again with new assumptions
void InitLNS::setSolverInterrupt() {
    if (screen > 0)
        printf("\t--> g %d pe %d initlns:1522 set collision_interrupt 1\n", 
            myGroup, myId);
    solution_shared->phase_collision_asynch_interrupt = true;
}

void InitLNS::unsetSolverInterrupt() {
    if (screen > 0)
        printf("\t--> g %d pe %d initlns:1527 set collision_interrupt 0\n", 
            myGroup, myId);
    solution_shared->phase_collision_asynch_interrupt = false;
}