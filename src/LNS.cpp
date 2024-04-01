#include "LNS.h"

#include <queue>

#include "ECBS.h"
#include "Logger.h"
#include "defines.h"

#include "lacam3/include/translator.hpp"
#include "lacam3/include/refiner.hpp"
#include "lacam3/include/lacam.hpp"

LNS::LNS(const Instance& instance, double time_limit,
         const string& init_algo_name, const string& init_replan_algo_name,
         const string& replan_algo_name, const string& destory_name,
         int neighbor_size, int max_num_of_iterations, bool use_init_lns,
         const string& init_destory_name, int use_sipp, int screen,
         PIBTPPS_option pipp_option, SolutionShared* solution_shared,
         bool preprocess_LL_engines)
    : BasicLNS(instance, time_limit, neighbor_size, screen),
      init_algo_name(init_algo_name),
      init_replan_algo_name(init_replan_algo_name),
      replan_algo_name(replan_algo_name),
      max_num_of_iterations(max_num_of_iterations),
      use_init_lns(use_init_lns),
      init_destory_name(init_destory_name),
      path_table(instance.map_size),
      pipp_option(pipp_option) {
    this->solution_shared = solution_shared;
    start_time = getTime();
    replan_time_limit = time_limit / 100;
    if (destory_name == "Adaptive") {
        ALNS = true;
        destroy_weights.assign(DESTORY_COUNT, 1);
        decay_factor = 0.01;
        reaction_factor = 0.01;
    } else if (destory_name == "RandomWalk")
        destroy_strategy = RANDOMWALK;
    else if (destory_name == "Intersection")
        destroy_strategy = INTERSECTION;
    else if (destory_name == "Random")
        destroy_strategy = RANDOMAGENTS;
    else {
        cerr << "Destroy heuristic " << destory_name << " does not exists. "
             << endl;
        exit(-1);
    }
    preprocessing_time = getTime() - start_time;
    if (screen > 0)
        cout << "g " << myGroup << " pe " << myId << " "
             << "Pre-processing time = " << preprocessing_time << " seconds."
             << endl;

    int N = instance.getDefaultNumberOfAgents();
    agents.reserve(N);

    for (int i = 0; i < N; i++) {
        agents.emplace_back(instance, i, use_sipp, preprocess_LL_engines);
    }

    if(preprocess_LL_engines) {
        int np_omp = param_num_solver;
        if(np_omp == 1)
            np_omp = 32;
        printf("init my_heuristic using %d thread\n", np_omp);
        // #pragma omp parallel for num_threads(param_num_solver)
        #pragma omp parallel for num_threads(np_omp)
        for (int i = 0; i < N; i++) {
            agents[i].path_planner->init();
        }
    }

    for (int i = 0; i < N; i++) {
        agents[i].path_planner->phase_collision_asynch_interrupt =
            &solution_shared->phase_collision_asynch_interrupt;
        agents[i].path_planner->phase_cost_asynch_interrupt =
            &solution_shared->phase_cost_asynch_interrupt;
        preprocessing_time = getTime() - start_time;
        if (i % 100 == 0 && screen > 0)
            cout << "agenti " << i << " "
                 << "Pre-processing time = " << preprocessing_time
                 << " seconds." << endl;
    }
    preprocessing_time = getTime() - start_time;
    if (screen > 0)
        cout << "pe " << myId << " "
             << "Pre-processing time = " << preprocessing_time << " seconds."
             << endl;
}

bool LNS::run_with_restart() {
    float wh_phase_collision = tl_single_agent_solver_f_w;
    SolutionShared *phase_collision_solution_shared = solution_shared;

    while(true) {
        run();

        if(max_num_of_iterations == 0) break;

        restart_times++;
        if (myId == 0) solution_shared->num_restart++;
        if (screen >= 0 && myId == 0) {
            stringstream sstream;
            runtime = getTime() - start_time;
            sstream << "g " << myGroup << " pe " << myId << " CostLNS Restart "
                 << restart_times << " Iteration " << num_of_iteration
                 << " failed " << num_of_failures << " "
                 << num_of_failures * 1. / (1e-5 + num_init_iteration)
                 << " cons_fail " << num_of_consecutive_failures << " cost "
                 << sum_of_costs << " remaining " << time_limit - runtime
                 << endl;
            printf("%s", sstream.str().c_str());
        }

        solution_shared->phase_cost_need_restart = true;

        if(group_mode_phase_cost == "one")
            pthread_barrier_wait(&share_manager->barrier_all_threads_restart1);
        else if(!run_costlns_only)
            pthread_barrier_wait(&solution_shared->barrier_phase_cost_restart1);

        // 切换成初始的group和weight
        tl_single_agent_solver_f_w = wh_phase_collision;
        solution_shared = phase_collision_solution_shared;
        if(!run_costlns_only) {
            myGroup = solution_shared->myGroup;
            solution_shared->numLocalSolver = numLocalSolver;
        }

        // 需要重新绑定interrupt，这里容易出错
        for (int i = 0; i < (int)agents.size(); i++) {
            agents[i].path_planner->phase_collision_asynch_interrupt =
                &solution_shared->phase_collision_asynch_interrupt;
            agents[i].path_planner->phase_cost_asynch_interrupt =
                &solution_shared->phase_cost_asynch_interrupt;
        }

        // reset shared data
        if(myId == 0 && !run_costlns_only) {
            solution_shared->reset();
        }
        share_manager->best_group_index = -1;

        // reset the solver data
        for (int i = 0; i < agents.size(); i++) {
            agents[i].path.clear();
            agents[i].state.clear();
        }
        neighbor.clear();
        path_table.reset();
        iteration_stats.clear();
        initial_sum_of_costs = MAX_COST;
        sum_of_costs = MAX_COST;
        path_table.sum_of_costs = MAX_COST;

        if(group_mode_phase_cost == "one")
            pthread_barrier_wait(&share_manager->barrier_all_threads_restart2);
        else if(!run_costlns_only)
            pthread_barrier_wait(&solution_shared->barrier_phase_cost_restart2);

        printf("g %d pe %d -------------end \n", myGroup, myId);

        bool need_break = false;
        if (num_of_iteration > 1 &&
            solution_shared->phase_cost_asynch_interrupt) {
            is_running = 60;
            if (screen >= 0) {
                stringstream sstream;
                sstream << "g " << myGroup << " "
                        << "pe " << myId 
                        << " need_break phase_cost_asynch_interrupt is true " << endl;
                printf("%s", sstream.str().c_str());
            }
            need_break = true;
        }
        runtime = getTime() - start_time;
        if (runtime > time_limit ||
            num_of_iteration > max_num_of_iterations) {
            is_running = 61;
            if (screen >= 0)
                cout << "g " << solution_shared->myGroup << " " << "pe " << myId
                     << " need_break timeout " << runtime << endl;
            need_break = true;
        }
        if (share_manager->interrupte_mem_out) {
            is_running = 62;
            if (screen >= 0)
                cout << "g " << solution_shared->myGroup << " " << "pe " << myId
                     << " need_break interrupte_mem_out " << endl;
            need_break = true;
        }
        if (need_break) {
            break;
        }
    }
    return true;
}

bool LNS::run() {
    if(test_speed_mode) {
        run_collision_lns();
        return false;
    }
    is_running = 1;
    // only for statistic analysis, and thus is not included in runtime
    // 可以用来计算subopt
    sum_of_distances = 0;
    for (const auto& agent : agents) {
        sum_of_distances +=
            agent.path_planner
                ->p_my_heuristic->at(agent.path_planner->start_location);
    }
    start_time = share_manager->time_phase_collision_start;

    initial_solution_runtime = 0;

    if(!run_costlns_only) {
        solution_shared->sum_of_distances = sum_of_distances;

        run_initial_solver();
        run_collision_lns();

        for (int i = 0; i < agents.size(); i++) {
            agents[i].state.tid = threadId;
            agents[i].state.iter = -1;
        }

        // 同步initial结果
        if (solution_shared->use_share_phase_cost) {
            pthread_barrier_wait(&solution_shared->barrier_phase_cost_start1);
            if(sum_of_costs < MAX_COST) {
                solution_shared->do_share_phase_cost(myId, path_table, agents,
                                                destroy_weights);
                sum_of_costs = path_table.sum_of_costs;
            }
            pthread_barrier_wait(&solution_shared->barrier_phase_cost_start2);
            solution_shared->do_share_phase_cost(myId, path_table, agents,
                                            destroy_weights);
            sum_of_costs = path_table.sum_of_costs;
            pthread_barrier_wait(&solution_shared->barrier_phase_cost_start3);
            if (screen > 0)
                printf("同步initial结果 cost %d solution_shared->cost %d\n", 
                    sum_of_costs, solution_shared->sum_of_costs);
        }
    }

    if(!regroup_solvers()) return false;

    //让low-level a*能够继续运行，只有两个阶段的interrupt都false才可以
    if (screen > 0)
        printf("\t--> g %d pe %d lns run():200 set phase_collision_asynch_interrupt 0\n", 
            myGroup, myId);
    solution_shared->phase_collision_asynch_interrupt = false;
    solution_shared->phase_cost_asynch_interrupt = false;

    iteration_stats.emplace_back(neighbor.agents.size(), initial_sum_of_costs,
                                 initial_solution_runtime, init_algo_name);
    runtime = initial_solution_runtime;
    if (sum_of_costs < MAX_COST) {
        if (screen >= 1)
            cout << "g " << myGroup << " pe " << myId << " "
                 << "Initial solution cost = " << initial_sum_of_costs << ", "
                 << "sum_of_costs = " << sum_of_costs << ", "
                 << "runtime = " << initial_solution_runtime << endl;
    } else {
        stringstream sout;
        sout << "g " << myGroup << " pe " << myId << " "
             << "Failed to find an initial solution in " << runtime
             << " seconds after  " << restart_times << " restarts" << endl;
        printf("%s", sout.str().c_str());
        is_running = 4;
        return false;  // terminate because no initial solution is found
    }

    // reweight the sipps
    if (param_use_fixed_wh1_phase_cost == 1)
        tl_single_agent_solver_f_w = 1.;
    else if (param_use_fixed_wh1_phase_cost == 3)
        tl_single_agent_solver_f_w = 1 + (tl_single_agent_solver_f_w - 1) / 5.;
    else if (param_use_fixed_wh1_phase_cost == 4)
        tl_single_agent_solver_f_w = 1 + (tl_single_agent_solver_f_w - 1) / 10.;
    if (screen > 0)
        printf("g %d pe %d tl_single_agent_solver_f_w %f\n", myGroup, myId,
            tl_single_agent_solver_f_w);

    if (param_use_simulated_annealing_phase_cost) {
        simulated_annealing::reset_temp(param_simulated_annealing_T_phase_cost, 1,
                                        param_sa_iteration_coolfactor_phase_cost);
    }
    run_cost_lns();

    return true;
}


void LNS::run_initial_solver() {
    // 初始化solution
    sum_of_costs = MAX_COST;
    succ_phase_collision = false;
    succ_phase_collision = getInitialSolution();

    initial_solution_runtime = getTime() - start_time;
    stringstream sout;
    sout << "g " << myGroup << " pe " << myId << " succ " << succ_phase_collision
         << " initial_solution_runtime " << initial_solution_runtime
         << " cost " << neighbor.sum_of_costs
         << " time_limit " << time_limit << endl;
    printf("%s", sout.str().c_str());
    if (succ_phase_collision || solution_shared->use_share_phase_cost) {
        double t1 = getTime();
        if (succ_phase_collision) {
            path_table.sum_of_costs = sum_of_costs;
            // solution_shared->do_share_phase_cost(myId, path_table, agents, destroy_weights);

            // PathTableWC path_table_wc;
            // vector<set<int>> collision_graph;
            // solution_shared->do_share_phase_collision(
            //     myId, path_table_wc, agents, collision_graph, destroy_weights, true);

        }
        // pthread_barrier_wait(&solution_shared->barrier_phase_collision_result);
        // 当succ为true时，就可以停掉其他正在运行的collision phase了
        if (solution_shared->use_share_phase_cost &&
            solution_shared->sum_of_costs < MAX_COST) {
            if (screen > 0)
                printf("\t--> g %d pe %d lns:269 set collision_interrupt 1\n", myGroup, myId);
            solution_shared->phase_collision_asynch_interrupt = true;  // phase_collision can stop
            // solution_shared->do_share_phase_cost(myId, path_table, agents,
            //                                  destroy_weights);
            // sum_of_costs = path_table.sum_of_costs;
            succ_phase_collision = true;
        }
        runtime_share_data += getTime() - t1;
    }
}

void LNS::run_collision_lns() {
    if (use_init_lns) {
        if(init_lns == nullptr) {
            init_lns = new InitLNS(
                instance, agents, time_limit - initial_solution_runtime,
                init_replan_algo_name, init_destory_name, neighbor_size, screen);
            init_lns->myId = myId;
            init_lns->myGroup = myGroup;
            init_lns->numLocalSolver = numLocalSolver;
            init_lns->solution_shared = solution_shared;
            init_lns->use_sync_mode_in_iteration = use_sync_mode_in_iteration;
            init_lns->is_running = &is_running;
            init_lns->test_speed_mode = test_speed_mode;
            init_lns->start_time = start_time;
        }

        if (!succ_phase_collision && initial_solution_runtime < time_limit) {
            is_running = 2;

            succ_phase_collision = init_lns->run();
            if(test_speed_mode) {
                init_lns->reset();
                init_lns->num_of_iteration = 1;
                return;
            } 

            num_init_iteration = init_lns->num_of_iteration;
            num_of_ll_search += init_lns->num_of_ll_search;
            runtime_share_data += init_lns->runtime_share_data;
            // accept new paths
            if (succ_phase_collision) {
                if (init_lns->sum_of_costs < MAX_COST) {
                    path_table.reset();
                    for (const auto& agent : agents) {
                        path_table.insertPath(agent.id, agent.path);
                    }
                    initial_sum_of_costs = init_lns->sum_of_costs;
                    sum_of_costs = initial_sum_of_costs;
                    path_table.sum_of_costs = sum_of_costs;
                } else {
                    succ_phase_collision = false;
                    // assert(solution_shared->num_of_colliding_pairs == 0);
                    // solution_shared->read_global_data(path_table, agents);
                    // sum_of_costs = path_table.sum_of_costs;
                    // initial_sum_of_costs = sum_of_costs;
                    // assert(sum_of_costs < MAX_COST);
                }
                // validateSolution();
            }
            if (screen > 0)
                cout << "g " << myGroup << " pe " << myId << " "
                    << "succ_phase_collision " << succ_phase_collision << " "
                    << "Initial solution cost = " << sum_of_costs << ", "
                    << endl;
            initial_solution_runtime = getTime() - start_time;
            is_running = 3;
            // init_lns->clear();
            init_lns->reset();
        } else
            init_lns->runWait();
    } else  // use random restart
    {
        // 当测试PP-restart时再用这个功能
        while (!succ_phase_collision && initial_solution_runtime < time_limit && false) {
            succ_phase_collision = getInitialSolution();
            initial_solution_runtime = getTime() - start_time;
            restart_times++;
            solution_shared->num_restart++;
        }
    }
}

bool LNS::regroup_solvers() {
    if (run_costlns_only) {
        while (true) {
            if (stop_costlns_only_solver) break;
            if (runtime > time_limit) break;
            int index = share_manager->best_group_index.load();
            if (index == -1) continue;
            if (share_manager->solutions[index]->numLocalSolver ==
                param_num_solver_init)
                break;
            double runtime = getTime() - start_time;
            sleep(1);
        }
    }

    if (run_costlns_only || solution_shared->use_share_phase_cost &&
        group_mode_phase_cost == "one" && numGroup > 1) {
        if (all_run_next_if_one_solution_found) {
            //停掉其他的group
            for (auto ss : share_manager->solutions)
                ss->phase_collision_asynch_interrupt = true;

            //等待
            pthread_barrier_wait(&share_manager->barrier_stop_all_init_solver);

            //让low-level a*能够继续运行，只有两个阶段的interrupt都false才可以
            for (auto ss : share_manager->solutions)
                ss->phase_collision_asynch_interrupt = false;
        }
        //停掉比当前组的wh高的还没有得到解的组
        for (auto ss : share_manager->solutions) {
            if (ss->num_of_colliding_pairs > 0 && ss->myGroup > myGroup &&
                !ss->has_eecbs_solver)
                ss->phase_collision_asynch_interrupt = true;
        }

        //找到当前最好的group
        // share_manager->print();
        solution_shared = share_manager->get_current_best_group_solution();
        if (share_manager->best_group_index == -1) {
            is_running = 42;
            printf("g %d pe %d index %d Failed %ld\n", myGroup, myId,
                   share_manager->best_group_index.load(), solution_shared);
            return false;  // terminate because no initial solution is found
        }

        if (screen >= 0 || true) {
            printf("\t--> g %d pe %d best_solution g %d\n", myGroup, myId,
                   solution_shared->myGroup);
        }
        // succ = true;

        //停掉0号group，让其参与phase_cost计算
        if (group_mode_phase_collision == "central") {
            share_manager->solutions[0]
                ->phase_collision_asynch_interrupt = true;
        }
        //当前group继续运行
        if (screen > 0)
            printf("\t--> g %d pe %d regroup set phase_collision_asynch_interrupt 0 in g %d\n", 
                myGroup, myId, solution_shared->myGroup);
        solution_shared->phase_collision_asynch_interrupt = false;

        // 需要重新绑定interrupt，这里容易出错
        for (int i = 0; i < (int)agents.size(); i++) {
            agents[i].path_planner->phase_collision_asynch_interrupt =
                &solution_shared->phase_collision_asynch_interrupt;
            agents[i].path_planner->phase_cost_asynch_interrupt =
                &solution_shared->phase_cost_asynch_interrupt;
        }

        // accept new paths, 这个有优化空间，可以在下面跑几轮，看结果自己更新
        if (run_costlns_only ||
            myGroup != solution_shared->myGroup && !succ_phase_collision) {
            solution_shared->read_global_data(path_table, agents);
            sum_of_costs = path_table.sum_of_costs;
            initial_sum_of_costs = sum_of_costs;
            if(sum_of_costs < MAX_COST)
                succ_phase_collision = true;
        }
        if(!succ_phase_collision) {
            printf("g %d pe %d succ_phase_collision 0\n", myGroup, myId);
            return false;
        }
        printf("validateSolution LNS:469\n");
        validateSolution();

        if (run_costlns_only) {
            solution_shared->numLocalSolver++; 
        } else if (myGroup != solution_shared->myGroup) {
            solution_shared->numLocalSolver++; 
            share_manager->solutions[myGroup]->numLocalSolver--;
            assert(share_manager->solutions[myGroup]->numLocalSolver >= 0);
        }
    }
    else {
        share_manager->best_group_index = 0;
    }
    return true;
}

void LNS::run_cost_lns() {

    num_of_failures = 0;
    num_of_consecutive_failures = 0;
    float cons_fail_rate = 0;
    num_of_iteration = 1;
    printf("validateSolution LNS:491\n");
    validateSolution();
    solution_shared->num_of_colliding_pairs = 0;
    solution_shared->colliding_pairs_4_print = 0;
    int succ = true;
    while (true) {
        is_running = 5;
        if (screen >= 1) {
            stringstream sout;
            sout << "g " << myGroup << " pe " << myId << " "
                 << "Iteration " << num_of_iteration << ", "
                 << "group size = " << neighbor.agents.size() << ", "
                 << "solution cost = " << sum_of_costs << ", "
                 << "cons fail = " << cons_fail_rate << ", "
                 << "remaining time = " << time_limit - runtime << endl;
            printf("%s", sout.str().c_str());
        }
        runtime = getTime() - start_time;
        bool need_break = false;
        if (num_of_iteration > 1 &&
            solution_shared->phase_cost_asynch_interrupt) {
            is_running = 6;
            if (screen > 0) {
                stringstream sout;
                sout << "g " << myGroup << " " << "pe " << myId 
                << " solution_shared->phase_cost_asynch_interrupt is true " << endl;
                printf("%s", sout.str().c_str());
            }
            need_break = true;
        }
        if (runtime > time_limit ||
            num_of_iteration > max_num_of_iterations) {
            is_running = 7;
            if (screen > 0) {
                stringstream sout;
                sout << "g " << solution_shared->myGroup << " " << "pe " << myId
                    << " costlns end because timeout or max_num_of_iterations " << endl;
                printf("%s", sout.str().c_str());
            }
            need_break = true;
        }
        if (num_of_iteration > 3000 && cons_fail_rate > param_costlns_max_con_fail ||
            solution_shared->phase_cost_need_restart) {
            is_running = 62;
            if (screen >= 0 && myId == 0 || screen > 0) {
                stringstream sout;
                sout << "g " << solution_shared->myGroup
                     << " pe " << myId << " costlns end because fail_rate > " 
                     << param_costlns_max_con_fail << endl;
                printf("%s", sout.str().c_str());
            }
            need_break = true;
        }
        if (need_break) {
            break;
        }
        if (solution_shared->use_share_phase_cost && use_sync_mode_in_iteration) {
            // cout << "g " << myGroup << " pe " << myId << " " << "Iteration "
            // << num_of_iteration << " "
            //     << solution_shared->num_iter_update_phase_collision
            //     << " numLocalSolver " << numLocalSolver
            //     << endl;
            if (num_of_iteration * numLocalSolver >
                solution_shared->num_iter_update_phase_cost)
                continue;
            // if(num_of_iteration > 1)
            //     path_table.sum_of_costs = MAX_COST;
            double t1 = getTime();
            solution_shared->do_share_phase_cost(myId, path_table, agents,
                                             destroy_weights);
            runtime_share_data += getTime() - t1;
            sum_of_costs = path_table.sum_of_costs;
        }
        if (screen >= 1 && num_of_iteration > 1) validateSolution();
        if (ALNS) chooseDestroyHeuristicbyALNS();

        switch (destroy_strategy) {
            case RANDOMWALK:
                succ = generateNeighborByRandomWalk();
                break;
            case INTERSECTION:
                succ = generateNeighborByIntersection();
                break;
            case RANDOMAGENTS:
                neighbor.agents.resize(agents.size());
                for (int i = 0; i < (int)agents.size(); i++)
                    neighbor.agents[i] = i;
                if (neighbor.agents.size() > neighbor_size) {
                    std::random_shuffle(neighbor.agents.begin(),
                                        neighbor.agents.end());
                    neighbor.agents.resize(neighbor_size);
                }
                succ = true;
                break;
            default:
                cerr << "Wrong neighbor generation strategy" << endl;
                exit(-1);
        }
        if (!succ) continue;

        // store the neighbor information
        neighbor.old_paths.resize(neighbor.agents.size());
        neighbor.old_state.resize(neighbor.agents.size());
        neighbor.old_sum_of_costs = 0;
        for (int i = 0; i < (int)neighbor.agents.size(); i++) {
            if (replan_algo_name == "PP") {
                neighbor.old_paths[i] = agents[neighbor.agents[i]].path;
                neighbor.old_state[i] = agents[neighbor.agents[i]].state;
            }
            path_table.deletePath(neighbor.agents[i],
                                  agents[neighbor.agents[i]].path);
            neighbor.old_sum_of_costs +=
                agents[neighbor.agents[i]].path.size() - 1;
            // if(myId == 1) {
            //     cout << " neighbor.agents " << neighbor.agents[i] << endl;
            // }
        }

        if (replan_algo_name == "EECBS")
            succ = runEECBS();
        else if (replan_algo_name == "CBS")
            succ = runCBS();
        else if (replan_algo_name == "PP")
            succ = runPP();
        else {
            cerr << "LNS Wrong replanning strategy " << replan_algo_name
                 << endl;
            cerr << "Need be in EECBS, CBS, PP" << endl;
            exit(-1);
        }
        if(succ)
            num_of_consecutive_failures = 0;
        else
            num_of_consecutive_failures++;

        if (screen > 0) {
            if (destroy_weights.empty()) {
                cerr << "g " << myGroup << " pe " << myId
                     << " destroy_weights empty " << endl;
                exit(-1);
            }
        }

        if (ALNS) {  // update destroy heuristics
            if (neighbor.old_sum_of_costs > neighbor.sum_of_costs)
                destroy_weights[selected_neighbor] =
                    reaction_factor *
                        (neighbor.old_sum_of_costs - neighbor.sum_of_costs) /
                        neighbor.agents.size() +
                    (1 - reaction_factor) * destroy_weights[selected_neighbor];
            else
                destroy_weights[selected_neighbor] =
                    (1 - decay_factor) * destroy_weights[selected_neighbor];
            double sum = 0;
            for (auto v : destroy_weights) sum += v;
            for (int i = 0; i < destroy_weights.size(); i++) {
                destroy_weights[i] /= sum;
                destroy_weights[i] += 1e-3;
            }
        }
        runtime = getTime() - start_time;
        sum_of_costs += neighbor.sum_of_costs - neighbor.old_sum_of_costs;
        path_table.sum_of_costs = sum_of_costs;
        cons_fail_rate = num_of_consecutive_failures * 1. / num_of_iteration;
        if(myId == 0)
            solution_shared->num_of_consecutive_failure_rate = cons_fail_rate;
        if (screen >= 1 && false)
            cout << "g " << myGroup << " pe " << myId << " "
                 << "Iteration " << num_of_iteration << ", "
                 << "group size = " << neighbor.agents.size() << ", "
                 << "solution cost = " << sum_of_costs << ", "
                 << " cons_fail " << num_of_consecutive_failures 
                 << "remaining time = " << time_limit - runtime << endl;
        if (solution_shared != 0 &&
            num_of_iteration % param_share_step_phase_cost == 0) {
            is_running = 8;
            double t1 = getTime();
            solution_shared->do_share_phase_cost(myId, path_table, agents,
                                             destroy_weights);
            runtime_share_data += getTime() - t1;
            if(sum_of_costs > path_table.sum_of_costs)
                num_of_consecutive_failures = 0;
            sum_of_costs = path_table.sum_of_costs;
        }
        num_of_iteration++;
        if(num_of_iteration < 100)
            iteration_stats.emplace_back(neighbor.agents.size(), sum_of_costs,
                                        runtime, replan_algo_name);
        neighbor.clear();
        is_running = 9;
    }

    printf("validateSolution LNS:682\n");
    validateSolution();
    average_group_size = -iteration_stats.front().num_of_agents;
    for (const auto& data : iteration_stats)
        average_group_size += data.num_of_agents;
    if (average_group_size > 0)
        average_group_size /= (double)(num_of_iteration - 1);

    stringstream sout;
    sout << "th " << threadId << " "
         << "g " << solution_shared->myGroup << " "
         << "pe " << myId << " " << getSolverName() << ": "
         << "runtime " << runtime << ", "
         << "iterations " << num_of_iteration << ", "
         << "solution cost " << sum_of_costs << ", "
         << "initial solution cost " << initial_sum_of_costs << ", "
         << "failed iterations " << num_of_failures << " "
         << num_of_failures * 1. / num_of_iteration << endl;
    printf("%s", sout.str().c_str());
    is_running = 10;
}

bool LNS::getInitialSolution() {
    // cout << "getInitialSolution " << init_algo_name << " ... " << endl;
    neighbor.agents.resize(agents.size());
    for (int i = 0; i < (int)agents.size(); i++) neighbor.agents[i] = i;
    neighbor.old_sum_of_costs = MAX_COST;
    neighbor.sum_of_costs = 0;
    bool succ = false;
    if (init_algo_name == "EECBS")
        succ = runEECBS();
    else if (init_algo_name == "lacam")
        succ = runlacam();
    else if (init_algo_name == "PP")
        succ = runPP();
    else if (init_algo_name == "PIBT")
        succ = runPIBT();
    else if (init_algo_name == "PPS")
        succ = runPPS();
    else if (init_algo_name == "winPIBT")
        succ = runWinPIBT();
    else if (init_algo_name == "CBS")
        succ = runCBS();
    else {
        cerr << "Initial MAPF solver " << init_algo_name << " does not exist!"
             << endl;
        exit(-1);
    }
    // cout << "getInitialSolution " << init_algo_name << " succ " << succ <<
    // endl;
    if (succ) {
        initial_sum_of_costs = neighbor.sum_of_costs;
        sum_of_costs = neighbor.sum_of_costs;
        return true;
    } else {
        return false;
    }
}

bool LNS::runEECBS() {
    vector<SingleAgentSolver*> search_engines;
    search_engines.reserve(neighbor.agents.size());
    for (int i : neighbor.agents) {
        search_engines.push_back(agents[i].path_planner);
    }

    is_running = 111;
    ECBS ecbs(search_engines, screen - 1, &path_table);
    ecbs.setPrioritizeConflicts(true);
    ecbs.setDisjointSplitting(false);
    ecbs.setBypass(true);
    ecbs.setRectangleReasoning(true);
    ecbs.setCorridorReasoning(true);
    ecbs.setHeuristicType(heuristics_type::WDG, heuristics_type::GLOBAL);
    ecbs.setTargetReasoning(true);
    ecbs.setMutexReasoning(false);
    ecbs.setConflictSelectionRule(conflict_selection::EARLIEST);
    ecbs.setNodeSelectionRule(node_selection::NODE_CONFLICTPAIRS);
    ecbs.setSavingStats(false);
    ecbs.phase_collision_asynch_interrupt = &solution_shared->phase_collision_asynch_interrupt;
    ecbs.phase_cost_asynch_interrupt = &solution_shared->phase_cost_asynch_interrupt;
    double w;
    if (iteration_stats.empty()) {
        w = 5;  // initial run
        cout << " eecbs initial run w=" << w << endl;
    } else
        w = 1.1;  // replan
    ecbs.setHighLevelSolver(high_level_solver_type::EES, w);
    runtime = getTime() - start_time;
    double T = time_limit - runtime;
    // T = min(T, 10.);
    if (!iteration_stats.empty())  // replan
        T = min(T, replan_time_limit);
    cout << " eecbs T=" << T << endl;
    bool succ = ecbs.solve(T*param_num_solver, 0);  // ecbs内部使用的是clock，这里简单处理乘以numLocalSolver
    if (succ &&
        ecbs.solution_cost < neighbor.old_sum_of_costs)  // accept new paths
    {
        auto id = neighbor.agents.begin();
        for (size_t i = 0; i < neighbor.agents.size(); i++) {
            agents[*id].path = *ecbs.paths[i];
            path_table.insertPath(agents[*id].id, agents[*id].path);
            ++id;
        }
        neighbor.sum_of_costs = ecbs.solution_cost;
        if (sum_of_costs_lowerbound < 0)
            sum_of_costs_lowerbound = ecbs.getLowerBound();
    } else  // stick to old paths
    {
        if (!neighbor.old_paths.empty()) {
            for (int id : neighbor.agents) {
                path_table.insertPath(agents[id].id, agents[id].path);
            }
            neighbor.sum_of_costs = neighbor.old_sum_of_costs;
        }
        if (!succ) num_of_failures++;
    }
    return succ;
}
bool LNS::runCBS() {
    if (screen >= 2)
        cout << "old sum of costs = " << neighbor.old_sum_of_costs << endl;
    vector<SingleAgentSolver*> search_engines;
    search_engines.reserve(neighbor.agents.size());
    for (int i : neighbor.agents) {
        search_engines.push_back(agents[i].path_planner);
    }

    CBS cbs(search_engines, screen - 1, &path_table);
    cbs.setPrioritizeConflicts(true);
    cbs.setDisjointSplitting(false);
    cbs.setBypass(true);
    cbs.setRectangleReasoning(true);
    cbs.setCorridorReasoning(true);
    cbs.setHeuristicType(heuristics_type::WDG, heuristics_type::ZERO);
    cbs.setTargetReasoning(true);
    cbs.setMutexReasoning(false);
    cbs.setConflictSelectionRule(conflict_selection::EARLIEST);
    cbs.setNodeSelectionRule(node_selection::NODE_CONFLICTPAIRS);
    cbs.setSavingStats(false);
    cbs.setHighLevelSolver(high_level_solver_type::ASTAR, 1);
    runtime = getTime() - start_time;
    double T = time_limit - runtime;  // time limit
    if (!iteration_stats.empty())     // replan
        T = min(T, replan_time_limit);
    bool succ = cbs.solve(T, 0);
    if (succ &&
        cbs.solution_cost <= neighbor.old_sum_of_costs)  // accept new paths
    {
        auto id = neighbor.agents.begin();
        for (size_t i = 0; i < neighbor.agents.size(); i++) {
            agents[*id].path = *cbs.paths[i];
            path_table.insertPath(agents[*id].id, agents[*id].path);
            ++id;
        }
        neighbor.sum_of_costs = cbs.solution_cost;
        if (sum_of_costs_lowerbound < 0)
            sum_of_costs_lowerbound = cbs.getLowerBound();
    } else  // stick to old paths
    {
        if (!neighbor.old_paths.empty()) {
            for (int id : neighbor.agents) {
                path_table.insertPath(agents[id].id, agents[id].path);
            }
            neighbor.sum_of_costs = neighbor.old_sum_of_costs;
        }
        if (!succ) num_of_failures++;
    }
    return succ;
}

bool LNS::runPP() {
    is_running = 101;
    auto shuffled_agents = neighbor.agents;
    std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());
    if (screen >= 2) {
        for (auto id : shuffled_agents)
            cout << id << "("
                 << agents[id]
                        .path_planner
                        ->p_my_heuristic->at(agents[id].path_planner->start_location)
                 << "->" << agents[id].path.size() - 1 << "), ";
        cout << endl;
    }
    int remaining_agents = (int)shuffled_agents.size();
    auto p = shuffled_agents.begin();
    neighbor.sum_of_costs = 0;
    runtime = getTime() - start_time;
    double T = time_limit - runtime;  // time limit
    if (!iteration_stats.empty())     // replan
        T = min(T, replan_time_limit);
    auto time = getTime();
    ConstraintTable constraint_table(instance.num_of_cols, instance.map_size,
                                     &path_table);
    while (p != shuffled_agents.end() &&
           getTime() - time < T &&
           !solution_shared->phase_cost_asynch_interrupt) {
        int id = *p;
        if (screen >= 3)
            cout << "Remaining agents = " << remaining_agents
                 << ", remaining time = "
                 << T - (getTime() - time) << " seconds. "
                 << endl
                 << "Agent " << agents[id].id << endl;
        agents[id].path = agents[id].path_planner->findPath(constraint_table);
        agents[id].state.tid = threadId;
        agents[id].state.iter = num_of_iteration;
        num_of_ll_search++;
        if (agents[id].path.empty()) break;
        if (solution_shared->phase_cost_asynch_interrupt) break;
        neighbor.sum_of_costs += (int)agents[id].path.size() - 1;
        if (!param_use_simulated_annealing_phase_cost) {
            if (neighbor.sum_of_costs >= neighbor.old_sum_of_costs) break;
        }
        is_running = 102;
        remaining_agents--;
        path_table.insertPath(agents[id].id, agents[id].path);
        if (shuffled_agents.size() >= 1000) {
            if (solution_shared->num_of_init_remaining_agents > remaining_agents)
                solution_shared->num_of_init_remaining_agents = remaining_agents;
        }
        if (myId == 1 && shuffled_agents.size() < 32 && false) {
            Agent& a2 = agents[id];
            for (int a1_id = 0; a1_id < agents.size(); a1_id++) {
                if (id == a1_id) continue;
                bool flag = false;
                for (auto a : shuffled_agents) {
                    // cout << " --" << id << " " << a1_id  << " " << a << "
                    // size " << shuffled_agents.size() << endl;
                    if (a1_id == a) {
                        flag = true;
                        break;
                    }
                }
                // cout << " " << id << " " << a1_id  << " " << flag << endl;
                if (flag) continue;
                Agent& a1 = agents[a1_id];
                int target = a1.path.back().location;
                for (int t = 0; t < (int)a2.path.size(); t++) {
                    if (a2.path[t].location == target) { // target conflict
                        cerr << "pe " << myId
                             << " Find a target conflict where agent " << a2.id
                             << " (of length " << a2.path.size() - 1
                             << ") traverses agent " << a1.id << " (of length "
                             << a1.path.size() - 1 << ")'s target location "
                             << target << " at timestep " << t << endl;
                        exit(-1);
                    }
                }
            }
        }
        ++p;
    }
    if (remaining_agents == 0 &&
        neighbor.sum_of_costs <= neighbor.old_sum_of_costs)  // accept new paths
    {
        is_running = 103;
        return true;
    } else  // stick to old paths
    {
        if (p != shuffled_agents.end()) num_of_failures++;
        auto p2 = shuffled_agents.begin();
        if (param_use_simulated_annealing_phase_cost) {
            double dt = neighbor.sum_of_costs - neighbor.old_sum_of_costs;
            bool b =
                simulated_annealing::can_accept(dt, screen, num_of_iteration);
            if (b) return true;
        }
        while (p2 != p) {
            int a = *p2;
            path_table.deletePath(agents[a].id, agents[a].path);
            ++p2;
        }
        if (!neighbor.old_paths.empty()) {
            p2 = neighbor.agents.begin();
            for (int i = 0; i < (int)neighbor.agents.size(); i++) {
                int a = *p2;
                agents[a].path = neighbor.old_paths[i];
                agents[a].state = neighbor.old_state[i];
                path_table.insertPath(agents[a].id, agents[a].path);
                ++p2;
            }
            neighbor.sum_of_costs = neighbor.old_sum_of_costs;
        }
        is_running = 104;
        return false;
    }
}

bool LNS::runlacam() {
    auto shuffled_agents = neighbor.agents;
    std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());

    auto map_name = instance.getMapFile();
    std::vector<Task*> T;
    PIBT_Agents A;

    std::vector<int> start_indexes;
    std::vector<int> goal_indexes;
    for (int i : shuffled_agents) {
        start_indexes.push_back(agents[i].path_planner->start_location);
        goal_indexes.push_back(agents[i].path_planner->goal_location);
    }
    
    const auto ins = lacam::InstanceLa(map_name, start_indexes, goal_indexes);

    auto D = new lacam::DistTable(ins);
    std::vector<std::vector<int>> table(ins.N, std::vector<int>(D->K, D->K)); 
    for (int id=0; id<D->K; id++) {
        auto n = ins.G->V[id];
        auto loc = instance.linearizeCoordinate(n->y, n->x);
        for (int i=0; i<ins.N; i++) {
            auto a = shuffled_agents[i];
            auto h = this->agents[a].path_planner->p_my_heuristic->at(loc);
            D->table[i][id] = h;
        }
    }
    // D->setup(&ins);

    auto func = [&] (const lacam::Solution &solution) {
        callbackLacamResult(solution, shuffled_agents);
    };

    const auto deadline = lacam::Deadline(time_limit * 1000);
    int seed = rand();
    const auto solution = lacam_solve(ins, 3, &deadline, seed, D, func);

    delete D;

    if (solution.empty()) 
        return false;

    updateLacamResult(solution, shuffled_agents);
    return true;
}

void LNS::callbackLacamResult(const lacam::Solution &solution, vector<int>& shuffled_agents) {
    path_table.reset();
    num_of_iteration++;
    updateLacamResult(solution, shuffled_agents);
    solution_shared->do_share_phase_cost(myId, path_table, agents, destroy_weights);
    solution_shared->colliding_pairs_4_print = 0;
}

void LNS::updateLacamResult(const lacam::Solution &solution, vector<int>& shuffled_agents) {
    std::vector<lacam::Path> paths = lacam::translateConfigsToPaths(solution);
    int soc = 0;
    for (int i = 0; i < paths.size(); i++) {
        int a_id = shuffled_agents[i];

        agents[a_id].path.resize(paths[i].size());
        int last_goal_visit = 0;
        // if (screen >= 2) std::cout << A[i]->logStr() << std::endl;
        for (int n_index = 0; n_index < paths[i].size(); n_index++) {
            auto n = paths[i][n_index];
            auto loc = instance.linearizeCoordinate(n->y, n->x);
            agents[a_id].path[n_index] = PathEntry(loc);

            // record the last time agent reach the goal from a non-goal vertex.
            if (agents[a_id].path_planner->goal_location == loc &&
                n_index - 1 >= 0 &&
                agents[a_id].path_planner->goal_location !=
                    agents[a_id].path[n_index - 1].location)
                last_goal_visit = n_index;
        }
        // resize to last goal visit time
        agents[a_id].path.resize(last_goal_visit + 1);
        if (screen >= 2)
            std::cout << " Length: " << agents[a_id].path.size() << std::endl;
        if (screen >= 5) {
            cout << "Agent " << a_id << ":";
            for (auto loc : agents[a_id].path) {
                cout << loc.location << ",";
            }
            cout << endl;
        }
        agents[a_id].state.tid = threadId;
        agents[a_id].state.iter = num_of_iteration;
        path_table.insertPath(agents[a_id].id, agents[a_id].path);
        soc += (int)agents[a_id].path.size() - 1;
    }
    path_table.sum_of_costs = soc;
    neighbor.sum_of_costs = soc;
}

bool LNS::runPPS() {
    auto shuffled_agents = neighbor.agents;
    std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());

    MAPF P = preparePIBTProblem(shuffled_agents);
    P.setTimestepLimit(pipp_option.timestepLimit);

    // seed for solver
    auto* MT_S = new std::mt19937(0);
    PPS solver(&P, MT_S);
    solver.setTimeLimit(time_limit);
    //    solver.WarshallFloyd();
    bool result = solver.solve();
    if (result) updatePIBTResult(P.getA(), shuffled_agents);
    return result;
}
bool LNS::runPIBT() {
    auto shuffled_agents = neighbor.agents;
    std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());

    MAPF P = preparePIBTProblem(shuffled_agents);

    // seed for solver
    auto MT_S = new std::mt19937(0);
    PIBT solver(&P, MT_S);
    solver.setTimeLimit(time_limit);
    bool result = solver.solve();
    if (result) updatePIBTResult(P.getA(), shuffled_agents);
    return result;
}
bool LNS::runWinPIBT() {
    auto shuffled_agents = neighbor.agents;
    std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());

    MAPF P = preparePIBTProblem(shuffled_agents);
    P.setTimestepLimit(pipp_option.timestepLimit);

    // seed for solver
    auto MT_S = new std::mt19937(0);
    winPIBT solver(&P, pipp_option.windowSize, pipp_option.winPIBTSoft, MT_S);
    solver.setTimeLimit(time_limit);
    bool result = solver.solve();
    if (result) updatePIBTResult(P.getA(), shuffled_agents);
    return result;
}

MAPF LNS::preparePIBTProblem(vector<int>& shuffled_agents) {
    // seed for problem and graph
    auto MT_PG = new std::mt19937(0);

    //    Graph* G = new SimpleGrid(instance);
    Graph* G = new SimpleGrid(instance.getMapFile());

    std::vector<Task*> T;
    PIBT_Agents A;

    for (int i : shuffled_agents) {
        assert(G->existNode(agents[i].path_planner->start_location));
        assert(G->existNode(agents[i].path_planner->goal_location));
        auto a =
            new PIBT_Agent(G->getNode(agents[i].path_planner->start_location));

        //        PIBT_Agent* a = new PIBT_Agent(G->getNode(
        //        agents[i].path_planner.start_location));
        A.push_back(a);
        Task* tau = new Task(G->getNode(agents[i].path_planner->goal_location));

        T.push_back(tau);
        if (screen >= 5) {
            cout << "Agent " << i << " start: " << a->getNode()->getPos()
                 << " goal: " << tau->getG().front()->getPos() << endl;
        }
    }

    return MAPF(G, A, T, MT_PG);
}

void LNS::updatePIBTResult(const PIBT_Agents& A, vector<int>& shuffled_agents) {
    int soc = 0;
    for (int i = 0; i < A.size(); i++) {
        int a_id = shuffled_agents[i];

        agents[a_id].path.resize(A[i]->getHist().size());
        int last_goal_visit = 0;
        if (screen >= 2) std::cout << A[i]->logStr() << std::endl;
        for (int n_index = 0; n_index < A[i]->getHist().size(); n_index++) {
            auto n = A[i]->getHist()[n_index];
            agents[a_id].path[n_index] = PathEntry(n->v->getId());

            // record the last time agent reach the goal from a non-goal vertex.
            if (agents[a_id].path_planner->goal_location == n->v->getId() &&
                n_index - 1 >= 0 &&
                agents[a_id].path_planner->goal_location !=
                    agents[a_id].path[n_index - 1].location)
                last_goal_visit = n_index;
        }
        // resize to last goal visit time
        agents[a_id].path.resize(last_goal_visit + 1);
        if (screen >= 2)
            std::cout << " Length: " << agents[a_id].path.size() << std::endl;
        if (screen >= 5) {
            cout << "Agent " << a_id << ":";
            for (auto loc : agents[a_id].path) {
                cout << loc.location << ",";
            }
            cout << endl;
        }
        path_table.insertPath(agents[a_id].id, agents[a_id].path);
        soc += (int)agents[a_id].path.size() - 1;
    }

    neighbor.sum_of_costs = soc;
}

void LNS::chooseDestroyHeuristicbyALNS() {
    rouletteWheel();
    switch (selected_neighbor) {
        case 0:
            destroy_strategy = RANDOMWALK;
            break;
        case 1:
            destroy_strategy = INTERSECTION;
            break;
        case 2:
            destroy_strategy = RANDOMAGENTS;
            break;
        default:
            cerr << "ERROR" << endl;
            exit(-1);
    }
}

bool LNS::generateNeighborByIntersection() {
    if (intersections.empty()) {
        for (int i = 0; i < instance.map_size; i++) {
            if (!instance.isObstacle(i) && instance.getDegree(i) > 2)
                intersections.push_back(i);
        }
    }

    set<int> neighbors_set;
    auto pt = intersections.begin();
    std::advance(pt, rand() % intersections.size());
    int location = *pt;
    path_table.get_agents(neighbors_set, neighbor_size, location);
    if (neighbors_set.size() < neighbor_size) {
        set<int> closed;
        closed.insert(location);
        std::queue<int> open;
        open.push(location);
        while (!open.empty() && (int)neighbors_set.size() < neighbor_size) {
            int curr = open.front();
            open.pop();
            for (auto next : instance.getNeighbors(curr)) {
                if (closed.count(next) > 0) continue;
                open.push(next);
                closed.insert(next);
                if (instance.getDegree(next) >= 3) {
                    path_table.get_agents(neighbors_set, neighbor_size, next);
                    if ((int)neighbors_set.size() == neighbor_size) break;
                }
            }
        }
    }
    neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
    if (neighbor.agents.size() > neighbor_size) {
        std::random_shuffle(neighbor.agents.begin(), neighbor.agents.end());
        neighbor.agents.resize(neighbor_size);
    }
    if (screen >= 2)
        cout << "Generate " << neighbor.agents.size()
             << " neighbors by intersection " << location << endl;
    return true;
}
bool LNS::generateNeighborByRandomWalk() {
    if (neighbor_size >= (int)agents.size()) {
        neighbor.agents.resize(agents.size());
        for (int i = 0; i < (int)agents.size(); i++) neighbor.agents[i] = i;
        return true;
    }

    int a = findMostDelayedAgent();
    if (a < 0) return false;

    set<int> neighbors_set;
    neighbors_set.insert(a);
    randomWalk(a, agents[a].path[0].location, 0, neighbors_set, neighbor_size,
               (int)agents[a].path.size() - 1);
    int count = 0;
    while (neighbors_set.size() < neighbor_size && count < 10) {
        if (agents[a].path.empty())  // path will empty because interupt
            return false;
        int t = rand() % agents[a].path.size();
        randomWalk(a, agents[a].path[t].location, t, neighbors_set,
                   neighbor_size, (int)agents[a].path.size() - 1);
        count++;
        // select the next agent randomly
        int idx = rand() % neighbors_set.size();
        int i = 0;
        for (auto n : neighbors_set) {
            if (i == idx) {
                a = i;
                break;
            }
            i++;
        }
    }
    if (neighbors_set.size() < 2) return false;
    neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
    if (screen >= 2)
        cout << "Generate " << neighbor.agents.size()
             << " neighbors by random walks of agent " << a << "("
             << agents[a]
                    .path_planner
                    ->p_my_heuristic->at(agents[a].path_planner->start_location)
             << "->" << agents[a].path.size() - 1 << ")" << endl;

    return true;
}

int LNS::findMostDelayedAgent() {
    int a = -1;
    int max_delays = -1;
    for (int i = 0; i < agents.size(); i++) {
        if (tabu_list.find(i) != tabu_list.end()) continue;
        int delays = agents[i].getNumOfDelays();
        if (max_delays < delays) {
            a = i;
            max_delays = delays;
        }
    }
    if (max_delays == 0) {
        tabu_list.clear();
        return -1;
    }
    tabu_list.insert(a);
    if (tabu_list.size() == agents.size()) tabu_list.clear();
    return a;
}

int LNS::findRandomAgent() const {
    int a = 0;
    int pt = rand() % (sum_of_costs - sum_of_distances) + 1;
    int sum = 0;
    for (; a < (int)agents.size(); a++) {
        sum += agents[a].getNumOfDelays();
        if (sum >= pt) break;
    }
    assert(sum >= pt);
    return a;
}

// a random walk with path that is shorter than upperbound and has conflicting
// with neighbor_size agents
void LNS::randomWalk(int agent_id, int start_location, int start_timestep,
                     set<int>& conflicting_agents, int neighbor_size,
                     int upperbound) {
    int loc = start_location;
    for (int t = start_timestep; t < upperbound; t++) {
        auto next_locs = instance.getNeighbors(loc);
        next_locs.push_back(loc);
        while (!next_locs.empty()) {
            int step = rand() % next_locs.size();
            auto it = next_locs.begin();
            advance(it, step);
            int next_h_val = agents[agent_id].path_planner->p_my_heuristic->at(*it);
            if (t + 1 + next_h_val < upperbound)  // move to this location
            {
                path_table.getConflictingAgents(agent_id, conflicting_agents,
                                                loc, *it, t + 1);
                loc = *it;
                break;
            }
            next_locs.erase(it);
        }
        if (next_locs.empty() || conflicting_agents.size() >= neighbor_size)
            break;
    }
}

void LNS::validateSolution() const {
    int sum = 0;
    stringstream sout;
    bool succ = true;
    for (const auto& a1_ : agents) {
        if (a1_.path.empty()) {
            sout << "No solution for agent " << a1_.id << endl;
            succ = false;
            goto error_flag;
        } else if (a1_.path_planner->start_location !=
                   a1_.path.front().location) {
            sout << "The path of agent " << a1_.id << " starts from location "
                 << a1_.path.front().location
                 << ", which is different from its start location "
                 << a1_.path_planner->start_location << endl;
            succ = false;
            goto error_flag;
        } else if (a1_.path_planner->goal_location !=
                   a1_.path.back().location) {
            sout << "The path of agent " << a1_.id << " ends at location "
                 << a1_.path.back().location
                 << ", which is different from its goal location "
                 << a1_.path_planner->goal_location << endl;
            succ = false;
            goto error_flag;
        }
        for (int t = 1; t < (int)a1_.path.size(); t++) {
            if (!instance.validMove(a1_.path[t - 1].location,
                                    a1_.path[t].location)) {
                sout << "The path of agent " << a1_.id << " jump from "
                     << a1_.path[t - 1].location << " to "
                     << a1_.path[t].location << " between timesteps " << t - 1
                     << " and " << t << endl;
                succ = false;
                goto error_flag;
            }
        }
        sum += (int)a1_.path.size() - 1;
        for (const auto& a2_ : agents) {
            if (a1_.id >= a2_.id || a2_.path.empty()) continue;
            const auto& a1 = a1_.path.size() <= a2_.path.size() ? a1_ : a2_;
            const auto& a2 = a1_.path.size() <= a2_.path.size() ? a2_ : a1_;
            int t = 1;
            for (; t < (int)a1.path.size(); t++) {
                if (a1.path[t].location ==
                    a2.path[t].location)  // vertex conflict
                {
                    sout << "Find a vertex conflict between agents " << a1.id
                         << " and " << a2.id << " at location "
                         << a1.path[t].location << " at timestep " << t << endl;
                    succ = false;
                    goto error_flag;
                } else if (a1.path[t].location == a2.path[t - 1].location &&
                           a1.path[t - 1].location ==
                               a2.path[t].location)  // edge conflict
                {
                    sout << "Find an edge conflict between agents " << a1.id
                         << " and " << a2.id << " at edge ("
                         << a1.path[t - 1].location << ","
                         << a1.path[t].location << ") at timestep " << t
                         << endl;
                    succ = false;
                    goto error_flag;
                }
            }
            int target = a1.path.back().location;
            for (; t < (int)a2.path.size(); t++) {
                if (a2.path[t].location == target)  // target conflict
                {
                    sout << "pe " << myId
                         << " Find a target conflict where agent " << a2.id
                         << " (of length " << a2.path.size() - 1
                         << ") traverses agent " << a1.id << " (of length "
                         << a1.path.size() - 1 << ")'s target location "
                         << target << " at timestep " << t << endl;
                    succ = false;
                    goto error_flag;
                }
            }
        }
    }
    if (sum_of_costs != sum) {
        sout << "pe " << myId << " The computed sum of costs " << sum_of_costs
             << " is different from the sum of the paths in the solution "
             << sum << endl;
        succ = false;
        goto error_flag;
    }
error_flag:
    if (!succ) {
        printf("%s", sout.str().c_str());
        exit(-1);
    }
}

void LNS::writeIterStatsToFile(const string& file_name) const {
    if (init_lns != nullptr) {
        init_lns->writeIterStatsToFile(file_name + "-initLNS.csv");
    }
    if (num_of_iteration <= 1) return;
    string name = file_name;
    if (use_init_lns or max_num_of_iterations > 0)
        name += "-LNS.csv";
    else
        name += "-" + init_algo_name + ".csv";
    std::ofstream output;
    output.open(name);
    // header
    output << "num of agents,"
           << "sum of costs,"
           << "runtime,"
           << "cost lowerbound,"
           << "sum of distances,"
           << "MAPF algorithm" << endl;

    for (const auto& data : iteration_stats) {
        output << data.num_of_agents << "," << data.sum_of_costs << ","
               << data.runtime << ","
               << max(sum_of_costs_lowerbound, sum_of_distances) << ","
               << sum_of_distances << "," << data.algorithm << endl;
    }
    output.close();
}

void LNS::writeResultToFile(const string& file_name) const {
    if (init_lns != nullptr) {
        init_lns->writeResultToFile(file_name + "-initLNS.csv",
                                    sum_of_distances, preprocessing_time);
    }
    string name = file_name;
    if (use_init_lns or max_num_of_iterations > 0)
        name += "-LNS.csv";
    else
        name += "-" + init_algo_name + ".csv";
    std::ifstream infile(name);
    bool exist = infile.good();
    infile.close();
    if (!exist) {
        ofstream addHeads(name);
        addHeads
            << "runtime,solution cost,initial solution cost,lower bound,sum of "
               "distance,"
            << "iterations,"
            << "group size,"
            << "runtime of initial solution,restart times,area under curve,"
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
            auc += (prev->sum_of_costs - sum_of_distances) *
                   (curr->runtime - prev->runtime);
            prev = curr;
            ++curr;
        }
        auc += (prev->sum_of_costs - sum_of_distances) *
               (time_limit - prev->runtime);
    }
    ofstream stats(name, std::ios::app);
    stats << runtime << "," << sum_of_costs << "," << initial_sum_of_costs
          << "," << max(sum_of_distances, sum_of_costs_lowerbound) << ","
          << sum_of_distances << "," << num_of_iteration << ","
          << average_group_size << "," << initial_solution_runtime << ","
          << restart_times << "," << auc << "," << num_LL_expanded << ","
          << num_LL_generated << "," << num_LL_reopened << "," << num_LL_runs
          << "," << preprocessing_time << "," << getSolverName() << ","
          << instance.getInstanceName() << endl;
    stats.close();
}

void LNS::writePathsToFile(const string& file_name) const {
    std::ofstream output;
    output.open(file_name);
    // header
    // output << agents.size() << endl;

    for (const auto& agent : agents) {
        output << "Agent " << agent.id << ":";
        for (const auto& state : agent.path)
            output << "(" << instance.getRowCoordinate(state.location) << ","
                   << instance.getColCoordinate(state.location) << ")->";
        output << endl;
    }
    output.close();
}

// Interrupt the solving, so it can be started again with new assumptions
void LNS::setSolverInterrupt() {
    // if (screen > 0)
        printf("\t--> g %d pe %d lns:1370 set collision_interrupt 1\n", 
               myGroup, myId);
    solution_shared->phase_collision_asynch_interrupt = true;
    solution_shared->phase_cost_asynch_interrupt = true;
    stop_costlns_only_solver = true;
}

void LNS::unsetSolverInterrupt() {
    // if (screen > 0)
        printf("\t--> g %d pe %d lns:1376 set collision_interrupt 0\n", 
               myGroup, myId);
    solution_shared->phase_collision_asynch_interrupt = false;
    solution_shared->phase_cost_asynch_interrupt = false;
    stop_costlns_only_solver = false;
}