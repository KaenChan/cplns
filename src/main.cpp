#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>

#include "AnytimeBCBS.h"
#include "AnytimeEECBS.h"
#include "CBS.h"
#include "LNS.h"
#include "Logger.h"
#include "PIBT/pibt.h"
#include "ShareDataManager.h"
#include "Threading.h"
#include "defines.h"
#include "ThreadPool.h"
#include "SysInfo.h"

// #define PROFILE      //需要cmake里有profiler
// #define HEAPPROFILE  //需要cmake里有tcmalloc

#ifdef PROFILE
#include <gperftools/profiler.h>
#endif
#ifdef HEAPPROFILE
#include <gperftools/heap-profiler.h>
#endif

vector<LNS *> solvers;
bool solvingDoneLocal = false;
Mutex interruptLock;
string group_mode_phase_cost = "one";

string group_mode_phase_collision;

LNS *lns_1st = nullptr;

//当使用eecbs时，在r64机器上不能用jemalloc，在r8上面可以
bool portfolio_pp_and_eecbs_init = false;

void stopAllSolvers() {
    interruptLock.lock();
    solvingDoneLocal = true;
    for (int i = 0; i < param_num_solver; i++) {
        solvers[i]->setSolverInterrupt();
    }
    interruptLock.unlock();
    // 等全部结束
    while (true) {
        usleep(10);
        // for (int i = 0; i < numSolver; i++) {
        // 	cout << solvers[i]->is_running << " ";
        // }
        // cout << endl;
        bool b = true;
        for (int i = 0; i < param_num_solver; i++) {
            if (solvers[i]->is_running != -1) b = false;
        }
        if (b) break;
    }
}

// Return true if the solver should stop.
bool getGlobalEnding(int mpi_size, int mpi_rank) {
    int sendMsg = 0;
    if (solvingDoneLocal) {
        stopAllSolvers();
        return true;
    }
    return false;
}

void *solverRunningThread(void *arg) {
    LNS *solver = (LNS *)arg;
    tl_use_simulated_annealing = param_use_simulated_annealing;
    if (param_single_agent_solver_f_w < -1 && param_num_solver > 1) {
        float wh = -param_single_agent_solver_f_w - 1;
        float p = 3.;
        p = param_single_agent_solver_f_w_p;
        // printf("g_single_agent_solver_f_w %f wh %f\n",
        // g_single_agent_solver_f_w, wh);
        if (group_mode_phase_collision == "central") {
            wh = std::pow(wh, 1 / p);
            tl_single_agent_solver_f_w =
                1 + std::pow(solver->myId * wh /
                                 std::max(1, solver->numLocalSolver - 1),
                             p);
            if (solver->myGroup > 0)
                tl_single_agent_solver_f_w += std::pow(
                    (solver->myGroup) * wh / (solver->numGroup - 1), p);
        } else {
            wh = std::pow(wh, 1 / p);
            tl_single_agent_solver_f_w =
                1 + std::pow((solver->threadId) * wh /
                                 std::max(1, param_num_solver - 1),
                             p);
            if (solver->myGroup == 0 && solver->numGroup > 1)
                tl_single_agent_solver_f_w = 1.0;
        }
        // if(solver->myId == 11)
        // 	tl_single_agent_solver_f_w = 3.0;
        // if(solver->myGroup == 9)
        // 	tl_single_agent_solver_f_w = 2.0;
        // if(solver->myGroup == 10)
        // 	tl_single_agent_solver_f_w = 3.0;
        // if(tl_single_agent_solver_f_w >= 2)
        // 	tl_use_simulated_annealing = false;
    } else {
        tl_single_agent_solver_f_w = std::abs(param_single_agent_solver_f_w);
        if (tl_single_agent_solver_f_w < 1) tl_single_agent_solver_f_w = 1;
    }
    if(portfolio_pp_and_eecbs_init && solver->init_algo_name == "EECBS")
        tl_single_agent_solver_f_w = 1.0;
    printf("g %d pe %d fw %.2f sa %d %s %s %s\n", solver->myGroup, solver->myId,
           tl_single_agent_solver_f_w, tl_use_simulated_annealing, 
           solver->init_algo_name.c_str(), solver->init_replan_algo_name.c_str(), solver->replan_algo_name.c_str());
    // while (true)
    {
        // interruptLock.lock();
        // if (solvingDoneLocal) {
        // 	interruptLock.unlock();
        // 	break;
        // } else {
        // 	solver->unsetSolverInterrupt();
        // }
        // interruptLock.unlock();
        // bool succ = solver->run();
        bool succ = solver->run_with_restart();
        solver->is_running = -1;
        if (succ == true) {
            solvingDoneLocal = true;
        } else {
            solvingDoneLocal = true;
        }
        // if(solvingDoneLocal)
        // 	cout << "solverRunningThread " << "g " << solver->myGroup << "
        // pe " << solver->myId << " solvingDoneLocal " << solvingDoneLocal <<
        // endl;
    }
    return NULL;
}

vector<int> generate_groups(int groupsCount, int solversCount) {
    vector<int> group_list;
    group_list.resize(groupsCount, 0);
    if (group_mode_phase_collision == "central")
        for (int i = 0; i < groupsCount; i++) {
            if (i == 0)
                group_list[i] = solversCount - groupsCount + 1;
            else
                group_list[i] = 1;
        }
    else
        for (int i = 0; i < solversCount; i++) {
            int gi = i % groupsCount;
            group_list[gi]++;
        }
    if(portfolio_pp_and_eecbs_init && group_list[groupsCount-1] > 1) {
        group_list[groupsCount-1]--;
        group_list.push_back(1);
    }

    return group_list;
}

int runThreadTestSpeed(void *arg) {
    LNS *solver = (LNS *)arg;
    tl_use_simulated_annealing = param_use_simulated_annealing;
    tl_single_agent_solver_f_w = 1;
    bool succ = solver->run();
    return solver->init_lns->num_of_ll_search;
}

int get_np_by_test_speed(Instance &instance, PIBTPPS_option &pipp_option,
                         vector<Agent*> &agents) {
    int num_of_thread = 0;
    int max_num_thread = 32;
    cout << "get_np_by_test_speed" << endl;

    ThreadPool thread_pool(max_num_thread);

    SolutionShared *solution_shared = new SolutionShared(0, false, false);
    vector<LNS *> lns_list;
    bool preprocess_LL_engines;
    for (int i = 0; i < max_num_thread; i++) {
        if (lns_1st == nullptr)
            preprocess_LL_engines = true;
        else
            preprocess_LL_engines = false;
        LNS *lns =
            new LNS(instance, 1000, "PP", "PP", "PP", "Adaptive", 8, 1, true,
                    "Adaptive", true, 0, pipp_option, solution_shared, preprocess_LL_engines);
        lns->myId = i;
        lns->threadId = i;
        lns->test_speed_mode = true;
        // for (int i = 0; i < agents.size(); i++)
        //     lns->agents[i].path_planner->my_heuristic =
        //         agents[i]->path_planner->my_heuristic;
        if (lns_1st == nullptr) {
            lns_1st = lns;
        } else {
            for (int i = 0; i < lns_1st->agents.size(); i++)
                lns->agents[i].path_planner->p_my_heuristic =
                    lns_1st->agents[i].path_planner->p_my_heuristic;
        }
        lns_list.push_back(lns);
    }

    cout << "get_np_by_test_speed start..." << endl;

#ifdef PROFILE
    ProfilerStart("cpu_profile.log");
#endif
#ifdef HEAPPROFILE
    HeapProfilerStart("heap_profile.log");
#endif

    for (int np = 4; np <= max_num_thread; np+=4) {
        int npi = np;
        double t1 = getTime();
        solution_shared->phase_collision_asynch_interrupt = false;
        // vector<std::thread *> solverThreads;
        std::vector< std::future<int> > results;
        for (int i = 0; i < npi; i++) {
            // std::thread *work_thread =
            //     new std::thread(runThreadTestSpeed, lns_list[i]);
            // solverThreads.push_back(work_thread);
            results.emplace_back(
                thread_pool.enqueue(runThreadTestSpeed, lns_list[i]));
        }
        // for (int i = 0; i < np; i++) {
        //     solverThreads[i]->join();
        // }
        int sum = 0;
        for (int i = 0; i < results.size(); i++) {
            int r2 = results[i].get();
            sum += r2;
        }
        // solverThreads.clear();
        double runtime = getTime() - t1;
        printf("np %02d runtime %.3f sipps %d aps %.1f\n", np, runtime, sum,
               sum / runtime / npi);
    }

#ifdef PROFILE
    ProfilerStop();
#endif
#ifdef HEAPPROFILE
    HeapProfilerStop();
#endif

    for (int i = 0; i < max_num_thread; i++) {
        if (lns_1st != lns_list[i])
            delete lns_list[i];
    }
    exit(1);
    return 8;
}

/* Main function */
int main(int argc, char **argv) {
    for (int i = 0; i < argc; i++) std::cout << argv[i] << " ";
    std::cout << std::endl;
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()("help", "produce help message")

        // params for the input instance and experiment settings
        ("map,m", po::value<string>()->required(), "input file for map")
        ("agents,a", po::value<string>()->required(), "input file for agents")
        ("agentNum,k", po::value<int>()->default_value(0), "number of agents")
        ("output,o", po::value<string>(), "output file name (no extension)")
        ("cutoffTime,t", po::value<double>()->default_value(7200), "cutoff time (seconds)")
        ("screen,s", po::value<int>()->default_value(0), 
            "screen option (0: none; 1: LNS results; 2:LNS detailed results; "
            "3: MAPF detailed results)")
        ("stats", po::value<string>(), "output stats file")

        // solver
        ("solver", po::value<string>()->default_value("LNS"), "solver (LNS, computeInitLB)")
        ("sipp", po::value<int>()->default_value(1), "Use SIPP as the single-agent solver")
        ("seed", po::value<int>()->default_value(0), "Random seed")

        // lacam
        ("lacam_star", po::value<bool>()->default_value(false), "lacam_star")
        ("lacam2_only", po::value<bool>()->default_value(false), "lacam2_only")
        ("lacam_refiner_num", po::value<int>()->default_value(4), "lacam_refiner_num")
        ("lacam_multi_thread", po::value<bool>()->default_value(false), "lacam_multi_thread")
        ("lacam_pibt_num", po::value<int>()->default_value(1), "lacam_multi_thread")
        ("lacam_scatter", po::value<bool>()->default_value(true), "lacam_scatter")

        // params for LNS
        ("initLNS", po::value<bool>()->default_value(true), 
            "use LNS to find initial solutions if the initial sovler fails")
        ("neighborSize", po::value<int>()->default_value(8),
            "Size of the neighborhood")
        ("maxIterations", po::value<int>()->default_value(1000),
            "maximum number of iterations")
        ("initAlgo", po::value<string>()->default_value("PP"),
            "MAPF algorithm for finding the initial solution (EECBS, PP, PPS, "
            "CBS, PIBT, winPIBT)")
        ("initReplanAlgo", po::value<string>()->default_value("PP"),
            "MAPF algorithm for replanning (GCBS, PBS, PP)")
        ("replanAlgo", po::value<string>()->default_value("PP"),
            "MAPF algorithm for replanning (EECBS, CBS, PP)")
        ("destoryStrategy", po::value<string>()->default_value("Adaptive"),
            "Heuristics for finding subgroups (Random, RandomWalk, "
            "Intersection, Adaptive)")
        ("pibtWindow", po::value<int>()->default_value(5),
            "window size for winPIBT")
        ("winPibtSoftmode", po::value<bool>()->default_value(true),
            "winPIBT soft mode")

        // ll a*
        ("astar_wh", po::value<float>()->default_value(1.),
            "astar_wh, wh>=1, fixed; wh<=-1, vary")
        ("astar_wh_p", po::value<float>()->default_value(3.), "astar_wh exp")
        ("use_fixed_wh1_phase2", po::value<int>()->default_value(1), "")

        // params for initLNS
        ("initDestoryStrategy", po::value<string>()->default_value("Adaptive"),
            "Heuristics for finding subgroups (Target, Collision, Random, "
            "Adaptive)")

        // sa phase_collision
        ("sa", po::value<bool>()->default_value(true), "simulated annealing")
        ("sa_T", po::value<double>()->default_value(1000.0),
            "simulated annealing parameter: T")
        ("sa_restart_resume", po::value<bool>()->default_value(false),
            "sa_restart_resume")
        ("sa_restart_coolfactor", po::value<double>()->default_value(1),
            "simulated annealing parameter: at")
        ("sa_iteration_coolfactor", po::value<double>()->default_value(0.99),
            "simulated annealing parameter: at")
        ("sa_max_con_fail", po::value<double>()->default_value(0.4),
            "sa_max_con_fail")
        ("costlns_max_con_fail", po::value<double>()->default_value(0.8),
            "costlns_max_con_fail")
        // sa phase_cost
        ("sa_phase2", po::value<bool>()->default_value(false),
         "simulated annealing in phase_cost for debug")
        ("sa_iteration_coolfactor_phase2",
            po::value<double>()->default_value(0.99),
            "simulated annealing parameter in phase_cost: at")

        // params for parallel
        ("syncMode", po::value<bool>()->default_value(false),
            "use sync mode for each iteration")
        ("doSharePhase1", po::value<bool>()->default_value(true),
            "use solution sharing in phase 1")
        ("doSharePhase2", po::value<bool>()->default_value(true),
            "use solution sharing in phase 2")
        ("numSolver", po::value<int>()->default_value(4),
            "num of solvers for parallel running")
        ("numSolverInit", po::value<int>()->default_value(-1),
            "num of solvers for init lns, -1 is the same as numSolver")
        ("numGroup", po::value<int>()->default_value(1),
            "num of groups for parallel running")
        ("shareStepPhase1", po::value<int>()->default_value(100), "")
        ("shareStepPhase2", po::value<int>()->default_value(1), "")
        ("groupModePhase1", po::value<string>()->default_value("evenly"),
            "evenly, central")
        ("groupModePhase2", po::value<string>()->default_value("one"),
            "same: same as phase_collision, one: one group")
        ("allRunNextIfOneSolutionFound", po::value<bool>()->default_value(false),
            "allRunNextIfOneSolutionFound")
        // param for parallel lns sa
        ("num_of_run_after_sa_accept", po::value<int>()->default_value(0), "")
        // param for initialLNS
        ("portfolio_pp_and_eecbs_init", po::value<bool>()->default_value(false), "")
        ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        cout << desc << endl;
        return 1;
    }

    PIBTPPS_option pipp_option;
    pipp_option.windowSize = vm["pibtWindow"].as<int>();
    pipp_option.winPIBTSoft = vm["winPibtSoftmode"].as<bool>();

    param_use_simulated_annealing = vm["sa"].as<bool>();
    param_use_simulated_annealing_phase_cost = vm["sa_phase2"].as<bool>();
    param_sa_iteration_coolfactor_phase_cost =
        vm["sa_iteration_coolfactor_phase2"].as<double>();

    param_use_fixed_wh1_phase_cost = vm["use_fixed_wh1_phase2"].as<int>();

    param_simulated_annealing_T = vm["sa_T"].as<double>();
    param_sa_restart_coolfactor = vm["sa_restart_coolfactor"].as<double>();
    param_sa_iteration_coolfactor = vm["sa_iteration_coolfactor"].as<double>();
    param_sa_max_con_fail = vm["sa_max_con_fail"].as<double>();
    param_sa_restart_resume = vm["sa_restart_resume"].as<bool>();
    param_share_step_phase_collision = vm["shareStepPhase1"].as<int>();
    param_share_step_phase_cost = vm["shareStepPhase2"].as<int>();
    param_num_of_run_after_sa_accept =
        vm["num_of_run_after_sa_accept"].as<int>();

    param_costlns_max_con_fail = vm["costlns_max_con_fail"].as<double>();
    portfolio_pp_and_eecbs_init =
        vm["portfolio_pp_and_eecbs_init"].as<bool>();

    lacam::Planner::FLG_STAR = vm["lacam_star"].as<bool>();

    auto lacam2_only = vm["lacam2_only"].as<bool>();
    if(lacam2_only) {
        lacam::Planner::FLG_SWAP = false;
        lacam::Planner::FLG_MULTI_THREAD = false;
        lacam::Planner::PIBT_NUM = 1;
        lacam::Planner::FLG_REFINER = false;
        lacam::Planner::FLG_SCATTER = false;
        lacam::Planner::RANDOM_INSERT_PROB1 = 0;
        lacam::Planner::RANDOM_INSERT_PROB2 = 0;
        lacam::Planner::FLG_RANDOM_INSERT_INIT_NODE = false;
        lacam::Planner::RECURSIVE_RATE = 0;
        lacam::Planner::RECURSIVE_TIME_LIMIT = 0;
    }
    else {
        lacam::Planner::REFINER_NUM = vm["lacam_refiner_num"].as<int>();
        lacam::Planner::FLG_MULTI_THREAD = vm["lacam_multi_thread"].as<bool>();
        lacam::Planner::PIBT_NUM = vm["lacam_pibt_num"].as<int>();
        lacam::Planner::FLG_SCATTER = vm["lacam_scatter"].as<bool>();
    }

    cout << "dd " << param_num_of_run_after_sa_accept 
		 << " step1 " << param_share_step_phase_collision 
		 << " step2 " << param_share_step_phase_cost
		 << " pp+eecbs " << portfolio_pp_and_eecbs_init
         << endl;

    param_single_agent_solver_f_w = vm["astar_wh"].as<float>();
    param_single_agent_solver_f_w_p = vm["astar_wh_p"].as<float>();

    group_mode_phase_collision = vm["groupModePhase1"].as<string>();
    group_mode_phase_cost = vm["groupModePhase2"].as<string>();

    po::notify(vm);

    int seed = vm["seed"].as<int>();
    if (seed < 0)
        srand((int)time(0));
    else
        srand(seed);
    log(0, "Node 0 start.. %.2f seconds.\n", getTime());

    Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(),
                      vm["agentNum"].as<int>());

    log(0, "Node 0 build map (%dx%d) instance %.2f seconds.\n",
        instance.num_of_cols, instance.num_of_rows, getTime());


    double time_limit = vm["cutoffTime"].as<double>();
    int screen = vm["screen"].as<int>();

    param_num_solver = vm["numSolver"].as<int>();
    param_num_solver_init = vm["numSolverInit"].as<int>();
    if(param_num_solver_init == -1)
        param_num_solver_init = param_num_solver;
    param_num_group = vm["numGroup"].as<int>();
    if (param_num_solver_init > param_num_solver)
        param_num_solver_init = param_num_solver;
    if(!vm["doSharePhase2"].as<bool>() && param_num_solver_init < param_num_solver) {
        cout << "if not use share in cost phase, np should = np_init" << endl;
        exit(0);
    }
    if (param_num_group > param_num_solver_init) param_num_group = param_num_solver_init;

    if (vm["solver"].as<string>() == "computeInitLB") {
        CBS cbs(instance, 1000, screen);
        double start_time = getTime();
        int lb = cbs.getRootLB();
        double runtime = getTime() - start_time;
        cout << "final"
             << " init_lb " << lb << " runtime " << runtime << endl;
        exit(0);
    }

    vector<Agent*> agents;
    agents.reserve(param_num_solver);
    for (int i = 0; i < param_num_solver; i++) {
        agents.emplace_back(new Agent(instance, i, true, true));
    }

    // get_np_by_test_speed(instance, pipp_option, agents);

    vector<int> group_list =
        generate_groups(param_num_group, param_num_solver_init);
    param_num_group = group_list.size();
    ShareDataManager share_manager(
        screen + 0, vm["doSharePhase2"].as<bool>(),
        vm["doSharePhase1"].as<bool>(), group_list, param_num_solver,
        param_num_solver_init);
    vector<SolutionShared *> &solutions =
        share_manager.solutions;

    log(0, "Node 0 SolutionShared %.2f seconds.\n", getTime());

    double max_mem = 0;
    int threadId = 0;
    for (int groupId = 0; groupId < param_num_group+1; groupId++) {
        bool run_costlns_only = false;
        int numLocalSolver = 0;
        if(groupId == param_num_group) {
            run_costlns_only = true;
            numLocalSolver = param_num_solver - param_num_solver_init;
        }
        else 
            numLocalSolver = group_list[groupId];
        log(0, "create group %d with %d solvers.\n", groupId, numLocalSolver);
        for (int i = 0; i < numLocalSolver; i++) {
            bool preprocess_LL_engines;
            if (lns_1st == nullptr)
                preprocess_LL_engines = true;
            else
                preprocess_LL_engines = false;
            // int N = 8;
            // string destory_name = "Adaptive";
            string initAlgo = vm["initAlgo"].as<string>();
            // if(portfolio_pp_and_eecbs_init && groupId == param_num_group -1) {
            //     initAlgo = "EECBS";
            //     solutions[groupId]->has_eecbs_solver = true;
            // }
            if(portfolio_pp_and_eecbs_init && groupId > param_num_group/2) {   // half. TODO: 优化配置
            // if(portfolio_pp_and_eecbs_init && groupId == param_num_group-1) {   // one: 优化配置
                initAlgo = "lacam";
                solutions[groupId]->has_eecbs_solver = true;
            }
            // string initReplanAlgo = "PP";
            // string initDestoryStrategy = "Adaptive";
            SolutionShared *ss = nullptr;
            if(!run_costlns_only)
                ss = solutions[groupId];
            LNS *lns =
                new LNS(instance, time_limit, initAlgo,
                        vm["initReplanAlgo"].as<string>(),
                        vm["replanAlgo"].as<string>(),
                        vm["destoryStrategy"].as<string>(),
                        vm["neighborSize"].as<int>(),
                        vm["maxIterations"].as<int>(), vm["initLNS"].as<bool>(),
                        vm["initDestoryStrategy"].as<string>(),
                        vm["sipp"].as<int>(), screen - 1, pipp_option,
                        ss, preprocess_LL_engines);
            lns->myId = i;
            lns->threadId = threadId++;
            lns->run_costlns_only = run_costlns_only;
            lns->myGroup = groupId;
            if(run_costlns_only)
                lns->myGroup = -1;
            lns->numLocalSolver = numLocalSolver;
            lns->numGroup = param_num_group;
            lns->use_sync_mode_in_iteration = vm["syncMode"].as<bool>();
            lns->all_run_next_if_one_solution_found =
                vm["allRunNextIfOneSolutionFound"].as<bool>();
            lns->share_manager = &share_manager;
            lns->group_mode_phase_collision = group_mode_phase_collision;
            lns->group_mode_phase_cost = group_mode_phase_cost;
            if (lns_1st == nullptr) {
                lns_1st = lns;
            } else {
                for (int i = 0; i < lns_1st->agents.size(); i++)
                    lns->agents[i].path_planner->p_my_heuristic =
                        lns_1st->agents[i].path_planner->p_my_heuristic;
            }
            solvers.push_back(lns);
        }
    }
    log(0, "Node %d started its solvers, initialization1 took %.2f seconds.\n",
        0, getTime());

#ifdef PROFILE
    ProfilerStart("cpu_profile.log");
#endif
    double start_time = getTime();
    for (int groupId = 0; groupId < param_num_group; groupId++)
        solutions[groupId]->time_phase_collision_start = start_time;
    share_manager.time_phase_collision_start = start_time;

    // Thread** solverThreads = (Thread**) malloc (numSolver*sizeof(Thread*));
    // for (int i = 0; i < numSolver; i++) {
    // 	solverThreads[i] = new Thread(solverRunningThread, solvers[i]);
    // 	// pin_thread(i*2, *solverThreads[i]);
    // }

    vector<std::thread *> solverThreads;
    for (int i = 0; i < param_num_solver; i++) {
        std::thread *work_thread =
            new std::thread(solverRunningThread, solvers[i]);
        solverThreads.push_back(work_thread);
        // if(i < 16)
        // 	pin_thread(i*2, *solverThreads.back());
        // else
        // 	pin_thread(64+i*2-32, *solverThreads.back());
    }

    int mpi_size = 1, mpi_rank = 0;
    double startSolving = getTime();
    log(0, "Node %d started its solvers, initialization2 took %.2f seconds.\n",
        mpi_rank, startSolving);

    int maxSeconds = (int)time_limit;  // unlimited
    int maxRounds = -1;
    size_t sleepInt = 1 * 1000 * 1000;
    int round = 1;

    // while (!getGlobalEnding(mpi_size, mpi_rank))
    while (true) {
        if (getGlobalEnding(mpi_size, mpi_rank)) {
            // cout << "getGlobalEnding--------------" << endl;
            break;
        }
        usleep(sleepInt);
        double timeNow = getTime() - start_time;
        // log(0, "%.2f seconds\n", timeNow - startSolving);
        if (round == maxRounds || (maxSeconds != 0 && timeNow > maxSeconds)) {
            solvingDoneLocal = true;
        }
        if (round % 1 == 0) {
            double mem = SysInfo::getMemUsageRate();
            if(max_mem < mem) max_mem = mem;
            if(mem > 0.75) { //停掉eecbs线程，直接停掉eecbs的运行还有bug，当运行largek时，会卡住
                for (int i = 0; i < param_num_solver; i++) {
                    auto s = solvers[i];
                    if(s->init_algo_name == "EECBS" && s->init_lns != nullptr && s->init_lns->num_of_colliding_pairs > 0)
                        s->init_lns->setSolverInterrupt();
                }
            }
            if(mem > 0.80) {
                // stopAllSolvers();
                for (int i = 0; i < param_num_solver; i++) {
                    solvers[i]->setSolverInterrupt();
                }
                share_manager.interrupte_mem_out = true;
                // exit(0);
            }
            share_manager.print(mem);
            if (screen >= 0) {
                cout << "\t";
                for (int i = 0; i < param_num_solver; i++) {
                    cout << solvers[i]->is_running << " ";
                }
                cout << endl;
            }
            fflush(stdout);
        }
        round++;
    }
    double searchTime = getTime() - startSolving;
    for (int groupId = 0; groupId < param_num_group; groupId++)
        solutions[groupId]->runtime_phase_cost =
            searchTime - solutions[groupId]->runtime_phase_collision;
    log(0, "node %d finished, joining solver threads\n", mpi_rank);
    for (int i = 0; i < param_num_solver; i++) {
        solverThreads[i]->join();
    }

#ifdef PROFILE
    ProfilerStop();
#endif

    int num_of_colliding_pairs = MAX_COST;
    int sum_of_costs = MAX_COST;
    int sum_of_costs_wc = MAX_COST;
    double runtime_phase_collision = 0;
    double runtime_phase_cost = 0;
    int num_of_restart = 0;
    int best_gid =
        share_manager.getCurrentBestGroupId(solutions);
    num_of_colliding_pairs = share_manager.g_num_of_colliding_pairs;
    sum_of_costs = share_manager.g_sum_of_costs;
    sum_of_costs_wc = solutions[best_gid]->sum_of_costs_wc;
    runtime_phase_collision = solutions[best_gid]->runtime_phase_collision;
    runtime_phase_cost = solutions[best_gid]->runtime_phase_cost;
    for (int groupId = 0; groupId < param_num_group; groupId++) {
        auto p = solutions[groupId];
        p->print();
        num_of_restart += solutions[groupId]->num_restart;
    }
    int iteration1 = 0;
    int iteration2 = 0;
    int num_of_ll_search = 0;
    double runtime_share_data = 0;
    for (int i = 0; i < param_num_solver; i++) {
        iteration1 += solvers[i]->num_init_iteration;
        iteration2 += solvers[i]->iteration_stats.size();
        num_of_ll_search += solvers[i]->num_of_ll_search;
        runtime_share_data += solvers[i]->runtime_share_data / param_num_solver;
    }
    float subopt = sum_of_costs * 1.0 / solutions[0]->sum_of_distances;
    cout.setf(std::ios::fixed, std::ios::floatfield);
    cout << std::setprecision(3);
    cout << "final"
         << " colliding " << num_of_colliding_pairs << " costs " << sum_of_costs
         << " costs_wc " << sum_of_costs_wc << " restart " << num_of_restart
         << " a*/s " << int(num_of_ll_search / searchTime) 
         << " iter " << iteration1 << " " << iteration2 
         << " iter/s " << (int)((iteration1 + iteration2) / searchTime) 
         << " t1 " << (int)runtime_phase_collision
         << " t2 " << (int)runtime_phase_cost 
         << " subopt " << std::round(subopt * 10000) / 10000 
         << " mem " << std::round(max_mem * 100) / 100 
         << " t_share " << runtime_share_data
         << " total " << searchTime << endl;

    // if (vm.count("output"))
    // 	solution_shared->writeResultToFile(vm["output"].as<string>(),
    // instance.getInstanceName());

    for (int i = 0; i < (int)solvers.size(); i++) {
        if (lns_1st != solvers[i])
            delete solvers[i];
    }
    delete lns_1st;

    return 0;
}