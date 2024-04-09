#include "ShareDataManager.h"

#include "common.h"

ShareDataManager::ShareDataManager(int screen, bool use_share,
                                   bool use_share_wc, vector<int> &group_list, 
                                   int np, int np_init)
    : screen(screen), np(np), np_init(np_init) {
    numGroupPP = group_list.size();
    int sum = 0;
    for (int i = 0; i < numGroupPP; i++) {
        SolutionShared *solution_shared =
            new SolutionShared(screen - 1, use_share, use_share_wc);
        solution_shared->myGroup = i;
        solution_shared->numLocalSolver = group_list[i];
        sum += group_list[i];
        solutions.push_back(solution_shared);
    }
    assert(sum == np_init);

    pthread_barrier_init(&barrier_stop_all_init_solver, NULL, np_init);
    pthread_barrier_init(&barrier_all_threads_restart1, NULL, np);
    pthread_barrier_init(&barrier_all_threads_restart2, NULL, np);

    for (int groupId = 0; groupId < numGroupPP; groupId++) {
        int numLocalSolver = group_list[groupId];
        log(0, "create group %d with %d solvers.\n", groupId, numLocalSolver);
        auto ss = solutions[groupId];
        int n = numLocalSolver;
        if (numLocalSolver == 1) {
            ss->use_share_phase_collision = 0;
        }
        pthread_barrier_init(&ss->barrier_phase_cost_start1, NULL, n);
        pthread_barrier_init(&ss->barrier_phase_cost_start2, NULL, n);
        pthread_barrier_init(&ss->barrier_phase_cost_start3, NULL, n);
        pthread_barrier_init(&ss->barrier_phase_cost_restart1, NULL, n);
        pthread_barrier_init(&ss->barrier_phase_collision_restart, NULL, n);

        pthread_barrier_init(&ss->barrier_phase_cost_restart2, NULL, n);
        pthread_barrier_init(&ss->barrier_phase_collision_result, NULL, n);
        pthread_barrier_init(&ss->barrier_phase_collision_result_31, NULL, n);
        pthread_barrier_init(&ss->barrier_phase_collision_result_32, NULL, n);
        pthread_barrier_init(&ss->barrier_phase_collision_result_33, NULL, n);
        pthread_barrier_init(&ss->barrier_phase_collision_result_34, NULL, n);        
    }
}

SolutionShared *ShareDataManager::get_current_best_group_solution() {
    //找到当前最好的group
    groupLock.lock();
    if (best_group_index == -1) {
        int best_cost = MAX_COST;
        for (auto ss : solutions) {
            if(screen > 0)
                printf("ss %d colli %d cost %d %d\n", ss->myGroup,
                    (int)ss->num_of_colliding_pairs.load(), ss->sum_of_costs,
                    ss->sum_of_costs_wc);
            if (ss->num_of_colliding_pairs > 0) continue;
            // int cost = std::min(ss->sum_of_costs, ss->sum_of_costs_wc);
            // ss->sum_of_costs = cost;  //把sum_of_costs的改变交给update
            int cost = ss->sum_of_costs;
            if (cost < best_cost) {
                best_cost = cost;
                best_group_index = ss->myGroup;
            }
        }
    }
    // printf("===> best_group_index %d return %ld\n", (int)best_group_index,
    //        (void *)solutions[best_group_index]);
    // printf("===> best_group_index %d\n", (int)best_group_index);
    groupLock.unlock();
    if (best_group_index == -1)
        return solutions[0];
    else
        return solutions[best_group_index];
}

int ShareDataManager::getCurrentBestGroupId(
    vector<SolutionShared *> &solutions) {
    int num_of_colliding_pairs = MAX_COST;
    int sum_of_costs = MAX_COST;
    int sum_of_costs_wc = MAX_COST;
    int num_remain_ag = MAX_COST;
    double runtime_phase_collision = 0;
    double runtime_phase_cost = 0;
    int num_of_restart = 0;
    int best_groupId = 0;
    for (int groupId = 0; groupId < numGroupPP; groupId++) {
        auto p = solutions[groupId];
        // p->print();
        if (p->num_of_colliding_pairs == 0 && num_of_colliding_pairs > 0 ||
            p->num_of_colliding_pairs == 0 && num_of_colliding_pairs == 0 &&
                p->sum_of_costs < sum_of_costs ||
            p->num_of_colliding_pairs > 0 && num_of_colliding_pairs > 0 &&
                p->num_of_colliding_pairs < num_of_colliding_pairs)
        // num_remain_ag > 10 && p->num_of_init_remaining_agents <
        // num_remain_ag)
        {
            num_of_colliding_pairs = p->num_of_colliding_pairs;
            sum_of_costs = p->sum_of_costs;
            sum_of_costs_wc = p->sum_of_costs_wc;
            runtime_phase_collision = p->runtime_phase_collision;
            runtime_phase_cost = p->runtime_phase_cost;
            num_remain_ag = p->num_of_init_remaining_agents;
            best_groupId = groupId;
        }
        num_of_restart += p->num_restart;
        ;
    }
    // update the g_result
    int cost = sum_of_costs;
    if (num_of_colliding_pairs == 0 && g_num_of_colliding_pairs > 0 ||
        num_of_colliding_pairs == 0 && g_num_of_colliding_pairs == 0 &&
            cost < g_sum_of_costs ||
        num_of_colliding_pairs > 0 && g_num_of_colliding_pairs > 0 &&
            num_of_colliding_pairs < g_num_of_colliding_pairs) {
        g_num_of_colliding_pairs = num_of_colliding_pairs;
        g_sum_of_costs = cost;
    }

    return best_groupId;
}

void ShareDataManager::print(double mem_use_rate) {
    int groupId = getCurrentBestGroupId(solutions);
    if (screen > 0) {
        for (int groupId = 0; groupId < numGroupPP; groupId++) {
            string s = solutions[groupId]->print();
            printf("[%.0f]-%d-%d-m%.2f %s\n", getTime(), g_num_of_colliding_pairs,
                   g_sum_of_costs, mem_use_rate, s.c_str());
        }
    } else {
        string s = solutions[groupId]->print();
        printf("[%.0f]-%d-%d-m%.2f %s\n", getTime(), g_num_of_colliding_pairs,
               g_sum_of_costs, mem_use_rate, s.c_str());
    }
}

void ShareDataManager::destory_barrier() {
    pthread_barrier_destroy(&barrier_stop_all_init_solver);
    pthread_barrier_destroy(&barrier_all_threads_restart1);
    pthread_barrier_destroy(&barrier_all_threads_restart2);
    for (int groupId = 0; groupId < numGroupPP; groupId++) {
        auto ss = solutions[groupId];
        pthread_barrier_destroy(&ss->barrier_phase_cost_start1);
        pthread_barrier_destroy(&ss->barrier_phase_cost_start2);
        pthread_barrier_destroy(&ss->barrier_phase_cost_start3);
        pthread_barrier_destroy(&ss->barrier_phase_cost_restart1);
        pthread_barrier_destroy(&ss->barrier_phase_cost_restart2);

        pthread_barrier_destroy(&ss->barrier_phase_collision_restart);
        pthread_barrier_destroy(&ss->barrier_phase_collision_result);
        pthread_barrier_destroy(&ss->barrier_phase_collision_result_31);
        pthread_barrier_destroy(&ss->barrier_phase_collision_result_32);
        pthread_barrier_destroy(&ss->barrier_phase_collision_result_33);
        pthread_barrier_destroy(&ss->barrier_phase_collision_result_34);
    }
}