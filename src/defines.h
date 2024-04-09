#pragma once
#include "common.h"

// param for sa in phase_collision initLNS
extern bool param_use_simulated_annealing;
extern double param_simulated_annealing_T;
extern double param_sa_restart_coolfactor;
extern double param_sa_iteration_coolfactor;
extern double param_sa_max_con_fail;
extern bool param_use_fast_run;       // 弃用
extern bool param_sa_restart_resume;  // 弃用

extern bool param_use_early_stop;

extern double param_costlns_max_con_fail;

// param for sa in phase_cost LNS
extern bool param_use_simulated_annealing_phase_cost;
extern double param_simulated_annealing_T_phase_cost;
extern double param_sa_iteration_coolfactor_phase_cost;

// param for parallel lns sa
extern int param_share_step_phase_collision;
extern int param_share_step_phase_cost;

// param for parallel
extern int param_num_solver;
extern int param_num_solver_init;
extern int param_num_solver_pp;
extern int param_num_group_pp;

extern string param_hybrid_init_algo_type;
extern int param_num_solver_hybrid;

// weighted sipps
extern float param_single_agent_solver_f_w;
extern float param_single_agent_solver_f_w_p;
extern int param_use_fixed_wh1_phase_cost;

//lacam_pibt_num
extern int param_lacam_pibt_num;
extern bool param_lacam_star;
extern bool param_lacam_star_last_thread;
extern bool param_lacam_recv_shared_solution;

// param for each thread
extern thread_local bool tl_use_simulated_annealing;
extern thread_local float tl_single_agent_solver_f_w;

namespace simulated_annealing {
extern thread_local double sa_iteration_T_init;
extern thread_local double sa_iteration_T;

void reset_temp(int T, int restart_coolfactor, double iter_coolfactor);
void update_temp(int iteration);
bool can_accept(double df, int screen, int iteration);
}  // namespace simulated_annealing
