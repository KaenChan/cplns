#include "defines.h"

// param for sa in phase_collision initLNS
bool param_use_simulated_annealing = true;
double param_simulated_annealing_T = 1000;
double param_sa_restart_coolfactor = 1.0;
double param_sa_iteration_coolfactor = 0.99;
double param_sa_max_con_fail = 0.4;
bool param_use_fast_run = false;       // 弃用
bool param_sa_restart_resume = false;  // 弃用

double param_costlns_max_con_fail = 0.7;

// param for sa in phase_cost LNS
bool param_use_simulated_annealing_phase_cost = false;
double param_simulated_annealing_T_phase_cost = 10;
double param_sa_iteration_coolfactor_phase_cost = 0.99;

// param for parallel
int param_num_solver = 4;
int param_num_solver_init = 4;
int param_num_group = 1;
int param_share_step_phase_collision = 1;
int param_share_step_phase_cost = 1;

// param for parallel lns sa
int param_num_of_run_after_sa_accept = 100;

// weighted sipps
float param_single_agent_solver_f_w = 1.0;
float param_single_agent_solver_f_w_p = 3.;
int param_use_fixed_wh1_phase_cost = 0;

thread_local bool tl_use_simulated_annealing = true;
thread_local float tl_single_agent_solver_f_w = 1.0;

namespace simulated_annealing {
thread_local double sa_iteration_T_init = 0.0;  // 会变化
thread_local double sa_iteration_T = 10.0;
int sa_iteration_coolstep = 1;

thread_local double sa_iteration_coolfactor = 0.99;

void _init_temp(int T) {
    sa_iteration_T = T;
    sa_iteration_T_init = T;
}

void reset_temp(int T, int restart_coolfactor, double iter_coolfactor) {
    if (sa_iteration_T_init < 1e-8) _init_temp(T);
    sa_iteration_T = sa_iteration_T_init;
    sa_iteration_T_init *= restart_coolfactor;
    sa_iteration_coolfactor = iter_coolfactor;
}

void update_temp(int iteration) {
    if (iteration % sa_iteration_coolstep == 0)
        sa_iteration_T *= sa_iteration_coolfactor;
}

bool can_accept(double df, int screen, int iteration) {
    double sj = std::rand() % 10000;
    sj /= 10000;
    bool b = exp(-df / (1e-8 + sa_iteration_T)) > sj;
    if (b && screen >= 1)
        cout << "df " << df << " T " << sa_iteration_T << " exp "
             << exp(-df / sa_iteration_T) << " > " << sj << " = " << b << endl;
    // return false;
    return b;
}
}  // namespace simulated_annealing
