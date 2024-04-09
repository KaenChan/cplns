#include "../include/lacam.hpp"

using namespace lacam;

Solution lacam_solve(const InstanceLa &ins, int verbose,
                       const Deadline *deadline, int seed, DistTable *D,
                       std::function<int(lacam::Solution &)> func,
                       std::atomic<bool> *interrupt, int *is_running) {
    info(1, verbose, deadline, "pre-processing");
    auto planner = Planner(&ins, verbose, deadline, seed, D, 0);
    planner.func = func;
    planner.is_running = is_running;
    return planner.solve(interrupt);
}
