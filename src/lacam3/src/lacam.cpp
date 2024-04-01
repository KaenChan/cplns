#include "../include/lacam.hpp"

using namespace lacam;

Solution lacam_solve(const InstanceLa &ins, int verbose, const Deadline *deadline,
                     int seed, DistTable *D, std::function<void(const lacam::Solution &)> func)
{
  info(1, verbose, deadline, "pre-processing");
  auto planner = Planner(&ins, verbose, deadline, seed, D, 0, func);
  return planner.solve();
}
