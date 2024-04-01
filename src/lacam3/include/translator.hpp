/*
 * translate between representations by paths and configurations
 */

#pragma once
#include "graph.hpp"
#include "metrics.hpp"
#include "utils.hpp"

namespace lacam {
std::vector<Path> translateConfigsToPaths(const std::vector<Config> &configs);
std::vector<Config> translatePathsToConfigs(const std::vector<Path> &paths);
}
