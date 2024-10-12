
#pragma once

#include <memory>

#include "sotg/constant_acceleration_solver.hpp"
#include "sotg/kinematic_solver.hpp"
#include "sotg/logger.hpp"
#include "sotg/path.hpp"
#include "sotg/path_manager.hpp"
#include "sotg/path_manager.hpp"
#include "sotg/point.hpp"
#include "sotg/section.hpp"
#include "sotg/symbol_group.hpp"
#include "sotg/result.hpp"

namespace SOTG {

// The enty point for interactions with SOTG in the form of new input or position and velocity calculations for a
// specifc point in time
class TrajectoryGenerator2 {
private:
    SymbolGroupMap symbol_group_map_;

    std::shared_ptr<Logger> default_logger_;
    const Logger& logger_;

    std::unique_ptr<detail::PathManager> path_manager_;
    std::shared_ptr<detail::KinematicSolver> kinematic_solver_;

    std::vector<std::map<std::string, double>> debug_info_vec_;

public:
    TrajectoryGenerator(SymbolGroupMap symbol_groups);
    TrajectoryGenerator(SymbolGroupMap symbol_groups, const Logger& logger);

    double getDuration();
    int getNumPassedWaypoints(double tick);
    void calcPositionAndVelocity(double time, Result& result);
    void resetPath(Path path);

    std::vector<std::map<std::string, double>>& getDebugInfo() { return debug_info_vec_; };
};
}  // namespace SOTG