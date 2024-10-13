#include "sotg/trajectory_generator.hpp"

#include <iostream>
#include <memory>

using namespace SOTG;
using namespace detail;

TrajectoryGenerator::TrajectoryGenerator(SymbolGroupMap symbol_groups)
    : symbol_group_map_(symbol_groups)
    , default_logger_(new Logger())
    , logger_(*default_logger_)
    , kinematic_solver_(new detail::ConstantAccelerationSolver(symbol_group_map_, logger_))
{
    path_manager_
        = std::unique_ptr<detail::PathManager>(new detail::PathManager(kinematic_solver_, debug_info_vec_));
}

TrajectoryGenerator::TrajectoryGenerator(SymbolGroupMap symbol_groups, const Logger& logger)
    : symbol_group_map_(symbol_groups)
    , logger_(logger)
    , kinematic_solver_(new detail::ConstantAccelerationSolver(symbol_group_map_, logger_))
{
    path_manager_
        = std::unique_ptr<detail::PathManager>(new detail::PathManager(kinematic_solver_, debug_info_vec_));
}

void TrajectoryGenerator::resetPath(Path path) { path_manager_->resetPath(path); }

double TrajectoryGenerator::getDuration()
{
    double total_time = 0.0;
    for (const auto section : path_manager_->getSections()) {
        total_time += section.getDuration();
    }
    return total_time;
}

int SOTG::TrajectoryGenerator::getNumPassedWaypoints(double tick)
{
    const Section& section = path_manager_->getSectionAtTime(tick);
    const Point& section_start_point = section.getStartPoint();

    return section_start_point.getID();

}

void TrajectoryGenerator::calcPositionAndVelocity(double time, Result& result)
{
    Section& section = path_manager_->getSectionAtTime(time);

    double t_section = time - section.getStartTime();

    kinematic_solver_->calcPosAndVelSection(t_section, section, result);
}