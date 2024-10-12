#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>

#include <bits/stdc++.h>

#include "sotg/kinematic_solver.hpp"
#include "sotg/section.hpp"
#include "sotg/utility_functions.hpp"

#include "sotg/result.hpp"

namespace SOTG {
namespace detail {

    // Implements the logic for section and segment generation aswell as the
    // calculation of positions and velocities for specific points in time using a
    // bang coast bang profile
    class ConstantAccelerationSolver : public KinematicSolver {
    private:
        void calcPhaseDurationAndDistance(double a_max, double v_max, double L_total, PhaseComponent& acc_phase_single_dof,
                                      PhaseComponent& coast_phase_single_dof, PhaseComponent& dec_phase_single_dof);

        void calcDurationAndDistancePerGroup(double& a_max, double& v_max, double distance,
                                               double& duration);

        void calcDurationAndDistance(Phase& acc_phase, Phase& coast_phase, Phase& dec_phase,Point& start_point, Point& end_point,
                                         std::vector<double>& total_time_per_dof,
                                         std::vector<double>& total_length_per_dof, std::map<std::string, double>& group_acc, std::map<std::string, double>& group_vel);

        void calcPosAndVelSingleGroup(double section_length, const Phase& phase,
                                          double phase_distance_to_p_start, double t_phase, double a_max_reduced,
                                          double v_max_reduced, double& pos_magnitude,
                                          double& vel_magnitude) const;

    public:
        ConstantAccelerationSolver(SymbolGroupMap& symbol_map, const Logger& logger)
            : KinematicSolver(symbol_map, logger)
        {
        }

        Section calcSection(Point& p_start_ref, Point& p_end_ref, size_t section_id) override;

        void calcPosAndVelSection(double t_section, Section& section, Result& result) const override;
    };
}  // namespace detail
}  // namespace SOTG