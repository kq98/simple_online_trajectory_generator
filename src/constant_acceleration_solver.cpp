#include "sotg/constant_acceleration_solver.hpp"

using namespace SOTG;
using namespace detail;

int findIndexOfMax(const std::vector<double>& values);
double calcPhaseLength(const Phase& phase);

double calcPhaseLength(const Phase& phase)
{
    double sum = 0.0;
    for (auto& component : phase.components) {
        sum += std::pow(component.second.length, 2);
    }
    return std::sqrt(sum);
}

int findIndexOfMax(const std::vector<double>& values)
{
    int index_max = 0;
    double max_value = values[0];
    for (size_t i = 1; i < values.size(); i++) {
        if (values[i] > max_value) {
            max_value = values[i];
            index_max = i;
        }
    }

    return index_max;
}

void ConstantAccelerationSolver::calcPhaseDurationAndDistance(double a_max, double v_max, double L_total,
                                                           PhaseComponent& acc_phase_single_dof,
                                                           PhaseComponent& coast_phase_single_dof,
                                                           PhaseComponent& dec_phase_single_dof)
{
    double T_acc;
    double T_coast;
    double T_dec;
    double L_acc;
    double L_coast;
    double L_dec;

    if (utility::nearlyZero(a_max)) {
        T_acc = 0.0;
        L_acc = 0.0;
    } else {
        T_acc = v_max / a_max;
        L_acc = 0.5 * a_max * std::pow(T_acc, 2);
    }
    acc_phase_single_dof.duration = T_acc;
    acc_phase_single_dof.length = L_acc;
    acc_phase_single_dof.distance_p_start = 0.0;

    if (utility::nearlyZero(v_max)) {
        T_dec = 0.0;
    } else {
        T_dec = v_max / a_max;
    }

    L_dec = -0.5 * a_max * std::pow(T_dec, 2) + v_max * T_dec;
    dec_phase_single_dof.duration = T_dec;
    dec_phase_single_dof.length = L_dec;

    L_coast = L_total - L_acc - L_dec;
    if (utility::nearlyZero(v_max)) {
        T_coast = 0.0;
    } else {
        T_coast = L_coast / v_max;
    }
    coast_phase_single_dof.duration = T_coast;
    coast_phase_single_dof.length = L_coast;
    coast_phase_single_dof.distance_p_start = L_acc;

    dec_phase_single_dof.distance_p_start = L_acc + L_coast;
}

void ConstantAccelerationSolver::calcDurationAndDistancePerGroup(double& a_max, double& v_max, double distance,
                                               double& duration)
{
    double T_acc;
    double T_coast;
    double T_dec;
    double L_acc;
    double L_coast;
    double L_dec;

    if (utility::nearlyZero(a_max)) {
        T_acc = 0.0;
        L_acc = 0.0;
    } else {
        T_acc = v_max / a_max;
        L_acc = 0.5 * a_max * std::pow(T_acc, 2);
    }

    if (utility::nearlyZero(v_max)) {
        T_dec = 0.0;
    } else {
        T_dec = v_max / a_max;
    }

    L_dec = -0.5 * a_max * std::pow(T_dec, 2) + v_max * T_dec;

    L_coast = distance - L_acc - L_dec;
    if (utility::nearlyZero(v_max)) {
        T_coast = 0.0;
    } else {
        T_coast = L_coast / v_max;
    }

    duration = T_acc + T_coast + T_dec;

    if (L_coast < 0.0 && !(std::abs(L_coast) < 1e-6)) {
        double v_max_reduced = std::sqrt(distance * a_max);

        calcDurationAndDistancePerGroup(a_max, v_max_reduced, distance, duration);
        v_max = v_max_reduced;
    }
}

void ConstantAccelerationSolver::calcDurationAndDistance(Phase& acc_phase, Phase& coast_phase,
                                                              Phase& dec_phase, Point& start_point, Point& end_point,
                                                              std::vector<double>& duration_per_group,
                                                              std::vector<double>& distance_per_group, std::map<std::string, double>& group_acc, std::map<std::string, double>& group_vel)
{
    std::vector<std::string> group_names;
    for (const auto& [group_name, symbols] : symbol_map_)
    {
        double duration;
        double distance;

        if(symbols.isQuaternion()) {
            distance = end_point[group_name].getQuaternionValues().angularDistance(start_point[group_name].getQuaternionValues());

        } else {
            distance = (end_point[group_name].getCartesianValues() - start_point[group_name].getCartesianValues()).norm();
        }
            distance_per_group.push_back(distance);

            double a_max = end_point[group_name].getMaxAcc();
            double v_max = end_point[group_name].getMaxVel();

            calcDurationAndDistancePerGroup(a_max,v_max, distance, duration);

            if (v_max < end_point[group_name].getMaxVel()) {
                logger_.log("[Section between Points " + std::to_string(start_point.getID()) + "(" + start_point.getName() + ")" + " and " + std::to_string(end_point.getID()) + "(" + end_point.getName() + ")]"
                                + "[" + group_name + "] KinematicSolver: Decreasing maximum velocity from "
                                + std::to_string(end_point[group_name].getMaxVel()) + " to " + std::to_string(v_max),
                            Logger::INFO);
            }

            group_vel[group_name] = v_max;
            group_acc[group_name] = a_max;
            duration_per_group.push_back(duration);
            group_names.push_back(group_name);
    }

    int index_slowest_group = findIndexOfMax(duration_per_group);

    for (size_t i = 0; i < distance_per_group.size(); ++i)
    {
        double lambda = 0.0; // Time sync between groups
        if (!utility::nearlyZero(distance_per_group[index_slowest_group])) {
            lambda = distance_per_group[i] / distance_per_group[index_slowest_group];
        }

        std::string current_group = group_names[i];
        double a_max = group_acc[current_group];
        double v_max = group_vel[current_group];

        double reduced_a_max = lambda * a_max;
        double reduced_v_max = lambda * v_max;

        group_acc[current_group] = reduced_a_max;
        group_vel[current_group] = reduced_v_max;

        double section_length = distance_per_group[i];
        PhaseComponent acc_phase_per_group, coast_phase_per_group, dec_phase_per_group;
        calcPhaseDurationAndDistance(reduced_a_max, reduced_v_max, section_length, acc_phase_per_group,
                                 coast_phase_per_group, dec_phase_per_group);

        acc_phase.components[current_group] = acc_phase_per_group;
        coast_phase.components[current_group] = coast_phase_per_group;
        dec_phase.components[current_group] = dec_phase_per_group;
    }

    // Find the first component duration that is not 0.0 because all of them have the same value or 0.0
    auto it_acc
        = std::find_if(acc_phase.components.begin(), acc_phase.components.end(),
                       [](const auto &it) { return !(utility::nearlyZero(it.second.duration)); });
    if (it_acc != acc_phase.components.end()) {
        acc_phase.duration = it_acc->second.duration;
    }

    auto it_coast
        = std::find_if(coast_phase.components.begin(), coast_phase.components.end(),
                       [](const auto &it) { return !(utility::nearlyZero(it.second.duration)); });
    if (it_coast != coast_phase.components.end()) {
        coast_phase.duration = it_coast->second.duration;
    }

    auto it_dec
        = std::find_if(dec_phase.components.begin(), dec_phase.components.end(),
                       [](const auto &it) { return !(utility::nearlyZero(it.second.duration)); });
    if (it_dec != dec_phase.components.end()) {
        dec_phase.duration = it_dec->second.duration;
    }
}

Section ConstantAccelerationSolver::calcSection(Point& p_start_ref, Point& p_end_ref, size_t section_id)
{
    /*
    
    Calculate the maximum velocity that can be used in a trajectory between two waypoints(Section), this might be smaller or the same as the maximum set by the user.
    Different ValueGroups, might traverse the same path between two points at different speeds, therfor all ValueGroups need to be time synchronised, 
    see https://www.diag.uniroma1.it/~deluca/rob1_en/14_TrajectoryPlanningCartesian.pdf such that all ValueGroups start and finish at the same time.

    */

    Section section(p_start_ref, p_end_ref, section_id);

    std::vector<double> duration_per_group;
    std::vector<double> distance_per_group;
    std::vector<Phase> phases;
    Phase acc_phase, coast_phase, dec_phase;

    std::map<std::string, double>& v_max = section.getVelocities();
    std::map<std::string, double>& a_max = section.getAccelerations();


    calcDurationAndDistance(acc_phase, coast_phase, dec_phase, p_start_ref, p_end_ref, duration_per_group, distance_per_group, a_max, v_max);

    // slowest ValueGroup is identified in order to slow all other groups such that they require the same time.
    int index_slowest_dof = findIndexOfMax(duration_per_group);
    double T_total = duration_per_group[index_slowest_dof];
    section.setIndexSlowestDoF(index_slowest_dof);
    section.setDuration(T_total);

    acc_phase.type = PhaseType::ConstantAcceleration;
    acc_phase.length = calcPhaseLength(acc_phase);
    phases.push_back(acc_phase);

    coast_phase.type = PhaseType::ConstantVelocity;
    coast_phase.t_start = acc_phase.duration;
    coast_phase.length = calcPhaseLength(coast_phase);
    coast_phase.distance_p_start = acc_phase.length;
    phases.push_back(coast_phase);

    dec_phase.type = PhaseType::ConstantDeacceleration;
    dec_phase.t_start = coast_phase.duration + coast_phase.t_start;
    dec_phase.length = calcPhaseLength(dec_phase);
    dec_phase.distance_p_start = coast_phase.distance_p_start + coast_phase.length;
    phases.push_back(dec_phase);

    section.setPhases(phases);

    return section;
}

void ConstantAccelerationSolver::calcPosAndVelSingleGroup(double section_distance, const Phase& phase,
                                                               double phase_distance_to_p_start, double t_phase,
                                                               double a_max_reduced, double v_max_reduced,
                                                               double& pos, double& vel) const
{
    double p_i{0}, v_i{0};
    if (phase.type == PhaseType::ConstantAcceleration) {
        p_i = 0.5 * a_max_reduced * std::pow(t_phase, 2);
        v_i = a_max_reduced * t_phase;
    } else if (phase.type == PhaseType::ConstantVelocity) {
        p_i = v_max_reduced * t_phase + phase_distance_to_p_start;
        v_i = v_max_reduced;
    } else if (phase.type == PhaseType::ConstantDeacceleration) {
        p_i = -0.5 * a_max_reduced * std::pow(t_phase, 2) + v_max_reduced * t_phase + phase_distance_to_p_start;
        v_i = -a_max_reduced * t_phase + v_max_reduced;
    }
    if (utility::nearlyZero(section_distance)) {
        pos = 0;
    } else {
        pos = p_i / section_distance;
    }
    vel = v_i;
}

void ConstantAccelerationSolver::calcPosAndVelSection(double t_section, Section& section, Result& result) const
{
    Point p_start = section.getStartPoint();
    Point p_end = section.getEndPoint();

    Point diff = p_end - p_start;

    Phase& phase = section.getPhaseByTime(t_section);
    double t_phase = t_section - phase.t_start;

    for (const auto& [group_name, symbols] : symbol_map_)
    {
        double a_max = section.getAccelerations()[group_name];
        double v_max = section.getVelocities()[group_name];

        ValueGroup diff_group = diff[group_name];
        ValueGroup& loc = result.getLocation()[group_name];
        ValueGroup& vel = result.getVelocity()[group_name];

        if (symbols.isQuaternion()) {
            double diff_angle = Eigen::AngleAxisd(diff_group.getQuaternionValues()).angle();

            // pos_interpolation_factor == [0,1] based on the requested point in time
            double pos_interpolation_factor, vel_interpolation_factor;
            calcPosAndVelSingleGroup(diff_angle, phase, phase.components[group_name].distance_p_start, t_phase,
                                     a_max, v_max, pos_interpolation_factor, vel_interpolation_factor);

            Eigen::Quaterniond startQuat = p_start[group_name].getQuaternionValues();
            Eigen::Quaterniond endQuat = p_end[group_name].getQuaternionValues();

            loc = startQuat.slerp(pos_interpolation_factor, endQuat);

            Eigen::Vector3d vel_dir = Eigen::AngleAxisd(diff_group.getQuaternionValues().normalized()).axis();
            Eigen::Vector3d ang_vel = vel_dir * vel_interpolation_factor;

            // vel is a Quaternion for convinience, it is used as a container for the angle velocity but no longer represent a rotation or orientation
            vel = Eigen::Quaterniond(0.0, ang_vel.x(), ang_vel.y(), ang_vel.z());

        } else {

            double diff_magnitude = diff_group.getCartesianValues().norm();
            ValueGroup dir_group = diff_group * (1.0 / diff_magnitude);

            double pos_magnitude, vel_magnitude;
            calcPosAndVelSingleGroup(diff_magnitude, phase, phase.components[group_name].distance_p_start, t_phase,
                                     a_max, v_max, pos_magnitude, vel_magnitude);
            loc = p_start[group_name] + diff_group * pos_magnitude;
            vel = dir_group * vel_magnitude;
        }

    }
}
