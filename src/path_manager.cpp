#include "sotg/path_manager.hpp"

#include <iostream>

using namespace SOTG;
using namespace detail;

PathManager::PathManager(std::shared_ptr<KinematicSolver> solver_ptr,
                           std::vector<std::map<std::string, double>>& debug_info_vec_tg)
    : kinematic_solver_(solver_ptr)
    , debug_info_vec_(debug_info_vec_tg)
{
}

void PathManager::resetSections()
{
    sections_.clear();

    size_t num_sections = path_.getNumWaypoints() - 1;
    for (size_t i = 0; i < num_sections; i++) {
        Section section = kinematic_solver_->calcSection(path_.getPoint(i), path_.getPoint(i + 1), i);

        sections_.push_back(section);
    }

    double section_time_offset = 0.0;
    for (Section& section : sections_) {
        double duration = section.getDuration();
        section.setStartTime(section_time_offset);
        section_time_offset += duration;

#ifdef DEBUG
        std::cout << "Section start time: " << section.getStartTime() << ", duration: " << section.getDuration()
                  << std::endl;
#endif
    }
}

void PathManager::resetPath(Path new_path)
{
    path_ = new_path;

    resetSections();
}

std::ostream& PathManager::operator<<(std::ostream& out)
{
    out << "[ ";

    if (!sections_.empty()) {
        std::list<Section>::iterator it;
        for (it = sections_.begin(); it != std::prev(sections_.end()); ++it) {
            out << it->getStartPoint() << ", ";
        }
        out << sections_.back().getStartPoint() << " ]";
    } else {
        out << " ]";
    }

    return out;
}

Section& PathManager::getSectionAtTime(double time)
{
    double last_t_end = 0.0;
    for (Section& section : sections_) {
        // Explanation for time_shift in the Section class
        // double time_shift = section.getTimeShift();
        double t_end = section.getStartTime() + section.getDuration();  // - time_shift;
        if (time > last_t_end && (time < t_end || (utility::nearlyEqual(time, t_end, 1e-6)))) {
            return section;
        }
        last_t_end = t_end;
    }
    if (utility::nearlyEqual(time, 0.0, 1e-6)) {
        return sections_.front();
    }

    throw std::runtime_error("Requested time \"" + std::to_string(time)
                             + "\" could not be mapped to any section!");
}