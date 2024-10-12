#pragma once

#include <algorithm>
#include <vector>

#include "sotg/phase.hpp"
#include "sotg/point.hpp"

namespace SOTG {
namespace detail {

    // Connects to waypoints of a path and is limited by an instance of SectionConstraint
    // Stores kinematic states for position and velocity calculations along this sectionin inside of phases
    class Section {
    private:
        Point& start_point_;
        Point& end_point_;

        const SymbolGroupMap& symbol_groups_;

        double length_;

        std::vector<Phase> phases_;

        int index_slowest_dof_ = -1;

        std::map<std::string, double> a_max_;
        std::map<std::string, double> v_max_;

        double duration_ = 0.0;
        double start_time_ = 0.0;

        size_t id_;

        // time_shift is the duration of the preceeding blend segment, without blending it is always zero
        double time_shift_ = 0.0;

    public:
        const Point& getStartPoint() const { return start_point_; }
        const Point& getEndPoint() const { return end_point_; }

        double getLength() const { return length_; }
        // const Point& getDirection() const { return dir_; }
        // const Point& getDifference() const { return diff_; }
        // void setLength(double length) { length_ = length; }
        // void setDirection(const Point& dir) { dir_ = dir; }
        // void setDifference(const Point& diff) { diff_ = diff; }

        Section(Point& p_start_ref, Point& p_end_ref, size_t section_id)
            : start_point_(p_start_ref)
            , end_point_(p_end_ref)
            , symbol_groups_(p_start_ref.getSymbolMap())
            , id_(section_id)
        {
        }

        void setIndexSlowestDoF(int index) { index_slowest_dof_ = index; }

        void setDuration(double time) { duration_ = time; }
        double getDuration() const { return duration_; }

        std::map<std::string, double>& getAccelerations() { return a_max_; }
        std::map<std::string, double>& getVelocities() { return v_max_; }

        Phase& getPhaseByDistance(double distance);
        Phase& getPhaseByTime(double time);
        void setPhases(const std::vector<Phase>& new_phases) { phases_ = new_phases; }
        std::vector<Phase>& getPhases() { return phases_; }

        double getStartTime() const { return start_time_; }
        double getEndTime() const { return start_time_ + duration_; }
        void setStartTime(double new_start_time) { start_time_ = new_start_time; }

        int getIndexSlowestDoF() const { return index_slowest_dof_; }

        Phase& getPhaseByType(PhaseType type);

        void setID(size_t new_id) { id_ = new_id; }
        size_t getID() { return id_; }
    };
}  // namespace detail
}  // namespace SOTG