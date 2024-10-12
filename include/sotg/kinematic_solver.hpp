#pragma once

#include <map>
#include <memory>
#include <string>

#include "sotg/logger.hpp"
#include "sotg/section.hpp"
#include "sotg/result.hpp"

namespace SOTG {
namespace detail {
    // Derived classes of this base class implement the logic for section and
    // segment generation aswell as the calculation of position and velocity for a
    // specific point in time
    class KinematicSolver {
    protected:
        SymbolGroupMap& symbol_map_;
        const Logger& logger_;

    public:
        KinematicSolver(SymbolGroupMap& symbol_map, const Logger& logger)
            : symbol_map_(symbol_map), logger_(logger)
        {
        }

        virtual Section calcSection(Point& p_start_ref, Point& p_end_ref, size_t section_id) = 0;

        virtual void calcPosAndVelSection(double t_section, Section& section, Result& result) const = 0;

        virtual ~KinematicSolver() = default;
    };
}  // namespace detail
}  // namespace SOTG
