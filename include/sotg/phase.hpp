#pragma once

#include <cmath>
#include <vector>
#include <map>
#include <string>

namespace SOTG {
namespace detail {

    enum PhaseType { ConstantAcceleration, ConstantVelocity, ConstantDeacceleration };

    inline const char* ToString(PhaseType type)
    {
        switch (type) {
        case ConstantAcceleration:
            return "ConstantAcceleration";
        case ConstantVelocity:
            return "ConstantVelocity";
        case ConstantDeacceleration:
            return "ConstantDeacceleration";
        default:
            return "[Unknown Phase Type]";
        }
    }

    struct PhaseComponent {
        double duration = 0.0;
        double length = 0.0;
        double distance_p_start = 0.0;
    };

    // A specific kinematic state that applies to a specific part of a section
    struct Phase {
        std::map<std::string ,PhaseComponent> components;

        double duration = 0.0;
        double length = 0.0;

        double t_start = 0.0;
        double distance_p_start = 0.0;

        PhaseType type;
    };
}  // namespace detail
}  // namespace SOTG