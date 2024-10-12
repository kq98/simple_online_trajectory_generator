#pragma once

#include "sotg/symbol_group.hpp"
#include "sotg/value_group.hpp"
#include "sotg/point.hpp"

namespace SOTG {

class Result
{
private:
    SymbolGroupMap& symbol_map_;
    Point location, velocity;

public:
    Result(SymbolGroupMap& symbol_map) : symbol_map_(symbol_map), location(symbol_map, "location"), velocity(symbol_map, "velocity") {

    }

    Point& getLocation() { return location; }
    Point& getVelocity() { return velocity; }
};

} // Namespace SOTG