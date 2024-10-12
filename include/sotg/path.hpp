#pragma once

#include <string>
#include <vector>

#include <eigen3/Eigen/Core>

#include "sotg/point.hpp"

namespace SOTG {

// A collection of points used as an input to the trajectory generator to
// quickly setup many new waypoints
class Path {
private:
    std::vector<Point> waypoints_;

public:
    void addPoint(Point point);

    size_t getNumWaypoints() { return waypoints_.size(); };
    Point& getPoint(std::string name);
    Point& getPoint(size_t index) { return waypoints_[index]; }

    std::vector<Point>::iterator begin() { return waypoints_.begin(); }
    std::vector<Point>::iterator end() { return waypoints_.end(); }

    std::ostream& operator<<(std::ostream& out);
};

}  // namespace SOTG