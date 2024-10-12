
#include "sotg/path.hpp"

#include <iostream>

using namespace SOTG;

void Path::addPoint(Point point) {
    int point_id = waypoints_.size() + 1;
    point.setID(point_id);
    waypoints_.push_back(point); }

Point& Path::getPoint(std::string name)
{
    for (auto& point : waypoints_) {
        if (point.getName().compare(name) == 0) {
            return point;
        }
    }
    throw std::runtime_error("No Point named \"" + name + "\" was found");
}


Point SOTG::Path::getPointValue(size_t index) const
{
    return waypoints_[index];
}

Path SOTG::Path::operator+(const Path& path)
{
    for(size_t i = 0; i < path.size(); i++)
    {
        addPoint(path.getPointValue(i));
    }

    return *this;
}

std::ostream& Path::operator<<(std::ostream& out)
{
    out << "[ ";

    if (!waypoints_.empty()) {
        for (size_t i = 0; i < waypoints_.size() - 1; i++) {
            out << waypoints_[i] << ", ";
        }
        out << waypoints_.back() << " ]";
    } else {
        out << " ]";
    }

    return out;
}