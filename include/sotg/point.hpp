
#pragma once

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include <eigen3/Eigen/Core>

#include "sotg/symbol_group.hpp"
#include "sotg/utility_functions.hpp"
#include "sotg/value_group.hpp"

namespace SOTG {

// Wraps std::vector and allows vector arithmetic, stores information about the
// unit of a particular value
class Point {
private:
    ValueGroupMap value_map_;
    SymbolGroupMap symbol_map_;
    std::string name_ = "ND";
    int id_ = -1;

public:
    Point(SymbolGroupMap symbol_map, std::string name = "ND") : symbol_map_(symbol_map), name_(name)
    {
        for (auto& [key, symbols] : symbol_map) {
            value_map_.insert({key, ValueGroup(symbols)});
        }
    }
    Point() = delete;

    std::string getName() { return name_; }
    ValueGroupMap& getValueMap() { return value_map_; }
    const SymbolGroupMap& getSymbolMap() const { return symbol_map_; }

    ValueGroup& operator[](std::string key) { return value_map_.at(key); }
    Point operator-(Point& p) {
        Point diff(symbol_map_);
        for (auto [group_name, symbols] : symbol_map_) {
            if (symbols.isQuaternion()) {
                diff[group_name] = p[group_name].getQuaternionValues().inverse() * (*this)[group_name].getQuaternionValues();
            } else {
                diff[group_name] = Eigen::VectorXd((*this)[group_name].getCartesianValues() - p[group_name].getCartesianValues());
            }
        }
        return diff;
    }

    int getID() const { return id_; }
    void setID(int id) { id_ = id; }

    friend std::ostream& operator<<(std::ostream& out, const Point& point)
    {
        out << "[[ " << point.name_ << " ] {";
        for (auto& [key, value] : point.value_map_) {
            out << " " << key << " : " << value << ",";
        }
        out << "\b";
        out << " }]";

        return out;
    }
};

}  // namespace SOTG