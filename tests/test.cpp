#include <iostream>

#include "sotg/sotg.hpp"

Eigen::Quaterniond eulerToQuat(double roll, double pitch, double yaw);
std::string vec_to_string(Eigen::Vector3d vec);
Eigen::Vector3d quatToEuler(Eigen::Quaterniond quat);

class TestLogger : public SOTG::Logger
{
public:
    void log(const std::string& message, [[maybe_unused]] MsgType type = INFO) const override final
    {
        std::cout << message << std::endl;
    }
};

int main()
{
    SOTG::SymbolGroupMap symbol_map;
    symbol_map["pos"] = {"x", "y", "z"};
    symbol_map["rot"] = {"w", "x", "y", "z"};
    symbol_map["rot"].setQuaternion(true);

    SOTG::Point p1(symbol_map, "p1");
    p1["pos"] = {0.0, 0.0, 0.0};
    p1["rot"] = {0.0, 0.0, 0.0, 1.0};

    p1["pos"].setConstraints(1.0, 1.0);
    p1["rot"].setConstraints(1.0, 1.0);

    SOTG::Point p2(symbol_map, "p2");
    p2["pos"] = {1.0, 0.0, 0.0};
    p2["rot"] = {1.0, 0.0, 0.0, 0.0};

    p2["pos"].setConstraints(1.0, 5.0);
    p2["rot"].setConstraints(1.0, 1.0);


    // SOTG::SymbolGroupMap symb_map2;
    // symb_map2["A1"] = {"a"};
    // symb_map2["A2"] = {"a"};
    // symb_map2["A3"] = {"a"};
    // symb_map2["A4"] = {"a"};
    // symb_map2["A5"] = {"a"};
    // symb_map2["A6"] = {"a"};

    // SOTG::Point p2(symb_map2, "p2");
    // p2["A3"] = {0.25};
    // p2["A5"] = {0.0};

    SOTG::Path my_path;
    my_path.addPoint(p1);
    my_path.addPoint(p2);
    my_path.addPoint(p1);


    TestLogger logger;
    SOTG::TrajectoryGenerator tg(symbol_map, logger);

    tg.resetPath(my_path);

    double duration = tg.getDuration();

    std::cout << "Duration: " << duration << "s" << std::endl;

    SOTG::Result result(symbol_map);

    for (size_t i = 0; i < 100; ++i)
    {
        double tick = i/100.0 * duration;
        tg.calcPositionAndVelocity(tick, result);
        Eigen::Vector3d euler_ang = quatToEuler(result.getLocation()["rot"].getQuaternionValues());
        Eigen::Vector3d euler_omega = result.getVelocity()["rot"].getQuaternionValues().vec();
        std::cout << " --- " << tick << "s --- " << std::endl;
        std::cout << result.getLocation() << std::endl;
        std::cout << result.getVelocity() << std::endl;
        std::cout << "Angle: "  << vec_to_string(euler_ang) << std::endl;
        std::cout << "Omega: " << vec_to_string(euler_omega) << std::endl;
        std::cout << " --- " << std::endl;

    }
}

std::string vec_to_string(Eigen::Vector3d vec) {
    const double PI = std::atan(1.0)*4;
    vec *= 180/PI;
    return "{" + std::to_string(vec.x()) + ", " + std::to_string(vec.y()) + ", " + std::to_string(vec.z()) + " }";
}

Eigen::Quaterniond eulerToQuat(double roll, double pitch, double yaw)
{
    using namespace Eigen;
    Quaterniond quat;
    quat = AngleAxisd(roll, Vector3d::UnitX()) * AngleAxisd(pitch, Vector3d::UnitY())
           * AngleAxisd(yaw, Vector3d::UnitZ());
    return quat;
}

Eigen::Vector3d quatToEuler(Eigen::Quaterniond quat) { return quat.toRotationMatrix().eulerAngles(0, 1, 2); }