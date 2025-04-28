#include <iostream>
#include <iomanip>
#include <cmath>
#include <opencv2/core.hpp>
#include "pose.hpp"
#include "quaternion.hpp"

using namespace coordinate;

// 辅助：把欧拉角（弧度）和位移输出
void printPose(const std::string& name, const Pose& p) {
    auto e = p.getRotation();      // {roll, pitch, yaw} in radians
    auto t = p.getTranslation();   // {x, y, z}

    std::cout << std::fixed << std::setprecision(5);
    std::cout << name
              << "  Euler (rad): ["
              << std::setw(8) << e[0] << ", "
              << std::setw(8) << e[1] << ", "
              << std::setw(8) << e[2] << "]"
              << "   Trans: ("
              << std::setw(8) << t.x << ", "
              << std::setw(8) << t.y << ", "
              << std::setw(8) << t.z << ")\n";
}

int main() {
    using namespace coordinate;

    Pose pose_C({0.1, 0.1, 0.1}, cv::Point3d(1, 2, 3));
    Pose G2C({0.0,0.0,0.0},cv::Point3d(2,0,0));
    Pose O2G({-0.1,-0.1,-0.1},cv::Point3d(0,0,0));

    std::cout << "G2C = " << G2C.getTranslation().x << " " 
                << G2C.getTranslation().y << " "
                << G2C.getTranslation().z << "\n"
                << G2C.getRotation()[0] << " "
                << G2C.getRotation()[1] << " "
                << G2C.getRotation()[2] << "\n";

    Pose pose_G = G2C * pose_C;

    std::cout << "pose_G = G2C * pose_C\n";
    std::cout << pose_G.getTranslation().x << " "
              << pose_G.getTranslation().y << " "
              << pose_G.getTranslation().z << "\n"
              << pose_G.getRotation()[0] << " "
              << pose_G.getRotation()[1] << " "
              << pose_G.getRotation()[2] << "\n";

    std::cout << "O2G = " << O2G.getTranslation().x << " "
                << O2G.getTranslation().y << " "
                << O2G.getTranslation().z << "\n"
                << O2G.getRotation()[0] << " "
                << O2G.getRotation()[1] << " "
                << O2G.getRotation()[2] << "\n";

    Pose pose_O = pose_G * O2G;
    std::cout << "pose_O = pose_G * O2G\n";
    std::cout << pose_O.getTranslation().x << " "
              << pose_O.getTranslation().y << " "
              << pose_O.getTranslation().z << "\n"
              << pose_O.getRotation()[0] << " "
              << pose_O.getRotation()[1] << " "
              << pose_O.getRotation()[2] << "\n";

    return 0;
}
