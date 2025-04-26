#include <iostream>
#include <vector>
#include "pose.hpp"
#include "quaternion.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <cmath>

int main()
{
    coordinate::Pose pose1({1,2,3}, cv::Point3d(0.1,0.1,0.1));
    coordinate::Pose Gimminal2Camera({2,0,0},cv::Point3d(0,0,0));
    coordinate::Pose Odom2Gimminal({0,0,0},cv::Point3d(-0.1,-0.1,-0.1));
    coordinate::Pose pose2 = pose1 * Gimminal2Camera * Odom2Gimminal;
    std::vector<double> result_q = pose2.getRotation();
    cv::Point3d result_t = pose2.getTranslation();
    std::cout << "result_q: " << result_q[0] << " " << result_q[1] << " " << result_q[2] << std::endl;
    std::cout << "result_t: " << result_t.x << " " << result_t.y << " " << result_t.z << std::endl;
    return 0;
}