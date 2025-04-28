#include <iostream>
#include <vector>
#include "pose.hpp"
#include "quaternion.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <cmath>
#include <cassert>

int main()
{/* 
    coordinate::Pose pose1({0.1,0.1,0.1}, cv::Point3d(1,2,3));
    std::cout << "pose1: " << pose1.getTranslation().x<< " " << pose1.getTranslation().y<< " " << pose1.getTranslation().z<< " " << pose1.getRotation()[0] << " " << pose1.getRotation()[1] << " " << pose1.getRotation()[2] << std::endl;
    coordinate::Pose Gimbal2Camera({0,0,0}, cv::Point3d(2,0,0));
    std::cout << "Gimbal2Camera: " << Gimbal2Camera.getTranslation().x<< " " << Gimbal2Camera.getTranslation().y<< " " << Gimbal2Camera.getTranslation().z<< " " << Gimbal2Camera.getRotation()[0] << " " << Gimbal2Camera.getRotation()[1] << " " << Gimbal2Camera.getRotation()[2] << std::endl;
    coordinate::Pose Odom2Gimbal({-0.1,-0.1,-0.1}, cv::Point3d(0,0,0));
    std::cout << "Odom2Gimbal: " << Odom2Gimbal.getTranslation().x<< " " << Odom2Gimbal.getTranslation().y<< " " << Odom2Gimbal.getTranslation().z<< " " << Odom2Gimbal.getRotation()[0] << " " << Odom2Gimbal.getRotation()[1] << " " << Odom2Gimbal.getRotation()[2] << std::endl;

    coordinate::Pose Camera2Odom = pose1 * Odom2Gimbal.inverse() * Gimbal2Camera.inverse();

    // 获取结果并格式化输出
    std::vector<double> rot = Camera2Odom.getRotation();
    cv::Point3d trans = Camera2Odom.getTranslation();
    
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "旋转: " << rot[0] << " " << rot[1] << " " << rot[2] << "\n"
              << "平移: " << trans.x << " " << trans.y << " " << trans.z << std::endl;
     */
    using namespace coordinate;
    
    std::cout << std::setprecision(12);  // 提高输出精度
Quaternion q0 = Quaternion::fromEulerAngles(0,0,0);
std::cout << "单位四元数验证: " 
         << q0.get_w() << "," << q0.get_x() << "," << q0.get_y() << "," << q0.get_z() << std::endl;

    Quaternion q_test = Quaternion::fromEulerAngles(M_PI/2, 0, 0);
    cv::Point3d p_test = q_test * cv::Point3d(0,1,0);
    std::cout << "X旋转90度测试: " << p_test << std::endl;  // 应输出(0,0,1)


    Pose identity = Pose::Identity();
    cv::Point3d p1(1, 2, 3);
    cv::Point3d p1_trans = identity.transformPoint(p1);
    std::cout << "测试1 - 单位变换: " 
        << (cv::norm(p1 - p1_trans) < 1e-9 ? "通过" : "失败") << std::endl; 
    // 测试2: 纯平移变换逆运算
    Pose trans_pose({0,0,0}, cv::Point3d(2,3,4));
    Pose inv_trans = trans_pose.inverse();
    cv::Point3d p2 = inv_trans.transformPoint(cv::Point3d(2,3,4));
    std::cout << "测试2 - 平移逆运算: "
              << (cv::norm(p2) < 1e-6 ? "通过" : "失败") << std::endl;

    // 测试3: 纯旋转变换逆运算
    Pose rot_pose({M_PI/2,0,0}, cv::Point3d(0,0,0)); // X轴旋转90度
    Pose inv_rot = rot_pose.inverse();
    cv::Point3d p3(0, 1, 0);
    cv::Point3d p3_rot = inv_rot.transformPoint(rot_pose.transformPoint(p3));
    std::cout << "测试3 - 旋转逆运算: "
              << (cv::norm(p3 - p3_rot) < 1e-9 ? "通过" : "失败") << std::endl;

    // 测试4: 复合变换验证
    Pose T1({0.1,0.2,0.3}, cv::Point3d(1,2,3));
    Pose T2({-0.1,0.2,-0.3}, cv::Point3d(2,1,0));
    Pose T_total = T1 * T2;
    Pose T_inv = T_total.inverse();
    
    // 验证 T_total * T_inv ≈ Identity
    Pose should_be_identity = T_total * T_inv;
    cv::Point3d p4(5,5,5);
    cv::Point3d p4_trans = should_be_identity.transformPoint(p4);
    std::cout << "测试4 - 复合逆运算: "
              << (cv::norm(p4 - p4_trans) < 1e-6 ? "通过" : "失败") << std::endl;

    // 测试5: 链式变换验证
    Pose chain = T1 * T2 * T_inv;
    cv::Point3d p5(1,1,1);
    cv::Point3d p5_chain = chain.transformPoint(p5);
    cv::Point3d expected = T1.transformPoint(T2.transformPoint(T_inv.transformPoint(p5)));
    std::cout << "测试5 - 链式运算: "
              << (cv::norm(p5_chain - expected) < 1e-6 ? "通过" : "失败") << std::endl;
    
     return 0;
}