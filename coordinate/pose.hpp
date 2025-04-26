#pragma once
#include <vector>
#include <string>
#include <iostream>
#include <cmath>
#include <opencv4/opencv2/opencv.hpp>
#include "quaternion.hpp"

namespace coordinate {
    class Pose {
        private:
            std::vector<double> q;//欧拉角表示
            cv::Point3d t;//平移向量
        public:
            static Pose Identity() {
                return Pose({0,0,0}, cv::Point3d(0,0,0));
            }
    
            Pose() : q{0,0,0}, t{0,0,0} {}//默认构造函数
            Pose(const double q[3], const cv::Point3d& t) : t(t) {//直接给定的构造函数
                for(int i = 0; i < 3; i++) {
                    this->q[i] = q[i];
                }
            }
            Pose(const Pose& p) : t(p.t) {//拷贝构造函数
                for(int i = 0; i < 3; i++) {
                    q[i] = p.q[i];
                }
            }
            Pose(Pose&& p) noexcept {//移动构造函数
                for(int i = 0; i < 3; i++) {
                    q[i] = std::exchange(p.q[i], 0.0);
                }
                t = std::move(p.t);
            }
            Pose(const cv::Mat &t_vec ,const cv::Mat &r_vec) {//给定旋转矩阵和平移向量的构造函数
                if(r_vec.rows != 3 || r_vec.cols != 3) { 
                    throw std::runtime_error("Rotation matrix must be 3x3");
                }
                if(r_vec.type() != CV_64F || t_vec.type() != CV_64F) {
                    throw std::runtime_error("Matrix must be double type");
                }
                Quaternion q_rot = Quaternion::fromRotationMatrix(r_vec);
                std::vector<double> angles = q_rot.toEulerAngles();
                for(int i = 0; i < 3; i++) {
                    q[i] = angles[i];
                }
                t = cv::Point3d(t_vec.at<double>(0), t_vec.at<double>(1), t_vec.at<double>(2));
            }

            Pose(const std::vector<double>& q, const cv::Point3d& t) : t(t) {//给定欧拉角和平移向量的构造函数
                if(q.size()!= 3) {
                    throw std::runtime_error("Invalid quaternion size");
                }
                for(int i = 0; i < 3; i++) {
                    this->q[i] = q[i];
                }
            }

            ~Pose() = default;//默认析构函数

            Pose operator*(const Pose& p) const {//位姿乘法
                    Quaternion q1 = Quaternion::fromEulerAngles(q[0], 
                        q[1],
                        q[2]).normalized();
                    Quaternion q2 = Quaternion::fromEulerAngles(p.q[0],
                        p.q[1],
                        p.q[2]).normalized();
                    // 直接保留四元数形式避免重复转换
                    Quaternion combined_rot = (q1 * q2).normalized();
                    cv::Point3d new_t = q1 * p.t + t;
                    return Pose(combined_rot.toEulerAngles(), new_t);
            }

            Pose inverse() const {//位姿求逆
                Quaternion q_rot = Quaternion::fromEulerAngles(q[0], q[1], q[2]).normalized();
                Quaternion q_rot_inv = q_rot.inverse();
                std::vector<double> angles = q_rot_inv.toEulerAngles(); 
                cv::Point3d t_rot = -q_rot_inv * t;
                return Pose(angles, t_rot);
            }
            
            std::vector<double> getRotation() const {//获取欧拉角
                return q;
            }
            cv::Point3d getTranslation() const {//获取平移向量
                return t;
            }
    };
};
