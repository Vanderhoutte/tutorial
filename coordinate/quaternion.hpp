#pragma once
#include <cmath>
#include <vector>
#include <iostream>
#include <utility>
#include <opencv4/opencv2/opencv.hpp>

namespace coordinate {
    class Quaternion {
        private:
            double w, x, y, z;//基本的四元数表示
        public:
            Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}//直接给定的构造函数
            Quaternion(const Quaternion& q) : w(q.w), x(q.x), y(q.y), z(q.z) {}//拷贝构造函数
            Quaternion(Quaternion&& q) noexcept //移动构造函数
            : w(std::exchange(q.w, 0.0)), 
              x(std::exchange(q.x, 0.0)),
              y(std::exchange(q.y, 0.0)),
              z(std::exchange(q.z, 0.0)) {}
            /* Quaternion(double x, double y, double z, double angle) {//给定角度的构造函数
                double norm = sqrt(x * x + y * y + z * z);
                if(norm == 0) {
                    w = 1.0;
                    this->x = this->y = this->z = 0.0;
                    return;
                }
                double sin_half_angle = sin(angle / 2);
                w = cos(angle / 2);
                x = sin_half_angle * x / norm;
                y = sin_half_angle * y / norm;
                z = sin_half_angle * z / norm;
            } */
            Quaternion(const std::vector<double>& v) { //通过向量构造四元数
                if (v.size() == 3) {//如果是三维向量，构造一个纯虚四元数
                    w = 0;
                    x = v[0];
                    y = v[1];
                    z = v[2];
                }
                else if (v.size() == 4) { 
                    w = v[0];
                    x = v[1];
                    y = v[2];
                    z = v[3];
                }
                else {
                    w = 0;
                    x = 0;
                    y = 0;
                    z = 0;
                }
            }

            Quaternion() = default;//默认构造函数



//^构造函数
//v运算符重载


            // 拷贝赋值运算符
            Quaternion& operator=(const Quaternion& other) {
                if (this != &other) {
                    w = other.w;
                    x = other.x;
                    y = other.y;
                    z = other.z;
                }
            return *this;
            }

            // 移动赋值运算符
            Quaternion& operator=(Quaternion&& other) noexcept {
                if (this != &other) {
                    w = std::exchange(other.w, 0.0);
                    x = std::exchange(other.x, 0.0);
                    y = std::exchange(other.y, 0.0);
                    z = std::exchange(other.z, 0.0);
                }
                return *this;
            }
            ~Quaternion() = default;//默认析构函数
            double norm() const {//四元数的模
                return sqrt(w * w + x * x + y * y + z * z);
            }
            Quaternion normalized() const {//四元数归一化
                double norm_value = norm();
                if (norm_value < 1e-8) {
                    return Quaternion(1, 0, 0, 0); // 返回默认单位四元数
                }
                return Quaternion(w / norm_value, x / norm_value, y / norm_value, z / norm_value);
            }
            Quaternion conjugate() const {//共轭四元数
                return Quaternion(w, -x, -y, -z);
            }

            Quaternion operator+(const Quaternion& q) const {//四元数加法
                return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
            }
            Quaternion operator-(const Quaternion& q) const {//四元数减法
                return Quaternion(w - q.w, x - q.x, y - q.y, z - q.z);
            }
            Quaternion operator*(const Quaternion& q) const {//四元数乘法
                return Quaternion(
                    w * q.w - x * q.x - y * q.y - z * q.z,
                    w * q.x + x * q.w + y * q.z - z * q.y,
                    w * q.y - x * q.z + y * q.w + z * q.x,
                    w * q.z + x * q.y - y * q.x + z * q.w
                );
            }
            Quaternion operator*(double scalar) const {//四元数与标量乘法
                return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
            }

            friend Quaternion operator*(double scalar, const Quaternion& q) {
                return q * scalar;
            }

            cv::Point3d operator*(const cv::Point3d& p) const {//四元数与三维向量乘法
                if(this->norm() < 1e-8) {
                    throw std::runtime_error("Quaternion norm is too small");
                }
                Quaternion vq(0, p.x, p.y, p.z);
                Quaternion result = (*this * vq * this->conjugate());
                return cv::Point3d( result.x, result.y, result.z);
            }

            Quaternion operator/(double scalar) const {//四元数与标量除法
                if (scalar == 0) {
                    throw std::runtime_error("Division by zero");
                }
                return Quaternion(w / scalar, x / scalar, y / scalar, z / scalar);
            }
            Quaternion operator-() const {//四元数取负
                return Quaternion(-w, -x, -y, -z);
            }


//^重载运算符
//v高级运算与特定功能实现
            double get_x() const {
                return x;
            }
            double get_y() const {
                return y;
            }
            double get_z() const {
                return z;
            }
            double get_w() const {
                return w;
            }
            
            //^获取四元数的各个分量

            static Quaternion fromEulerAngles(double roll, double pitch, double yaw) {//欧拉角转四元数
                // 计算每个轴的半角
                double cy = cos(yaw * 0.5);
                double sy = sin(yaw * 0.5);
                double cp = cos(pitch * 0.5);
                double sp = sin(pitch * 0.5);
                double cr = cos(roll * 0.5);
                double sr = sin(roll * 0.5);
        
                // 四元数乘法顺序：Z → Y → X 旋转
                return Quaternion(
                    cy * cp * cr + sy * sp * sr,
                    cy * cp * sr - sy * sp * cr,
                    sy * cp * sr + cy * sp * cr,
                    sy * cp * cr - cy * sp * sr
                ).normalized();
            }
            std::vector<double> toEulerAngles() const {//四元数转欧拉角
                double sinr_cosp = 2 * (w * x + y * z);
                double cosr_cosp = 1 - 2 * (x * x + y * y);
                double roll = std::atan2(sinr_cosp, cosr_cosp);

                double sinp = 2 * (w * y - z * x);
                double pitch = std::asin(sinp);
                double siny_cosp = 2 * (w * z + x * y);
                double cosy_cosp = 1 - 2 * (y * y + z * z);
                double yaw = std::atan2(siny_cosp, cosy_cosp);
                return {roll, pitch, yaw};
            }
            std::vector<double> toVector() const {//四元数转向量
                return {w, x, y, z};
            }
            Quaternion operator*(const std::vector<double>& v) const {
                if (v.size() != 3) {
                    throw std::runtime_error("Invalid vector size");
                }
                Quaternion vq(0, v[0], v[1], v[2]); // 构造一个纯虚四元数
                return (*this * vq * this->conjugate()).normalized(); 
            }

            Quaternion inverse() const {//四元数求逆
                return conjugate() / (w*w + x*x + y*y + z*z);
            }
            Quaternion slerp(const Quaternion& q, double t) const {//四元数球面插值
                double cos_omega = w * q.w + x * q.x + y * q.y + z * q.z;
                double omega = acos(cos_omega);
                double sin_omega = sin(omega);
                double scale0 = sin((1 - t) * omega) / sin_omega;
                double scale1 = sin(t * omega) / sin_omega;
                return Quaternion(
                    scale0 * w + scale1 * q.w,
                    scale0 * x + scale1 * q.x,
                    scale0 * y + scale1 * q.y,
                    scale0 * z + scale1 * q.z
                )
                .normalized();
            }



            cv::Mat toRotationMatrix() const {//四元数转旋转矩阵（opencv）
                cv::Mat mat = cv::Mat::eye(3, 3, CV_64F);
                double qw = w, qx = x, qy = y, qz = z;
                
                // 计算旋转矩阵元素
                mat.at<double>(0,0) = 1 - 2*qy*qy - 2*qz*qz;
                mat.at<double>(0,1) = 2*qx*qy - 2*qz*qw;
                mat.at<double>(0,2) = 2*qx*qz + 2*qy*qw;
                
                mat.at<double>(1,0) = 2*qx*qy + 2*qz*qw;
                mat.at<double>(1,1) = 1 - 2*qx*qx - 2*qz*qz;
                mat.at<double>(1,2) = 2*qy*qz - 2*qx*qw;
                
                mat.at<double>(2,0) = 2*qx*qz - 2*qy*qw;
                mat.at<double>(2,1) = 2*qy*qz + 2*qx*qw;
                mat.at<double>(2,2) = 1 - 2*qx*qx - 2*qy*qy;
            
                return mat;
            }

            static Quaternion fromRotationMatrix(const cv::Mat& mat) {//旋转矩阵转四元数（opencv）
                if(mat.rows != 3 || mat.cols != 3) {
                    throw std::runtime_error("Invalid rotation matrix size");
                }
                
                double m00 = mat.at<double>(0,0);
                double m11 = mat.at<double>(1,1);
                double m22 = mat.at<double>(2,2);
                double trace = m00 + m11 + m22;
                
                if(trace > 0) {
                    double s = 0.5 / sqrt(trace + 1.0);
                    return Quaternion(0.25 / s,
                                      (mat.at<double>(2,1) - mat.at<double>(1,2)) * s,
                                      (mat.at<double>(0,2) - mat.at<double>(2,0)) * s,
                                      (mat.at<double>(1,0) - mat.at<double>(0,1)) * s);
                }
                else {
                    // 处理不同轴的最大值情况
                    int i = (m00 > m11) ? ((m00 > m22) ? 0 : 2) : ((m11 > m22) ? 1 : 2);
                    int j = (i + 1) % 3;
                    int k = (i + 2) % 3;
                    
                    double s = sqrt(mat.at<double>(i,i) - mat.at<double>(j,j) - mat.at<double>(k,k) + 1.0);
                    double q[4];
                    q[i+1] = 0.5 * s;
                    q[0] = (mat.at<double>(k,j) - mat.at<double>(j,k)) / (2 * s);
                    q[j+1] = (mat.at<double>(j,i) + mat.at<double>(i,j)) / (2 * s);
                    q[k+1] = (mat.at<double>(k,i) + mat.at<double>(i,k)) / (2 * s);
                    
                    return Quaternion(q[0], q[1], q[2], q[3]).normalized();
                }
            }

            static Quaternion fromAxisAngle(const cv::Point3d& axis, double angle) {//轴角转四元数
                double norm = sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
                if(norm == 0) {
                    return Quaternion(1, 0, 0, 0); // 返回单位四元数
                }
                double sin_half_angle = sin(angle / 2);
                return Quaternion(cos(angle / 2), 
                                  sin_half_angle * axis.x / norm,
                                  sin_half_angle * axis.y / norm,
                                  sin_half_angle * axis.z / norm);
            }
            cv::Point3d toAxisAngle() const {//四元数转轴角
                double angle = 2 * acos(w);
                double s = sqrt(1 - w * w);
                if (s < 1e-8) {
                    return cv::Point3d(1, 0, 0); // 返回单位向量
                }
                return cv::Point3d(x / s, y / s, z / s) * angle;
            }
    };
};