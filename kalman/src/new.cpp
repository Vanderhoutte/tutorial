// 为目标函数编写单元测试，采用表驱动测试风格

#include <gtest/gtest.h>
#include "Eigen/Dense.cpp"
#include "kalman.hpp" // 假设目标函数定义在kalman.h中

// 表驱动测试数据结构
struct TestCase {
    int state_dim;  // 状态维度
    int mea_dim;    // 测量维度
    Eigen::VectorXd expected_x_hat; // 期望的x_hat
    Eigen::MatrixXd expected_P;     // 期望的P
};

// 测试目标函数
TEST(KalmanConstructorTest, TableDrivenTests) {
    // 定义测试用例
    std::vector<TestCase> test_cases = {
        {1, 1, Eigen::VectorXd::Zero(1), Eigen::MatrixXd::Identity(1, 1)},
        {2, 1, Eigen::VectorXd::Zero(2), Eigen::MatrixXd::Identity(2, 2)},
        {3, 2, Eigen::VectorXd::Zero(3), Eigen::MatrixXd::Identity(3, 3)},
        {4, 3, Eigen::VectorXd::Zero(4), Eigen::MatrixXd::Identity(4, 4)},
        {5, 5, Eigen::VectorXd::Zero(5), Eigen::MatrixXd::Identity(5, 5)},
        {10, 8, Eigen::VectorXd::Zero(10), Eigen::MatrixXd::Identity(10, 10)},
        {20, 15, Eigen::VectorXd::Zero(20), Eigen::MatrixXd::Identity(20, 20)},
        {50, 30, Eigen::VectorXd::Zero(50), Eigen::MatrixXd::Identity(50, 50)},
        {100, 80, Eigen::VectorXd::Zero(100), Eigen::MatrixXd::Identity(100, 100)},
    };

    // 遍历测试用例
    for (const auto& test_case : test_cases) {
        // 创建Kalman对象
        kalman kf(test_case.state_dim, test_case.mea_dim);

        // 验证x_hat是否正确
        EXPECT_EQ(kf.get_state(), test_case.expected_x_hat) 
            << "Failed for state_dim: " << test_case.state_dim 
            << ", mea_dim: " << test_case.mea_dim;

        // 验证P是否正确
        EXPECT_EQ(kf.get_p(), test_case.expected_P) 
            << "Failed for state_dim: " << test_case.state_dim 
            << ", mea_dim: " << test_case.mea_dim;
            
    }
}