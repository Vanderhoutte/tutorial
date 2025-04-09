#ifndef UKF_HPP
#define UKF_HPP

#include "Eigen/Dense.cpp"
#include <functional>

class UKF {
public:
    // 构造函数
    UKF(int n_states, int n_measurements, double alpha, double beta, double kappa)
        : n_states_(n_states), n_measurements_(n_measurements), alpha_(alpha), beta_(beta), kappa_(kappa) {
        lambda_ = alpha_ * alpha_ * (n_states_ + kappa_) - n_states_;
        sigma_points_.resize(2 * n_states_ + 1, n_states_);
        Wm_.resize(2 * n_states_ + 1);
        Wc_.resize(2 * n_states_ + 1);
    }

    // 初始化
    void initialize(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
        x_ = x0;
        P_ = P0;
        Q_ = Q;
        R_ = R;
        computeWeights();
    }

    // 预测步骤
    void predict(const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& f) {
        computeSigmaPoints();
        Eigen::MatrixXd sigma_points_pred(2 * n_states_ + 1, n_states_);
        for (int i = 0; i < 2 * n_states_ + 1; ++i) {
            sigma_points_pred.row(i) = f(sigma_points_.row(i));
        }
        x_.setZero();
        for (int i = 0; i < 2 * n_states_ + 1; ++i) {
            x_ += Wm_(i) * sigma_points_pred.row(i);
        }
        P_.setZero();
        for (int i = 0; i < 2 * n_states_ + 1; ++i) {
            Eigen::VectorXd diff = sigma_points_pred.row(i) - x_;
            P_ += Wc_(i) * diff * diff.transpose();
        }
        P_ += Q_;
    }

    // 更新步骤
    void update(const Eigen::VectorXd& z, const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& h) {
        computeSigmaPoints();
        Eigen::MatrixXd sigma_points_meas(2 * n_states_ + 1, n_measurements_);
        for (int i = 0; i < 2 * n_states_ + 1; ++i) {
            sigma_points_meas.row(i) = h(sigma_points_.row(i));
        }
        Eigen::VectorXd z_pred = Eigen::VectorXd::Zero(n_measurements_);
        for (int i = 0; i < 2 * n_states_ + 1; ++i) {
            z_pred += Wm_(i) * sigma_points_meas.row(i);
        }
        Eigen::MatrixXd P_zz = Eigen::MatrixXd::Zero(n_measurements_, n_measurements_);
        Eigen::MatrixXd P_xz = Eigen::MatrixXd::Zero(n_states_, n_measurements_);
        for (int i = 0; i < 2 * n_states_ + 1; ++i) {
            Eigen::VectorXd diff_z = sigma_points_meas.row(i) - z_pred;
            Eigen::VectorXd diff_x = sigma_points_.row(i) - x_;
            P_zz += Wc_(i) * diff_z * diff_z.transpose();
            P_xz += Wc_(i) * diff_x * diff_z.transpose();
        }
        P_zz += R_;
        Eigen::MatrixXd K = P_xz * P_zz.inverse();
        x_ += K * (z - z_pred);
        P_ -= K * P_zz * K.transpose();
    }

    // 获取当前状态估计
    Eigen::VectorXd getState() const {
        return x_;
    }

    // 获取当前状态协方差
    Eigen::MatrixXd getCovariance() const {
        return P_;
    }

private:
    int n_states_;
    int n_measurements_;
    double alpha_;
    double beta_;
    double kappa_;
    double lambda_;

    Eigen::MatrixXd x_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;

    Eigen::MatrixXd sigma_points_;
    Eigen::MatrixXd Wm_;
    Eigen::MatrixXd Wc_;

    // 计算sigma点
    void computeSigmaPoints() {
        Eigen::MatrixXd L = (n_states_ + lambda_) * P_.llt().matrixL();
        sigma_points_.row(0) = x_;
        for (int i = 0; i < n_states_; ++i) {
            sigma_points_.row(i + 1) = x_ + L.col(i);
            sigma_points_.row(n_states_ + i + 1) = x_ - L.col(i);
        }
    }

    // 计算权重
    void computeWeights() {
        Wm_.setConstant(0.5 / (n_states_ + lambda_));
        Wc_.setConstant(0.5 / (n_states_ + lambda_));
        Wm_(0) = lambda_ / (n_states_ + lambda_);
        Wc_(0) = lambda_ / (n_states_ + lambda_) + (1 - alpha_ * alpha_ + beta_);
    }
};

#endif    