#ifndef KALMAN_H
#define KALMAN_H

#include <cmath>
#include <iostream>
#include "Eigen/Dense.cpp"
using namespace std;

class kalman {
    private:
        int state_dim,mea_dim;
        Eigen::MatrixXd x_hat;
        Eigen::MatrixXd P;
    public:
        kalman(int state_dim,int mea_dim):state_dim(state_dim),mea_dim(mea_dim)
        {
            x_hat = Eigen::MatrixXd::Zero(state_dim,1);
            cout << "x_hat:" << x_hat << endl;
            P = Eigen::MatrixXd::Identity(state_dim,state_dim);
            cout << "P:" << P << endl;
            cout<<"kalman construct"<<std::endl;
        }

        Eigen::MatrixXd get_p() const
        {
            return P;
        }

        void predict(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& u, const Eigen::MatrixXd& Q,bool B_in = true)
        {
            cout << "predict" << endl;
            cout << "A:" << A << endl;
            cout << "B:" << B << endl;
            cout << "u:" << u << endl;
            if(B_in)
            {
                cout << "B_in" << endl;
                x_hat = A * x_hat + B * u;
                cout << x_hat << endl;
                P = A * P * A.transpose() + Q;
                cout << P << endl;
            }
            else
            {
                cout << "B_out" << endl;
                x_hat = A * x_hat;
                cout << x_hat << endl;
                P = A * P * A.transpose() + Q;
                cout << P << endl;
            }
        }

        void update(const Eigen::MatrixXd& H, const Eigen::MatrixXd& z, const Eigen::MatrixXd& R)
        {
            cout << "update" << endl;
            cout << "H:" <<H << endl;
            cout << "z:" << z << endl;
            cout << "R:" << R << endl;
            cout << "P:" << P << endl;
            Eigen::MatrixXd S = (H * P * H.transpose() + R);
            cout << "S:" << S << endl;
            Eigen::MatrixXd K = P * H.transpose() * S.inverse();
            cout << "K:" << K << endl;
            x_hat = x_hat + K * (z - H * x_hat);
            cout << "x_hat:" << x_hat << endl;
            P= (Eigen::MatrixXd::Identity(state_dim, state_dim) - K * H) * P;
            cout << "P:" << P << endl;
        }

        void elementChange_P(int n)
        {
            P *= std::pow(10,n);
            cout << P << std::endl;
        }

        void elementChange_X(Eigen::MatrixXd X_new)
        {
            x_hat = X_new;
        }

        Eigen::MatrixXd get_state() const
        {
            return x_hat;
        }
};

#endif 