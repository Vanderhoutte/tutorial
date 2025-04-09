#include <iostream>
#include <fstream>
#include "Eigen/Dense.cpp"
#include "kalman.hpp"

using namespace std;

int main()
{
    Eigen::MatrixXd A(2,2);
    Eigen::MatrixXd B(2,2);
    kalman kal(4,4);
    Eigen::MatrixXd Q();
    
}
