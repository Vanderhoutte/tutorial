#include"Eigen/Dense.cpp"
#include<iostream>
using namespace std;
int main()
{
    Eigen::MatrixXd A(1,2);
    A << 1,0;
    Eigen::MatrixXd B(2,1);
    B << 1,0;
    cout << A << endl;
    cout << B << endl;
    cout << A(0) << A(1) << endl;
    return 0;
}