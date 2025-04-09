#include <iostream>
#include <fstream>
#include "Eigen/Dense.cpp"
#include "kalman.hpp"
using namespace std;

int main()
{
    fstream file_in;
    fstream file_out;
    Eigen::MatrixXd A(2,2);
    A << 1,0.001,0,1;
    Eigen::MatrixXd B(1,2);
    B << 0,0;
    Eigen::MatrixXd u(1,2);
    u << 0,0;
    Eigen::MatrixXd Q(2,2);
    Q << 0.025,0,0,0.025;
    Eigen::MatrixXd H(1,2);
    H << 1,0;
    Eigen::MatrixXd R(1,1);
    R << 90;
    kalman k(2,1);
    Eigen::MatrixXd z(1,1);
    z << 0;
    k.elementChange_P(3);
    cout << "algo start" << endl;
    for(int i = 1;i < 5;i++)
    {
        cout << "file:" << i << " start" << endl;
        string file_name = "../in/homework_data_" + to_string(i) + ".txt";
        file_in.open(file_name,ios::in);
        file_out.open("../out/homework_data_" + to_string(i) + ".txt",ios::out);
        if(!file_in.is_open())
        {
            cout << "file open failed" << endl;
            return 0;
        }
        double x,y;
        file_in >> x >> y;
        Eigen::MatrixXd fst(2,1);
        fst << x,y;
        k.elementChange_X(fst);
        while( file_in >> x >> y)
        {
            z << y;
            cout << "z:" << z << endl;
            k.predict(A,B,u,Q,false);
            k.update(H,z,R);
            file_out << x << " " << k.get_state()(0)<< endl;
        }
        file_in.close();
        file_out.close();
    }
    return 0;
}