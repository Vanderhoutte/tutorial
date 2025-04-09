#include <iostream>
#include <fstream>
#include "../fast-cpp-csv-parser-master/csv/csv.h"
#include "Eigen/Dense.cpp"
#include "kalman.hpp"
using namespace std;

int main()
{
    fstream file_in;
    fstream file_out;
    Eigen::MatrixXd A(2,2);
    A << 1,1,0,1;
    Eigen::MatrixXd B(1,1);
    B << 0;
    Eigen::MatrixXd u(1,1);
    u << 0;
    Eigen::MatrixXd Q(2,2);
    Q << 0.03,0,0,0.03;
    Eigen::MatrixXd H(1,2);
    H << 1,0;
    Eigen::MatrixXd R(1,1);
    R << 100;
    kalman kal(2,1);
    kal.elementChange_P(3);
    io::CSVReader<2> in("../in/stock_prices.csv");
    in.read_header(io::ignore_extra_column, "DATE", "PRICE");
    file_out.open("../out/result.txt",ios::out);
    float date,price;
    Eigen::MatrixXd fst(2,1);
    in.read_row(date,price);
    fst << price,0;
    kal.elementChange_X(fst);
    while(in.read_row(date,price))
    {
        Eigen::MatrixXd z(1,1);
        z << price;
        kal.predict(A,B,u,Q,false);
        file_out << date << " " << kal.get_state()(0)<< endl;
        kal.update(H,z,R);
    }
    return 0;
}