#include <iostream>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <sstream>
#include "kalman_filter.cpp"

using namespace std;
using namespace Eigen;

vector<double> getUdata(){
    std::ifstream file("u.csv");
    std::vector<double> data;
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Error opening file" << std::endl;
        
    }

    while (getline(file, line)) {
        // Convert string to double and store in vector
        double value = std::stod(line);
        data.push_back(value);
    }

    file.close();

    // Output the data to verify
    // for (double num : data) {
    //     std::cout << num << std::endl;
    // }

    return data;
}

vector<double> getYdata(){
    std::ifstream file("y.csv");
    std::vector<double> data;
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Error opening file" << std::endl;
        
    }

    while (getline(file, line)) {
        // Convert string to double and store in vector
        double value = std::stod(line);
        data.push_back(value);
    }

    file.close();

    // Output the data to verify
    // for (double num : data) {
    //     std::cout << num << std::endl;
    // }

    return data;
}

int main(){
    MatrixXd A(2, 2); // System dynamics matrix
    MatrixXd B(2, 1); // Input control matrix
    MatrixXd C(1, 2); // Output matrix
    MatrixXd Q(2, 2); // Process -- R in matlab eg
    MatrixXd R(1, 1); // Measurement -- Q in matlab eg
    MatrixXd P(2, 2); // Estimate error covariance
    MatrixXd P0(2,2);

    VectorXd x0(2);



    A << 1,0.1096, 0,0.9962;
    B << 0, 0.0229;
    C << 1, 0;
    Q << 0.01,0, 0,0.01;
    R << 0.01;
    // cout<<"Here"<<endl;
    P0 << 0,0,0,0.00001;
    x0 << 1, 0.3;
    // cout<<"Here1"<<endl;
    Kalman_Filter kf(A,B,C,Q,R,P0,x0);
    

    vector<double> u = getUdata();
    vector<double> y = getYdata();

    // vector<VectorXd> u_;
    // vector<VectorXd> y_;

    for(int i=0; i<100; i++){
        // VectorXd u_;
        // VectorXd y_;
        // u_ << u[i];
        // y_ << y[i];
        // kf.combined(u[i], y[i]);
        kf.predict(u[i]);
        kf.update(y[i]);
        //Some issue in the Update Step
    }

    kf.print();

}