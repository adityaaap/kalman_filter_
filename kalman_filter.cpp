#include <iostream>
#include <eigen3/Eigen/Dense>
#include "kalman_filter.hpp"

using namespace std;
using namespace Eigen;



void Kalman_Filter::predict(const double& u_){
    VectorXd u(1);
    u(0) = u_;
    x_hat = A*x_hat + B*u;
    P = A*P*A.transpose() + Q;
    cout<<"End Predict"<<endl;
}   

void Kalman_Filter::update(const double& y_){
    VectorXd y(1);
    y(0) = y_;
    cout<<"Measurement Received"<<endl;
    K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
    cout<<"Gain found"<<endl;
	x_hat += K * (y - C*x_hat);
    cout<<"Estimate found"<<endl;
	P = (I - K*C)*P;
    cout<<"Cov found"<<endl;
    x_hat_store.push_back(x_hat);
    cout<<"Estimate Stored"<<endl;
    cout<<"End update"<<endl;
}

void Kalman_Filter::combined(const double&u_, const double& y_){
    cout<<"Here"<<endl;
    VectorXd u(1);
    VectorXd y(1);
    u(0) = u_;
    y(0) = y_;
    cout<<"Before KF Alo starts"<<endl;
    K = P*C.transpose() * (C*P*C.transpose() + R).inverse();
    x_hat = A*x_hat + B*u + A*K*(y - x_hat);
    P = A * (P - K*C*P) * A.transpose() + Q;
    x_hat_store.push_back(x_hat);
}

void Kalman_Filter::print(){
    for (size_t i = 0; i < x_hat_store.size(); ++i) {
        cout << "VectorXd at index " << i << ":\n" << x_hat_store[i] << endl;
        // Or, if you prefer to print it in a line-by-line fashion:
        // for (int j = 0; j < x_hat_store[i].size(); ++j) {
        //     cout << x_hat_store[i](j) << " ";
        // }
        // cout << endl; // Provide a newline character after each VectorXd
    }
}

