#include <iostream>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

class Kalman_Filter{
    public:
    MatrixXd A;
    MatrixXd B;
    MatrixXd C;
    //MatrixXd D;
    MatrixXd Q;
    MatrixXd R;
    MatrixXd P;
    MatrixXd K;
    MatrixXd I;

    MatrixXd P0;
    VectorXd x0;
    int n;
    int m;
    int c;
    VectorXd x_hat;

    vector<VectorXd> x_hat_store;

    Kalman_Filter(MatrixXd A, MatrixXd B, MatrixXd C, MatrixXd Q, MatrixXd R, MatrixXd P0, VectorXd x0){
        this->A = A;
        this->B = B;
        this->C = C;
        //this->D = D;
        this->Q = Q;
        this->R = R;
        this->P = P0;
        this->P0 = P0;
        this->x0 = x0;
        this->n = A.rows();
        this->m = C.rows();
        this->c = B.cols();
        this->x_hat = x0;
        this->I.setIdentity(2,2);

        x_hat_store.push_back(x0);
    };

    void predict(const double& u);
    void update(const double& y);

    void combined(const double &u, const double& y);

    void print();

};