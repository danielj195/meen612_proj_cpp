#include "pd_controller.hpp"

namespace pd_controller
{


PD_Controller::PD_Controller(std::vector<std::array<double,8>> poly_1, 
    std::vector<std::array<double,8>> poly_2, std::vector<double> time_vec) : controller::Controller(poly_1, poly_2, time_vec){
    poly_1_ = poly_1;
    poly_2_ = poly_2;
    time_vec_ = time_vec;

    wn_ = 200;
    damp_ = 0.98;
    Kp_ = pow(wn_,2) * Eigen::MatrixXd::Identity(2,2);
    Kd_ = 2 * wn_ * damp_ * Eigen::MatrixXd::Identity(2,2);
    q_dot_(2,1);
}

PD_Controller::~PD_Controller(){}

Eigen::Array4d PD_Controller::pd_control_law(double t, Eigen::Array4d y){
    q_(0,0) = y[0];
    q_(1,0) = y[1];
    dq_(0,0) = y[2];
    dq_(1,0) = y[3];

    find_poly_coeff(t);
    find_des_state(t);
    compute_error();

    q_dot_ = q_dd_des_ - Kd_ * d_err_ - Kp_ * err_;

    dy_[0] = dq_(0,0);
    dy_[1] = dq_(1,0);
    dy_[2] = q_dot_(0,0);
    dy_[3] = q_dot_(1,0);

    return dy_;
}

} // namespace pd_controller
