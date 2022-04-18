#include "controller.hpp"

namespace controller
{

Controller::Controller(std::vector<std::array<double,8>> poly_1, 
        std::vector<std::array<double,8>> poly_2, std::vector<double> time_vec){
    poly_1_ = poly_1;
    poly_2_ = poly_2;
    time_vec_ = time_vec;

    q_des_(2,1);
    q_d_des_(2,1);
    q_dd_des_(2,1);
    err_(2,1);
    d_err_(2,1);
    q_(2,1);
    dq_(2,1);
}

Controller::~Controller(){};

void Controller::find_poly_coeff(double t){
    int ind = 0;
    for(size_t i = 0; i<time_vec_.size(); i++){
        if(t>time_vec_[i]){
            break;
        }
        ind++;
    }
    coeff_1_ = poly_1_[ind];
    coeff_2_ = poly_2_[ind];
}

void Controller::find_des_state(double t){
    q_des_(0,0) = coeff_1_[0] + coeff_1_[1]*t + coeff_1_[2]*pow(t,2) + coeff_1_[3]*pow(t,3) + 
            coeff_1_[4]*pow(t,4) + coeff_1_[5]*pow(t,5) + coeff_1_[6]*pow(t,6) + coeff_1_[7]*pow(t,7);

    // TODO:  Continue for q_d_des_ and q_dd_des_
}

void Controller::compute_error(){
    err_ = q_ - q_des_;
    d_err_ = dq_ - q_d_des_;
}


} // namespace controller
