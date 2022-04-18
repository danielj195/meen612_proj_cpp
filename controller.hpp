#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <vector>
#include <array>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <cmath>

namespace controller
{

class Controller
{
public:
    Controller(std::vector<std::array<double,8>> poly_1, 
        std::vector<std::array<double,8>> poly_2, std::vector<double> time_vec);
    ~Controller();

protected:
    void find_poly_coeff(double t);
    void find_des_state(double t);
    void compute_error();


    Eigen::MatrixXd q_des_;
    Eigen::MatrixXd q_d_des_;
    Eigen::MatrixXd q_dd_des_;
    Eigen::MatrixXd err_;
    Eigen::MatrixXd d_err_;
    Eigen::MatrixXd q_;
    Eigen::MatrixXd dq_;
    std::vector<double> time_vec_;

    std::array<double,8> coeff_1_;
    std::array<double,8> coeff_2_;
    std::vector<std::array<double,8>> poly_1_;
    std::vector<std::array<double,8>> poly_2_;

};


} // namespace controller

#endif
