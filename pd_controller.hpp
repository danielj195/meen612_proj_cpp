#ifndef PD_CONTROLLER_HPP_
#define PD_CONTROLLER_HPP_

#include <vector>
#include <array>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <cmath>

#include "controller.hpp"

namespace pd_controller
{

class PD_Controller : public controller::Controller
{
public:
    PD_Controller(std::vector<std::array<double,8>> poly_1, 
        std::vector<std::array<double,8>> poly_2, std::vector<double> time_vec);
    ~PD_Controller();

    Eigen::Array4d pd_control_law(double t, Eigen::Array4d y);

private:
    double wn_;
    double damp_;
    Eigen::MatrixXd Kp_;
    Eigen::MatrixXd Kd_;
    Eigen::MatrixXd q_dot_;
    Eigen::Array4d dy_;

};


} // namespace pd_controller

#endif
