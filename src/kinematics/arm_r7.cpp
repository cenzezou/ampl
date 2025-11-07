

#include "arms.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ampl/kinematics.hpp>
#include <iostream>
#include <memory>

namespace ampl {

void ArmR7::initialize_urdf(const double *xyzrpyaxis,
                            const double *joint_limits) {}
void ArmR7::initialize_preset(const std::string &name) {};
void ArmR7::set_joint_limits(const double *joint_limits) {};
void ArmR7::fk(const double *q, double *qts_link) {};

} // namespace ampl