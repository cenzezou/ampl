

#include "arms.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ampl/kinematics.hpp>
#include <iostream>
#include <memory>

namespace ampl {

void ArmR7::initialize_urdf(const double *xyzrpyaxis,
                            const double *joint_limits,
                            const double *xyzrpy_tool0) {}
void ArmR7::initialize_preset(const std::string &name) {};
void ArmR7::set_joint_limits(const double *, const double *) {};
void ArmR7::fk(const double *q, double *qts_link) {};
void ArmR7::set_link_end_tool0(const double *xyzrpy) {};
void ArmR7::set_tcp(const double *tf44, bool colmajor) {};
void ArmR7::get_pose_tool0(double *tf44, bool colmajor) {};
} // namespace ampl