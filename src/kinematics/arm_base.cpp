

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ampl/kinematics.hpp>
#include <iostream>
#include <memory>

#include "arms.hpp"

namespace ampl {

std::unique_ptr<ArmBase> ArmBase::create(const std::string &name,
                                         const ArmType &arm_type,
                                         const uint32_t dof,
                                         const float *urdf) {

  if (arm_type == ArmType::Industrial6)
    return std::make_unique<ArmR6>(name);

  if (arm_type == ArmType::Humanoid7)
    return std::make_unique<ArmR7>(name);

  return std::make_unique<ArmNull>();
};

} // namespace ampl