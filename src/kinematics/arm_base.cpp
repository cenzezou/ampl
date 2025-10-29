

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ampl/kinematics.hpp>
#include <iostream>
#include <memory>

namespace ampl {

class ArmNull : public ArmBase {
public:
  ArmNull() : dof_{0} {};
  uint32_t dof_ = 0;
  std::string info() override { return std::string("null"); };
  const uint32_t &dof() override { return dof_; };
};

class ArmR6 : public ArmBase {
public:
  ArmR6(std::string name) : info_{name + "_industrial_R6"}, dof_{6U} {};

  std::string info() override { return info_; };

  const uint32_t &dof() override { return dof_; };

public:
  std::string info_;
  uint32_t dof_ = 6;
};

std::unique_ptr<ArmBase> ArmBase::create(const std::string &name,
                                         const ArmType &arm_type,
                                         const uint32_t dof,
                                         const float *urdf) {

  if (arm_type == ArmType::Industrial6)
    return std::make_unique<ArmR6>(name);

  return std::make_unique<ArmNull>();
};

} // namespace ampl