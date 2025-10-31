

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

class ArmR7 : public ArmBase {
public:
  ArmR7(std::string name) : info_{name + "_humanoid_R7"}, dof_{7U} {};
  std::string info() override { return info_; };
  const uint32_t &dof() override { return dof_; };

public:
  std::string info_;
  uint32_t dof_ = 7;
};

} // namespace ampl