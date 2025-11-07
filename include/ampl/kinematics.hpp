#ifndef AMPL_KINEMTICS_HPP
#define AMPL_KINEMTICS_HPP

#include <memory>
#include <string>

namespace ampl {

enum class ArmType { Industrial6, UR6, Humanoid7 };

class ArmBase {
public:
  virtual ~ArmBase() = default;
  static std::unique_ptr<ArmBase> create(const std::string &name,
                                         const ArmType &arm_type,
                                         const uint32_t dof, const float *urdf);
  static std::unique_ptr<ArmBase>
  create(const std::string &name, const ArmType &arm_type, const uint32_t dof);
  virtual std::string info() = 0;
  virtual const uint32_t &dof() = 0;

  virtual void initialize_urdf(const double *xyzrpyaxis,
                               const double *joint_limits = nullptr) = 0;

  virtual void initialize_preset(const std::string &name) = 0;
  virtual void set_joint_limits(double *joint_limits) = 0;
  virtual void fk(const double *q, double *qts_link) = 0;
};
} // namespace ampl

#endif