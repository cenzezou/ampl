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
  virtual std::string info() = 0;
  virtual const uint32_t &dof() = 0;
};
} // namespace ampl

#endif