#ifndef AMPL_KINEMATICS_HPP
#define AMPL_KINEMATICS_HPP

#include <memory>
#include <string>

namespace ampl
{

enum class ArmType
{
  Industrial6,
  UR6,
  Humanoid7
};

static const char *ArmPresetIndustrial6[ 3 ] = { "abb_irb6700_150_320", "yaskawa_gp12", "elfin_10l" };
static const char *ArmPresetHumanoid7[ 3 ]   = { "hillbot_left", "hillbot_right", "tianji_left" };

class ArmBase
{
 public:
  virtual ~ArmBase() = default;

  static std::unique_ptr<ArmBase> create( const std::string &name, ArmType arm_type, uint32_t dof );
  virtual std::string info()    = 0;
  virtual const uint32_t &dof() = 0;

  virtual void initialize_urdf( const double *xyzrpyaxis, const double *joint_limits = nullptr,
                                const double *xyzrpy_tool0 = nullptr ) = 0;

  virtual void initialize_preset( const std::string &name ) = 0;
  /**
   * @brief
   * @param joint_limits_low
   * @param joint_limits_hi
   */
  virtual void set_joint_limits( const double *joint_limits_low, const double *joint_limits_hi ) = 0;
  /**
   * @brief
   * @param q array of dof lenght
   * @param qts_link qt_link_1 ... qt_link_dof qt_ee
   */
  virtual void fk( const double *q, double *qts_link ) = 0;
  /**
   * @brief
   * @param tf44_tool0 robot tool0 pose
   * @param q8 buffer of length dof * 8
   */
  virtual unsigned char ik( const double *tf44_tool0, double *q8 ) = 0;
  /**
   * @brief Tool0 = flann which is fixed for all robot arms
   * @param xyzrpy
   */
  virtual void set_link_end_tool0( const double *xyzrpy )           = 0;
  virtual void set_tcp( const double *tf44, bool colmajor = true )  = 0;
  virtual void get_pose_tool0( double *tf44, bool colmajor = true ) = 0;

  virtual void set_base( const double *tf44, bool colmajor = true ) = 0;
};
}  // namespace ampl

#endif