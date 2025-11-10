

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ampl/kinematics.hpp>
#include <iostream>
#include <memory>

#include "arms.hpp"

namespace ampl
{

std::unique_ptr<ArmBase> ArmBase::create( const std::string &name, ArmType arm_type, uint32_t dof )
{
  if ( arm_type == ArmType::Industrial6 ) return std::make_unique<ArmR6>( name );

  if ( arm_type == ArmType::UR6 ) return std::make_unique<ArmNull>();

  if ( arm_type == ArmType::Humanoid7 ) return std::make_unique<ArmR7>( name );

  return std::make_unique<ArmNull>();
};

#ifdef ARM_TEMPLATE

#include "arms.hpp"
#include "arm_database.hpp"
#include "kinematics_common.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ampl/kinematics.hpp>
#include <iostream>
#include <memory>

namespace ampl
{

void ArmR7::initialize_urdf( const double *xyzrpyaxis, const double *joint_limits, const double *xyzrpy_tool0 ) {}
void ArmR7::initialize_preset( const std::string &name ){};
void ArmR7::set_joint_limits( const double *, const double * ){};
void ArmR7::fk( const double *q, double *qts_link ){};
void ArmR7::set_link_end_tool0( const double *xyzrpy ){};
void ArmR7::set_tcp( const double *tf44, bool colmajor ){};
void ArmR7::get_pose_tool0( double *tf44, bool colmajor ){};
unsigned char ArmR7::ik( const double *tf44_tool0, double *q8 ) { return static_cast<unsigned char>( 0 ); };
}  // namespace ampl

#endif

}  // namespace ampl