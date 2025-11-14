
#include "geometry.hpp"
#include "kinematics.hpp"
#include "utils.hpp"
#include "stlfile.hpp"
#include "trimesh.hpp"
#include "collision.hpp"
NB_MODULE( _core_ext, pymodule )
{
  ampl::binding::init_utils( pymodule );
  ampl::binding::init_kinematics( pymodule );
  ampl::binding::init_geometry( pymodule );
  ampl::binding::init_stl_reader( pymodule );
  ampl::binding::init_trimesh( pymodule );
  ampl::binding::init_collision( pymodule );
}
