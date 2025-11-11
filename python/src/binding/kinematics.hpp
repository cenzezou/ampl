#ifndef AMPL_BINDING_KINEMATICS_HPP
#define AMPL_BINDING_KINEMATICS_HPP

#include <iostream>
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>

#include <ampl/ampl.hpp>
#include <nanobind/trampoline.h>
namespace nb = nanobind;
using namespace nb::literals;

struct PyArmBase
{
  using arr4f = nb::ndarray<float, nb::numpy, nb::shape<4, 4>, nb::c_contig>;
  using arr4d = nb::ndarray<double, nb::numpy, nb::shape<4, 4>, nb::c_contig>;

  using anyXd  = nb::ndarray<double, nb::numpy, nb::device::cpu>;
  using anyXcd = nb::ndarray<const double, nb::numpy, nb::device::cpu>;
  using matXd  = nb::ndarray<double, nb::shape<-1, -1>, nb::device::cpu>;
  using mat4d  = nb::ndarray<double, nb::shape<4, 4>, nb::device::cpu>;
  using matXcd = nb::ndarray<const double, nb::shape<-1, -1>, nb::device::cpu>;
  using arrXd  = nb::ndarray<double, nb::shape<-1>, nb::device::cpu>;
  using arrXcd = nb::ndarray<const double, nb::shape<-1>, nb::device::cpu>;

 private:
  std::unique_ptr<ampl::ArmBase> m;

 public:
  PyArmBase( const std::string &name, ampl::ArmType arm_type, uint32_t dof )
  {
    m = ampl::ArmBase::create( name, arm_type, dof );
    m->initialize_preset( name );
  };
  std::string info() { return m->info(); }

  void fk_qt7( arrXcd q, matXd qts ) { m->fk( q.data(), qts.data() ); }
  void set_base( matXd tf_world_base )
  {
    Eigen::Matrix4d tf_world_base_colmajor;
    for ( int k = 0; k < 16; k++ ) tf_world_base_colmajor.data()[ k ] = tf_world_base.data()[ k ];
    tf_world_base_colmajor.transposeInPlace();
    m->set_base( tf_world_base_colmajor.data(), true );
  }
  unsigned char ik( matXd tf_tool0, nb::ndarray<double, nb::shape<-1, -1>, nb::device::cpu> &q8 )
  {
    // double tf_tool0_colmajor[ 16 ];

    Eigen::Matrix4d tf_tool0_colmajor;
    for ( int k = 0; k < 16; k++ ) tf_tool0_colmajor.data()[ k ] = tf_tool0.data()[ k ];
    tf_tool0_colmajor.transposeInPlace();

    return m->ik( tf_tool0_colmajor.data(), q8.data() );
  }
};

namespace ampl::binding
{

inline void init_kinematics( nanobind::module_ &pymodule )
{
  nb::enum_<ampl::ArmType>( pymodule, "ArmType" )
      .value( "Humanoid7", ampl::ArmType::Humanoid7 )
      .value( "Industrial6", ampl::ArmType::Industrial6 )
      .value( "UR6", ampl::ArmType::UR6 )
      .export_values();

  nb::class_<PyArmBase>( pymodule, "ArmBase" )
      .def( nb::init<const std::string &, ampl::ArmType, uint32_t>(), "arm_preset"_a, "arm_type"_a, "dof"_a,
            "create a preset arm solver" )
      .def( "info", &PyArmBase::info )
      .def( "fk_links", &PyArmBase::fk_qt7, "q"_a, "arr_rwt"_a,
            "len(q) >= dof, arr_rwt.shape = (dof+1,7), rwt=[rx,ry,rz,w,tx,ty,tz]" )
      .def( "ik", &PyArmBase::ik, "m44_world_tool0"_a, "arr_qik"_a, "arr_qik.shape = (8,dof)" )
      .def( "set_base", &PyArmBase::set_base, "m44_world_base"_a, "set pose of arm base in world" );
  ;
  ;
}
}  // namespace ampl::binding
#endif