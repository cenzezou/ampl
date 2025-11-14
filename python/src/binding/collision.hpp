#ifndef AMPL_BINDINGS_COLLISION_HPP
#define AMPL_BINDINGS_COLLISION_HPP

#include <iostream>
#include <ampl/ampl.hpp>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/trampoline.h>
namespace nb = nanobind;
using namespace nb::literals;
namespace ampl::binding
{
inline void init_collision( nanobind::module_ &pymodule )

{
  nb::class_<soa::SphereGroupf<512, 8>>( pymodule, "VSphG8f" )
      .def( nb::init<>() )
      .def_rw( "nb_sph", &soa::SphereGroupf<512, 8>::nb_sph )
      .def_rw( "nb_offset", &soa::SphereGroupf<512, 8>::nb_offset );
  ;

  pymodule.def(
      "collision_initialize_object",
      []( const nb::ndarray<float, nb::shape<-1, 4>, nb::device::cpu> &xyzr,

          const nb::ndarray<uint32_t, nb::shape<-1>, nb::device::cpu> &offset, soa::SphereGroupf<512, 8> &spheres ) {
        ampl::collision_initialize_object<512>( spheres, xyzr.data(), xyzr.shape( 0 ), offset.data(),
                                                offset.shape( 0 ) );
      },

      "Print version of ampl in terminal." );
}
}  // namespace ampl::binding
#endif