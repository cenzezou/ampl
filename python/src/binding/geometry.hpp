#ifndef AMPL_BINDING_GEOMETRY_HPP
#define AMPL_BINDING_GEOMETRY_HPP

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

namespace ampl::binding {

inline void init_geometry(nanobind::module_ &pymodule) {

  pymodule.def(
      "transform_xyz",
      [](const nb::ndarray<float, nb::shape<-1, 3>, nb::device::cpu> &xyz_src,
         nb::ndarray<float, nb::shape<-1, 3>, nb::device::cpu> &xyz_dst,
         nb::ndarray<float, nb::numpy, nb::device::cpu> &rwt

      ) {
        ampl::transform_xyz<float>(rwt.data(), rwt.data() + 4, xyz_src.data(),
                                   xyz_src.shape(0), xyz_dst.data());
      },
      "xyz_src"_a, "xyz_dst"_a, "rwt"_a);
}
} // namespace ampl::binding
#endif