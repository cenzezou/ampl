#ifndef AMPL_BINDINGS_UTILS_HPP
#define AMPL_BINDINGS_UTILS_HPP

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
inline void init_utils(nanobind::module_ &pymodule) {

  pymodule.def("version", &ampl::version, "Print version of ampl in terminal.");
}
} // namespace ampl::binding
#endif