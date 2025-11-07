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

struct PyArmBase {
  using arr4f = nb::ndarray<float, nb::numpy, nb::shape<4, 4>, nb::c_contig>;
  using arr4d = nb::ndarray<double, nb::numpy, nb::shape<4, 4>, nb::c_contig>;

private:
  std::unique_ptr<ampl::ArmBase> m;

public:
  PyArmBase(const std::string &name, ampl::ArmType arm_type, uint32_t dof) {
    // ampl::ArmType at = ampl::ArmType::Industrial6;
    m = ampl::ArmBase::create(name, arm_type, dof);
    m->initialize_preset(name);
  };
  std::string info() { return m->info(); }
};

namespace ampl::binding {

inline void init_kinematics(nanobind::module_ &pymodule) {

  nb::enum_<ampl::ArmType>(pymodule, "ArmType")
      .value("Humanoid7", ampl::ArmType::Humanoid7)
      .value("Industrial6", ampl::ArmType::Industrial6)
      .value("UR6", ampl::ArmType::UR6)
      .export_values(); // Optional: exports

  nb::class_<PyArmBase>(pymodule, "ArmBase")
      // Bind the constructor
      .def(nb::init<const std::string &, ampl::ArmType, uint32_t>())
      .def("info", &PyArmBase::info);
}
} // namespace ampl::binding
#endif