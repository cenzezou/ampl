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

namespace ampl::binding
{

inline void init_geometry( nanobind::module_ &pymodule )
{
  pymodule
      .def(
          "transform_xyz",
          []( const nb::ndarray<float, nb::shape<-1, 3>, nb::device::cpu> &xyz_src,
              nb::ndarray<float, nb::shape<-1, 3>, nb::device::cpu> &xyz_dst,
              nb::ndarray<float, nb::numpy, nb::device::cpu> &rwt ) {
            ampl::transform_xyz<float>( rwt.data(), rwt.data() + 4, xyz_src.data(), xyz_src.shape( 0 ),
                                        xyz_dst.data() );
          },
          "xyz_src"_a, "xyz_dst"_a, "rwt"_a )
      .def(
          "tf44_to_qt7",
          []( const nb::ndarray<float, nb::shape<4, 4>> &tf )
          {
            Eigen::Vector<float, 7> qt;
            ampl::convertf_tf_to_qt( tf.data(), qt.data(), false, true );
            return qt;
          },
          "tf44"_a, "return qt7" )
      .def(
          "tf44_to_qt7",
          []( const nb::ndarray<double, nb::shape<4, 4>> &tf )
          {
            Eigen::Vector<double, 7> qt;
            ampl::convertd_tf_to_qt( tf.data(), qt.data(), false, true );
            return qt;
          },
          "tf44"_a, "return qt7" )
      .def(
          "qt7_to_tf44",
          []( const nb::ndarray<double, nb::shape<-1>> &qt7 )
          {
            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> tf;
            ampl::convertd_qt_to_tf( qt7.data(), tf.data(), false, true );
            return tf;
          },
          "qt7"_a, "return tf44" )

      .def(
          "qt7_to_tf44",
          []( const nb::ndarray<float, nb::shape<-1>> &qt7 )
          {
            Eigen::Matrix<float, 4, 4, Eigen::RowMajor> tf;
            ampl::convertf_qt_to_tf( qt7.data(), tf.data(), false, true );
            return tf;
          },
          "qt7"_a, "return tf44" )
      .def(
          "wxyz_t_to_tf44",
          []( const nb::ndarray<double, nb::shape<-1>> &wxyz, const nb::ndarray<double, nb::shape<-1>> &t )
          {
            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> tf;
            double qt7[ 7 ] = { wxyz.data()[ 1 ], wxyz.data()[ 2 ], wxyz.data()[ 3 ], wxyz.data()[ 0 ],
                                t.data()[ 0 ],    t.data()[ 1 ],    t.data()[ 2 ] };
            ampl::convertd_qt_to_tf( qt7, tf.data(), false, true );
            return tf;
          },
          "wxyz"_a, "t"_a, "return tf44" )
      .def(
          "wxyz_t_to_tf44",
          []( const nb::ndarray<float, nb::shape<-1>> &wxyz, const nb::ndarray<float, nb::shape<-1>> &t )
          {
            Eigen::Matrix<float, 4, 4, Eigen::RowMajor> tf;
            float qt7[ 7 ] = { wxyz.data()[ 1 ], wxyz.data()[ 2 ], wxyz.data()[ 3 ], wxyz.data()[ 0 ],
                               t.data()[ 0 ],    t.data()[ 1 ],    t.data()[ 2 ] };
            ampl::convertf_qt_to_tf( qt7, tf.data(), false, true );
            return tf;
          },
          "wxyz"_a, "t"_a, "return tf44" );

  ;
}
}  // namespace ampl::binding
#endif