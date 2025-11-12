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
          "wxyz"_a, "t"_a, "return tf44" )
      .def(
          "distancefield_xyz2occ",
          []( const nb::ndarray<float, nb::shape<-1, 3>, nb::device::cpu, nb::c_contig> &xyz,  // NOAQ
              nb::ndarray<unsigned char> &occ,                                                 // NOAQ
              const nb::ndarray<uint32_t> &shape,                                              // NOAQ
              const nb::ndarray<float> &origin,                                                // NOAQ
              float dx                                                                         // NOAQ
          )
          {
            ampl::distancefield_xyz2occ( xyz.data(), xyz.shape( 0 ), occ.data(), shape.data()[ 0 ], shape.data()[ 1 ],
                                         shape.data()[ 2 ], origin.data()[ 0 ], origin.data()[ 1 ], origin.data()[ 2 ],
                                         dx );
            return;
          },
          "xyz"_a,     // NOAQ
          "occ"_a,     // NOAQ,
          "shape"_a,   // NOAQ
          "origin"_a,  // NOAQ
          "dx"_a,      // NOAQ
          "This function computes an occupation grid from a point cloud" )
      .def(
          "distancefield_occ2edf",
          []( const nb::ndarray<unsigned char, nb::shape<-1, 3>, nb::device::cpu, nb::c_contig> &occ,  // NOAQ
              nb::ndarray<float> &edf,                                                                 // NOAQ
              const nb::ndarray<uint32_t> &shape,                                                      // NOAQ
              const nb::ndarray<float> &origin,                                                        // NOAQ
              float dx,                                                                                // NOAQ
              uint32_t nb_worker                                                                       // NOAQ
          )
          {
            ampl::distancefield_occ2edf( occ.data(), edf.data(), shape.data()[ 0 ], shape.data()[ 1 ],
                                         shape.data()[ 2 ], dx, nb_worker );
            return;
          },
          "occ"_a,            // NOAQ
          "edf"_a,            // NOAQ,
          "shape"_a,          // NOAQ
          "origin"_a,         // NOAQ
          "dx"_a,             // NOAQ
          "nb_worker"_a = 4,  // NOAQ
          "This function computes an euclidean distance field from a occupation grid" )
      .def(
          "distancefield_xyz2edf",
          []( const nb::ndarray<float, nb::shape<-1, 3>, nb::device::cpu, nb::c_contig> &xyz,  // NOAQ
              nb::ndarray<unsigned char> &occ,                                                 // NOAQ
              nb::ndarray<float> &edf,                                                         // NOAQ
              const nb::ndarray<uint32_t> &shape,                                              // NOAQ
              const nb::ndarray<float> &origin,                                                // NOAQ
              float dx,                                                                        // NOAQ
              uint32_t nb_worker                                                               // NOAQ
          )
          {
            ampl::distancefield_xyz2occ( xyz.data(), xyz.shape( 0 ), occ.data(), shape.data()[ 0 ], shape.data()[ 1 ],
                                         shape.data()[ 2 ], origin.data()[ 0 ], origin.data()[ 1 ], origin.data()[ 2 ],
                                         dx );

            ampl::distancefield_occ2edf( occ.data(), edf.data(), shape.data()[ 0 ], shape.data()[ 1 ],
                                         shape.data()[ 2 ], dx, nb_worker );
            return;
          },
          "xyz"_a,            // NOAQ
          "occ"_a,            // NOAQ,
          "edf"_a,            // NOAQ,
          "shape"_a,          // NOAQ
          "origin"_a,         // NOAQ
          "dx"_a,             // NOAQ
          "nb_worker"_a = 4,  // NOAQ
          "This function computes an euclidean distance field from a point cloud" )
      .def(
          "distancefield_df2trimesh",
          []( const nb::ndarray<float> &df,        // NOAQ
              float df_offset,                     // NOAQ
              const nb::ndarray<uint32_t> &shape,  // NOAQ
              const nb::ndarray<float> &origin,    // NOAQ
              float dx                             // NOAQ
          )
          {
            Eigen::Matrix3Xf V;
            Eigen::Matrix<uint32_t, 3, Eigen::Dynamic> F;
            ampl::distancefield_edf2trimesh<float>( df.data(), df_offset,                                        // NOAQ
                                                    shape.data()[ 0 ], shape.data()[ 1 ], shape.data()[ 2 ],     // NOAQ
                                                    origin.data()[ 0 ], origin.data()[ 1 ], origin.data()[ 2 ],  // NOAQ
                                                    dx, V, F );

            const uint32_t sizeV = V.cols() * 3;
            float *dataV         = new float[ sizeV ];
            memcpy( dataV, V.data(), sizeof( float ) * sizeV );
            nb::capsule ownerV( dataV, []( void *p ) noexcept { delete[]( float * ) p; } );
            const uint32_t sizeF = F.cols() * 3;
            uint32_t *dataF      = new uint32_t[ sizeF ];
            memcpy( dataF, F.data(), sizeof( uint32_t ) * sizeF );
            nb::capsule ownerF( dataF, []( void *p ) noexcept { delete[]( uint32_t * ) p; } );

            return std::tuple<nb::ndarray<nb::numpy, float>, nb::ndarray<nb::numpy, uint32_t>>(
                nb::ndarray<nb::numpy, float>( dataV, { sizeV / 3, 3 }, ownerV ),
                nb::ndarray<nb::numpy, uint32_t>( dataF, { sizeF / 3, 3 }, ownerF ) );
          },

          "df"_a,        // NOAQ,
          "mc_level"_a,  // NOAQ,
          "shape"_a,     // NOAQ
          "origin"_a,    // NOAQ
          "dx"_a,        // NOAQ
          "This function performs marching cube on a distance field under a "
          "given level." )

      ;

  ;
}
}  // namespace ampl::binding
#endif