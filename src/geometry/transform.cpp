

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ampl/geometry.hpp>
#include <iostream>
#include <math.h>

namespace ampl
{
template void transform_xyz<double>( const double *rw, const double *t, const double *xyz, uint32_t nb_xyz,
                                     double *xyz_dst );
template void transform_xyz<float>( const float *rw, const float *t, const float *xyz, uint32_t nb_xyz,
                                    float *xyz_dst );

template void convert_qt_to_tf<float>( const float *qt7, float *tf44, bool colmajor, bool q_then_t );
template void convert_qt_to_tf<double>( const double *qt7, double *tf44, bool colmajor, bool q_then_t );
template void convert_tf_to_qt<float>( const float *tf44, float *qt7, bool colmajor, bool q_then_t );
template void convert_tf_to_qt<double>( const double *tf44, double *qt7, bool colmajor, bool q_then_t );

template void so3_up<float>( const Eigen::Vector3<float> &w, Eigen::Ref<Eigen::Matrix3<float>> so3 );

template void so3_up<double>( const Eigen::Vector3<double> &w, Eigen::Ref<Eigen::Matrix3<double>> so3 );
template <typename REAL>
void so3_up( const Eigen::Vector3<REAL> &omg, Eigen::Ref<Eigen::Matrix3<REAL>> so3 )
{
  so3 << 0, -omg( 2 ), omg( 1 ), omg( 2 ), 0, -omg( 0 ), -omg( 1 ), omg( 0 ), 0;
};

template <typename REAL>
void transform_xyz( const REAL *rw, const REAL *t, const REAL *xyz, uint32_t nb_xyz, REAL *xyz_dst )
{
  Eigen::Quaternion<REAL> quat( rw );

  // std::cout << quat << std::endl;
  Eigen::Map<const Eigen::Matrix3X<REAL>> V( xyz, 3, nb_xyz );
  Eigen::Map<Eigen::Matrix3X<REAL>> V_dst( xyz_dst, 3, nb_xyz );
  for ( uint32_t j = 0; j < nb_xyz; j++ ) V_dst.col( j ) = ( quat * V.col( j ) );
  Eigen::Map<const Eigen::Vector3<REAL>> T( t );
  V_dst.colwise() += T;
};

template <typename REAL>
void convert_qt_to_tf( const REAL *qt7, REAL *tf44, bool colmajor, bool q_then_t )
{
  memset( tf44, 0, 16 * sizeof( double ) );

  if ( colmajor )
  {
    Eigen::Map<Eigen::Matrix<REAL, 4, 4>> tf( tf44 );
    if ( q_then_t )
    {
      Eigen::Map<const Eigen::Quaternion<REAL>> qt( qt7 );
      // Eigen::Quaterniond::toRotationMatrix
      tf.template topLeftCorner<3, 3>() = qt.toRotationMatrix();
      memcpy( tf44 + 12, qt7 + 4, 3 * sizeof( REAL ) );
    }
    else
    {
      Eigen::Map<const Eigen::Quaternion<REAL>> qt( qt7 + 3 );
      // Eigen::Quaterniond::toRotationMatrix
      tf.template topLeftCorner<3, 3>() = qt.toRotationMatrix();
      memcpy( tf44 + 12, qt7 + 0, 3 * sizeof( REAL ) );
    }
    tf44[ 15 ] = 1;
    return;
  }
  else
  {
    Eigen::Map<Eigen::Matrix<REAL, 4, 4, Eigen::RowMajor>> tf( tf44 );
    if ( q_then_t )
    {
      Eigen::Map<const Eigen::Quaternion<REAL>> qt( qt7 );
      // Eigen::Quaterniond::toRotationMatrix
      tf.template topLeftCorner<3, 3>() = qt.toRotationMatrix();
      tf44[ 3 ]                         = qt7[ 4 ];
      tf44[ 7 ]                         = qt7[ 5 ];
      tf44[ 11 ]                        = qt7[ 6 ];
    }
    else
    {
      Eigen::Map<const Eigen::Quaternion<REAL>> qt( qt7 + 3 );
      // Eigen::Quaterniond::toRotationMatrix
      tf.template topLeftCorner<3, 3>() = qt.toRotationMatrix();
      tf44[ 3 ]                         = qt7[ 0 ];
      tf44[ 7 ]                         = qt7[ 1 ];
      tf44[ 11 ]                        = qt7[ 2 ];
    }

    tf44[ 15 ] = 1;
    return;
  }
}

template <typename REAL>
void convert_tf_to_qt( const REAL *tf44, REAL *qt7, bool colmajor, bool q_then_t )
{
  if ( colmajor )
  {
    Eigen::Map<const Eigen::Matrix<REAL, 4, 4>> tf( tf44 );
    if ( q_then_t )
    {
      Eigen::Map<Eigen::Quaternion<REAL>> qt( qt7 );
      qt = Eigen::Quaternion<REAL>( tf.template topLeftCorner<3, 3>() );
      memcpy( qt7 + 4, tf44 + 12, 3 * sizeof( REAL ) );
      // qt7[ 4 ] = tf44[ 12 ];
      // qt7[ 5 ] = tf44[ 13 ];
      // qt7[ 6 ] = tf44[ 14 ];
      return;
    }
    else
    {
      Eigen::Map<Eigen::Quaternion<REAL>> qt( qt7 + 3 );
      qt = Eigen::Quaternion<REAL>( tf.template topLeftCorner<3, 3>() );
      memcpy( qt7 + 0, tf44 + 12, 3 * sizeof( REAL ) );

      // qt7[ 0 ] = tf44[ 12 ];
      // qt7[ 1 ] = tf44[ 13 ];
      // qt7[ 2 ] = tf44[ 14 ];
      return;
    }
  }
  else
  {
    Eigen::Map<const Eigen::Matrix<REAL, 4, 4, Eigen::RowMajor>> tf( tf44 );
    if ( q_then_t )
    {
      Eigen::Map<Eigen::Quaternion<REAL>> qt( qt7 );
      qt       = Eigen::Quaternion<REAL>( tf.template topLeftCorner<3, 3>() );
      qt7[ 4 ] = tf44[ 3 ];
      qt7[ 5 ] = tf44[ 7 ];
      qt7[ 6 ] = tf44[ 11 ];
      return;
    }
    else
    {
      Eigen::Map<Eigen::Quaternion<REAL>> qt( qt7 + 3 );
      qt       = Eigen::Quaternion<REAL>( tf.template topLeftCorner<3, 3>() );
      qt7[ 0 ] = tf44[ 3 ];
      qt7[ 1 ] = tf44[ 7 ];
      qt7[ 2 ] = tf44[ 11 ];
      return;
    }
  }
}
}  // namespace ampl