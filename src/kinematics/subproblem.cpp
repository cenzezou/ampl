#include "kinematics_common.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ampl
{

void subproblem_PK2( const Eigen::Vector3d &p, const Eigen::Vector3d &q, const Eigen::Vector3d &r,
                     const Eigen::Vector3d &w1, const Eigen::Vector3d &w2, double *th1, double *th2, bool *status )
{
  Eigen::Vector3d u, v, z, c;
  double alpha, beta, gama;

  u = p - r;
  v = q - r;

  // 求解中间向量z
  double temp1, temp2;
  bool tmp_status[ 2 ];
  Eigen::Vector3d vec_temp;
  temp1 = ( w1.dot( w2 ) ) * w2.dot( u ) - w1.dot( v );
  temp2 = std::pow( ( w1.dot( w2 ) ), 2 ) - 1;

  alpha = temp1 / temp2;
  temp1 = ( w1.dot( w2 ) ) * w1.dot( v ) - w2.dot( u );
  beta  = temp1 / temp2;

  temp1 = u.norm() * u.norm() - alpha * alpha - beta * beta - 2 * alpha * beta * w1.dot( w2 );

  if ( temp1 < 0.0 )
  {
    status[ 0 ] = false;
    status[ 1 ] = false;
    return;
  }
  vec_temp = w1.cross( w2 );
  temp2    = vec_temp.norm() * vec_temp.norm();

  gama = -std::sqrt( temp1 / temp2 );

  z               = alpha * w1 + beta * w2 + gama * ( vec_temp );
  c               = z + r;
  tmp_status[ 0 ] = subproblem_PK1( c, q, w1, r, &th1[ 0 ] );
  tmp_status[ 1 ] = subproblem_PK1( p, c, w2, r, &th1[ 1 ] );
  status[ 0 ]     = tmp_status[ 0 ] && tmp_status[ 1 ];

  z               = alpha * w1 + beta * w2 - gama * ( vec_temp );
  c               = z + r;
  tmp_status[ 0 ] = subproblem_PK1( c, q, w1, r, &th2[ 0 ] );
  tmp_status[ 1 ] = subproblem_PK1( p, c, w2, r, &th2[ 1 ] );
  status[ 1 ]     = tmp_status[ 0 ] && tmp_status[ 1 ];

  // return true;
}

bool subproblem_RRR( const Eigen::Matrix3d &R, const Eigen::Vector3d &w1, const Eigen::Vector3d &w2,
                     const Eigen::Vector3d &w3, double *thA, double *thB )
{
  Eigen::Matrix3d w2Hat2;
  // so3_up(w2, w2Hat);
  so3_up2( w2, w2Hat2 );

  double a = -w1.dot( w2Hat2 * w3 );
  double b = w1.dot( w2.cross( w3 ) );
  double c = w1.dot( ( R - Eigen::Matrix3d::Identity() - w2Hat2 ) * w3 );

  thA[ 1 ] = std::atan2( b, a ) + std::atan2( std::sqrt( a * a + b * b - c * c ), c );
  thB[ 1 ] = std::atan2( b, a ) - std::atan2( std::sqrt( a * a + b * b - c * c ), c );

  Eigen::Vector3d v1, v3, ww3, ww1;
  ww1                   = R * w3;
  ww3                   = R.transpose() * w1;
  Eigen::Matrix3d thAw2 = Eigen::AngleAxisd( thA[ 1 ], w2 ).matrix();
  Eigen::Matrix3d thBw2 = Eigen::AngleAxisd( thB[ 1 ], w2 ).matrix();

  v1       = thAw2 * w3;
  v3       = thAw2.transpose() * w1;
  thA[ 0 ] = std::atan2( ww1.dot( w1.cross( v1 ) ), v1.dot( ww1 ) - ( v1.dot( w1 ) * ( ww1.dot( w1 ) ) ) );
  thA[ 2 ] = -1 * std::atan2( ww3.dot( w3.cross( v3 ) ), v3.dot( ww3 ) - ( v3.dot( w3 ) * ( ww3.dot( w3 ) ) ) );

  v1       = thBw2 * w3;
  v3       = thBw2.transpose() * w1;
  thB[ 0 ] = std::atan2( ww1.dot( w1.cross( v1 ) ), v1.dot( ww1 ) - ( v1.dot( w1 ) * ( ww1.dot( w1 ) ) ) );
  thB[ 2 ] = -1 * std::atan2( ww3.dot( w3.cross( v3 ) ), v3.dot( ww3 ) - ( v3.dot( w3 ) * ( ww3.dot( w3 ) ) ) );
  return true;
}

bool subproblem_PK1( const Eigen::Vector3d &p, const Eigen::Vector3d &q, const Eigen::Vector3d &w,
                     const Eigen::Vector3d &r, double *th )
{
  Eigen::Vector3d u   = p - r;
  Eigen::Vector3d v   = q - r;
  Eigen::Matrix3d wwt = w * w.transpose();
  u                   = u - wwt * u;
  v                   = v - wwt * v;
  *th                 = std::atan2( w.transpose() * ( u.cross( v ) ), u.dot( v ) );
  return true;
}

bool subproblem_PK3( const Eigen::Vector3d &p, const Eigen::Vector3d &q, const Eigen::Vector3d &r,
                     const Eigen::Vector3d &w, const double *delta, double *th_p, double *th_m )
{
  // Eigen::Vector3d u, v, u_temp, v_temp;
  Eigen::Vector3d u_temp = p - r;
  Eigen::Vector3d v_temp = q - r;
  Eigen::Matrix3d wwt    = w * w.transpose();
  double delta_temp_sqr, temp, th0;

  //   u = p - r;
  //   v = q - r;
  u_temp = u_temp - wwt * u_temp;
  v_temp = v_temp - wwt * v_temp;

  temp = w.dot( p - q );

  // delta_temp = std::sqrt(*delta * *delta - temp * temp);
  delta_temp_sqr = *delta * *delta - temp * temp;

  double u_temp_norm_sqr = u_temp.squaredNorm();
  double v_temp_norm_sqr = v_temp.squaredNorm();
  temp =
      ( u_temp_norm_sqr + v_temp_norm_sqr - delta_temp_sqr ) / ( 2 * std::sqrt( v_temp_norm_sqr * u_temp_norm_sqr ) );

  // std::cout << temp << std::endl;

  if ( std::abs( temp ) > 1.0 ) return false;

  th0 = std::atan2( w.transpose() * ( u_temp.cross( v_temp ) ), u_temp.dot( v_temp ) );

  *th_m = th0 - std::acos( temp );
  *th_p = th0 + std::acos( temp );

  return true;
};

}  // namespace ampl