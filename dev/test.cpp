#include <Eigen/Dense>

#include "../src/algo/simd_sse.hpp"
#include "test_util.hpp"
#include <ampl/ampl.hpp>
#include <iostream>

void test_tf( int argc, char *argv[] )
{
  Eigen::Matrix3Xf xyz_3X;
  Eigen::MatrixX3f xyz_X3;
  Eigen::Matrix3Xf xyz_3X_res, xyz_3X_res_quat, xyz_3X_res_affine;
  Eigen::MatrixX3f xyz_X3_res, xyz_X3_res_quat;
  int num_xyz = 500 - 2;

  Eigen::Matrix<float, 3, 3, Eigen::ColMajor> R_col;
  Eigen::Matrix<float, 3, 3, Eigen::RowMajor> R_row;

  R_col = Eigen::AngleAxisf( 0.1, Eigen::Vector3f::Random().normalized() ).matrix();
  Eigen::Quaternionf quat( R_col );

  R_row = R_col.eval();

  Eigen::RowVector3f t_row;
  Eigen::Vector3f t_col;
  {
    t_row.setRandom();
    t_col = t_row.transpose();
  }

  {
    xyz_X3.resize( num_xyz, 3 );
    xyz_3X.resize( 3, num_xyz );
    xyz_3X.setRandom();
    xyz_X3 = xyz_3X.transpose();
  }

  {
    xyz_3X_res        = xyz_3X;
    xyz_X3_res        = xyz_X3;
    xyz_3X_res_quat   = xyz_3X;
    xyz_X3_res_quat   = xyz_X3;
    xyz_3X_res_affine = xyz_3X;
  }

  {
    for ( int i = 0; i < 1000; i++ ) xyz_3X_res = R_col * xyz_3X;
  }
  int trial = 10000;

  // iso = iso.pretranslate( t_col );

  std::cout << "xyz_3X_res = R_col * xyz_3X" << std::endl;
  tic();
  for ( int i = 0; i < trial; i++ )
  {
    xyz_3X_res = R_col * xyz_3X;
    xyz_3X_res.colwise() += t_col;
  }
  toc( true );

  Eigen::MatrixX3f xyz_X3_copy = xyz_X3;
  xyz_X3_copy.setZero();

  std::cout << "SIMD" << std::endl;
  tic();
  for ( int i = 0; i < trial; i++ )
  {
    ampl::transform_points_soa( xyz_X3.col( 0 ).data(), xyz_X3.col( 1 ).data(), xyz_X3.col( 2 ).data(), num_xyz,
                                quat.coeffs().data(), t_row.data(),  // NOAQ
                                xyz_X3_copy.col( 0 ).data(), xyz_X3_copy.col( 1 ).data(),
                                xyz_X3_copy.col( 2 ).data()  // NOAQ
    );
  }
  toc( true );

  std::cout << ( xyz_X3_copy.transpose() - xyz_3X_res ).cwiseAbs().maxCoeff() << std::endl;
  // return;

  std::cout << "transform_xyz" << std::endl;

  tic();

  for ( int i = 0; i < trial; i++ )
  {
    ampl::transform_xyz<float>( quat.coeffs().data(), t_col.data(), xyz_3X.data(), xyz_3X.cols(),
                                xyz_3X_res_quat.data() );
    // for (int j = 0; j < num_xyz; j++)
    //   xyz_3X_res_quat.col(j) = (quat * xyz_3X.col(j));
    // xyz_3X_res_quat.colwise() += t_col;
  }
  toc( true );
  // std::cout << xyz_3X_res << std::endl;
  // std::cout << xyz_3X_res_quat << std::endl;
  std::cout << ( xyz_3X_res_quat - xyz_3X_res ).cwiseAbs().maxCoeff() << std::endl;
  std::cout << "xyz_X3_res = xyz_X3 * R_col" << std::endl;
  tic();
  for ( int i = 0; i < trial; i++ )
  {
    xyz_X3_res = xyz_X3 * R_col.transpose();
    xyz_X3_res.rowwise() += t_row;
  }
  toc( true );
  std::cout << ( xyz_X3_res.transpose() - xyz_3X_res ).cwiseAbs().maxCoeff() << std::endl;

  std::cout << "xyz_3X_res = R_row * xyz_3X" << std::endl;
  tic();
  for ( int i = 0; i < trial; i++ )
  {
    xyz_3X_res = R_row * xyz_3X;
    xyz_3X_res.colwise() += t_col;
  }
  toc( true );
  std::cout << ( xyz_X3_res.transpose() - xyz_3X_res ).cwiseAbs().maxCoeff() << std::endl;

  std::cout << "xyz_X3_res = xyz_X3 * R_row" << std::endl;
  tic();
  for ( int i = 0; i < trial; i++ )
  {
    xyz_X3_res = xyz_X3 * R_row.transpose();
    xyz_X3_res.rowwise() += t_row;
  }
  toc( true );
  std::cout << ( xyz_X3_res.transpose() - xyz_3X_res ).cwiseAbs().maxCoeff() << std::endl;
}
int main( int argc, char *argv[] )
{
  // test_tf( argc, argv );
#ifdef TEST_0
  {
    float ns[ 6 ] = { 1, 0, 0, 0, 1, 0 };
    float os[ 6 ] = { 0, 0, 0, 0, 0, 0 };

    Eigen::Vector3f p;

    ampl::intersectf_line3( os, ns, 2, p.data() );
  }
#endif
  std::unique_ptr<ampl::ArmBase> m;
  m = ampl::ArmBase::create( std::string( "abb_irb6700_150_320" ), ampl::ArmType::Industrial6, 6 );

  std::cout << m->info() << std::endl;

  Eigen::Matrix<double, 7, 7> qts_link;
  m->initialize_preset( "abb_irb6700_150_320" );
  // return 0;
  Eigen::Vector<double, 6> q;
  q << 0, 0, 0, 0, 0, 0;
  q.setConstant( 0.1 );
  m->fk( q.data(), qts_link.data() );
  std::cout << qts_link << std::endl;

  Eigen::Matrix4d tf_link_end;
  ampl::convert_qt_to_tf( qts_link.col( 6 ).data(), tf_link_end.data() );
  // std::cout << tf_link_end << std::endl;

  Eigen::Matrix<double, 6, 8> qs_ik;
  unsigned char ik_status;

  ik_status = m->ik( tf_link_end.data(), qs_ik.data() );
  /// std::cout << qts_link << std::endl;
  std::cout << qs_ik.transpose() << std::endl;

  return 0;
}