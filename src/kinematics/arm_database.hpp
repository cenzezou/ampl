#ifndef AMPL_ARM_DATABASE_HPP
#define AMPL_ARM_DATABASE_HPP

#include <cmath>
namespace ampl {

struct ABB_IRB6700_150_320 {
  static constexpr auto name = "abb_irb6700_150_320";
  static constexpr auto type = "industrial";
  static constexpr auto dof = 6U;
  static constexpr const double urdf[dof * 9] = {
      0.0,    0.0, 0.78, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, //
      0.32,   0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 1.0, 0.0, //
      0.0,    0.0, 1.28, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, //
      0.0,    0.0, 0.2,  0.0, 0.0, 0.0, 1.0, 0.0, 0.0, //
      1.5925, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 1.0, 0.0, //
      0.2,    0.0, 0.0,  0.0, 0.0, 0.0, 1.0, 0.0, 0.0, //
  };
  static constexpr const double bounds[dof * 2] = {
      -2.9670597283903604, -1.1344640137963142, -3.141592653589793,
      -5.235987755982989,  -2.2689280275926285, -6.283185307179586,
      2.9670597283903604,  1.4835298641951802,  1.2217304763960306,
      5.235987755982989,   2.2689280275926285,  6.283185307179586};

  static constexpr const double tool0[6] = {
      0, 0, 0, 0, M_PI_2, 0 //
  };
};
struct YASKAWA_GP12 {
  static constexpr auto name = "yaskawa_gp12";
  static constexpr auto type = "industrial";
  static constexpr auto dof = 6U;
  static constexpr const double urdf[dof * 9] = {

      0,     0,  0.450, 0, 0,
      0,     0,  0,     1, //
      0.155, 0,  0,     0, 0,
      0,     0,  1,     0, //
      0,     0,  0.614, 0, 0,
      0,     0,  -1,    0, //
      0.640, 0,  0.200, 0, 0,
      0,     -1, 0,     0, //
      0,     0,  0,     0, 0,
      0,     0,  -1,    0, //
      0,     0,  0,     0, 0,
      0,     -1, 0,     0 //

      //   0.0,   0.0, 0.450, 0.0, 0.0, 0.0, 0.0,  0.0,  1.0, //
      //   0.155, 0.0, 0,     0.0, 0.0, 0.0, 0.0,  1.0,  0.0, //
      //   0.0,   0.0, 0.614, 0.0, 0.0, 0.0, 0.0,  -1.0, 0.0, //
      //   0.640, 0.0, 0.200, 0.0, 0.0, 0.0, -1.0, 0.0,  0.0, //
      //   0.0,   0.0, 0,     0.0, 0.0, 0.0, 0.0,  -1.0, 0.0, //
      //   0.0,   0.0, 0,     0.0, 0.0, 0.0, -1.0, 0.0,  0.0  //

  };

  static constexpr const double bounds[dof * 2] = {
      -2.9670, -1.5708, -1.4835, -3.4906, -2.6179, -7.9412, //
      +2.9670, +2.7052, +2.6179, +3.4906, +2.6179, +7.9412  //
  };
  static constexpr const double tool0[6] = {
      0.100, 0, 0, M_PI, -M_PI_2, 0 //
  };
};
struct ELFIN_10L {
  static constexpr auto name = "elfin_10l";
  static constexpr auto type = "industrial";
  static constexpr auto dof = 6U;
  static constexpr const double urdf[dof * 9] = {
      0, 0, 0.0685, 0, 0, 0, 0, 0,  1, //
      0, 0, 0.1915, 0, 0, 0, 0, -1, 0, //
      0, 0, 0.6,    0, 0, 0, 0, 1,  0, //
      0, 0, 0.156,  0, 0, 0, 0, 0,  1, //
      0, 0, 0.544,  0, 0, 0, 0, 1,  0, //
      0, 0, 0.1735, 0, 0, 0, 0, 0,  1  //
  };
  static constexpr const double bounds[dof * 2] = {
      -M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -M_PI, //
      M_PI,  M_PI,  M_PI,  M_PI,  M_PI,  M_PI   //
  };
  static constexpr const double tool0[6] = {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0 //
  };
};

} // namespace ampl

#endif