#ifndef AMPL_ARM_DATABASE_HPP
#define AMPL_ARM_DATABASE_HPP
namespace ampl {

struct ABB_IRB6700_150_320 {
  static constexpr auto name = "abb_irb6700_150_320";
  static constexpr auto type = "industrial";
  static constexpr auto dof = 6U;
  static constexpr const double urdf[dof * 9] = {
      0.0,    0.0, 0.78, 0, 0, 0, 0.0, 0.0, 1.0, //
      0.32,   0.0, 0.0,  0, 0, 0, 0.0, 1.0, 0.0, //
      0.0,    0.0, 1.28, 0, 0, 0, 0.0, 1.0, 0.0, //
      0.0,    0.0, 0.2,  0, 0, 0, 1.0, 0.0, 0.0, //
      1.5925, 0.0, 0.0,  0, 0, 0, 0.0, 1.0, 0.0, //
      0.2,    0.0, 0.0,  0, 0, 0, 1.0, 0.0, 0.0, //
  };
  static constexpr const double bounds[dof * 2] = {
      -2.9670597283903604, -1.1344640137963142, -3.141592653589793,
      -5.235987755982989,  -2.2689280275926285, -6.283185307179586,
      2.9670597283903604,  1.4835298641951802,  1.2217304763960306,
      5.235987755982989,   2.2689280275926285,  6.283185307179586};
};

} // namespace ampl

#endif