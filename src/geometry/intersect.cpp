
#include <Eigen/Dense>
#include <ampl/geometry.hpp>
#include <math.h>

#include <stdio.h>
namespace ampl {

template bool intersect_line3<float>(const float *, const float *, uint32_t,
                                     float *);

template bool intersect_line3<double>(const double *, const double *, uint32_t,
                                      double *);

template bool intersect_plane3<float>(const float *p0, const float *n0,
                                      const float *p1, const float *n1,
                                      float *o, float *t);
template bool intersect_plane3<double>(const double *p0, const double *n0,
                                       const double *p1, const double *n1,
                                       double *o, double *t);

template <typename REAL>
bool intersect_line3(const REAL *os, const REAL *ns, uint32_t nb_line, REAL *p);

template <typename REAL>
bool intersect_line3(const REAL *os, const REAL *ns, uint32_t num_lines,
                     REAL *p) {
  Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
  Eigen::Vector3d q = Eigen::Vector3d::Zero();

  Eigen::Matrix3d Qi;
  Eigen::Vector3d qi;

  for (size_t i = 0; i < num_lines; i++) {
    double nx = ns[i * 3 + 0];
    double ny = ns[i * 3 + 1];
    double nz = ns[i * 3 + 2];
    double ox = os[i * 3 + 0];
    double oy = os[i * 3 + 1];
    double oz = os[i * 3 + 2];
    Qi << 1 - 2 * std::pow(nx, 2) + std::pow(nx, 4) +
              std::pow(nx, 2) * std::pow(ny, 2) +
              std::pow(nx, 2) * std::pow(nz, 2),
        (-4 * nx * ny + 2 * std::pow(nx, 3) * ny + 2 * nx * std::pow(ny, 3) +
         2 * nx * ny * std::pow(nz, 2)) /
            2.,
        (-4 * nx * nz + 2 * std::pow(nx, 3) * nz +
         2 * nx * std::pow(ny, 2) * nz + 2 * nx * std::pow(nz, 3)) /
            2.,
        (-4 * nx * ny + 2 * std::pow(nx, 3) * ny + 2 * nx * std::pow(ny, 3) +
         2 * nx * ny * std::pow(nz, 2)) /
            2.,
        1 - 2 * std::pow(ny, 2) + std::pow(nx, 2) * std::pow(ny, 2) +
            std::pow(ny, 4) + std::pow(ny, 2) * std::pow(nz, 2),
        (-4 * ny * nz + 2 * std::pow(nx, 2) * ny * nz +
         2 * std::pow(ny, 3) * nz + 2 * ny * std::pow(nz, 3)) /
            2.,
        (-4 * nx * nz + 2 * std::pow(nx, 3) * nz +
         2 * nx * std::pow(ny, 2) * nz + 2 * nx * std::pow(nz, 3)) /
            2.,
        (-4 * ny * nz + 2 * std::pow(nx, 2) * ny * nz +
         2 * std::pow(ny, 3) * nz + 2 * ny * std::pow(nz, 3)) /
            2.,
        1 - 2 * std::pow(nz, 2) + std::pow(nx, 2) * std::pow(nz, 2) +
            std::pow(ny, 2) * std::pow(nz, 2) + std::pow(nz, 4);

    qi << -2 * ox + 4 * std::pow(nx, 2) * ox - 2 * std::pow(nx, 4) * ox -
              2 * std::pow(nx, 2) * std::pow(ny, 2) * ox -
              2 * std::pow(nx, 2) * std::pow(nz, 2) * ox + 4 * nx * ny * oy -
              2 * std::pow(nx, 3) * ny * oy - 2 * nx * std::pow(ny, 3) * oy -
              2 * nx * ny * std::pow(nz, 2) * oy + 4 * nx * nz * oz -
              2 * std::pow(nx, 3) * nz * oz -
              2 * nx * std::pow(ny, 2) * nz * oz -
              2 * nx * std::pow(nz, 3) * oz,
        4 * nx * ny * ox - 2 * std::pow(nx, 3) * ny * ox -
            2 * nx * std::pow(ny, 3) * ox - 2 * nx * ny * std::pow(nz, 2) * ox -
            2 * oy + 4 * std::pow(ny, 2) * oy -
            2 * std::pow(nx, 2) * std::pow(ny, 2) * oy -
            2 * std::pow(ny, 4) * oy -
            2 * std::pow(ny, 2) * std::pow(nz, 2) * oy + 4 * ny * nz * oz -
            2 * std::pow(nx, 2) * ny * nz * oz - 2 * std::pow(ny, 3) * nz * oz -
            2 * ny * std::pow(nz, 3) * oz,
        4 * nx * nz * ox - 2 * std::pow(nx, 3) * nz * ox -
            2 * nx * std::pow(ny, 2) * nz * ox - 2 * nx * std::pow(nz, 3) * ox +
            4 * ny * nz * oy - 2 * std::pow(nx, 2) * ny * nz * oy -
            2 * std::pow(ny, 3) * nz * oy - 2 * ny * std::pow(nz, 3) * oy -
            2 * oz + 4 * std::pow(nz, 2) * oz -
            2 * std::pow(nx, 2) * std::pow(nz, 2) * oz -
            2 * std::pow(ny, 2) * std::pow(nz, 2) * oz -
            2 * std::pow(nz, 4) * oz;

    Q += Qi;
    q += qi;
  }
  Eigen::Matrix3d A = 2 * Q;
  Eigen::Vector3d x = A.colPivHouseholderQr().solve(-q);
  p[0] = x[0];
  p[1] = x[1];
  p[2] = x[2];
  return x.hasNaN();
};

template <typename REAL>
bool intersect_plane3(const REAL *p0, const REAL *n0, const REAL *p1,
                      const REAL *n1, REAL *o, REAL *t) {
  constexpr REAL D_EPS = 1e-6;
  double d[3];

  d[0] = -n0[2] * n1[1] + n0[1] * n1[2];
  d[1] = n0[2] * n1[0] - n0[0] * n1[2];
  d[2] = -n0[1] * n1[0] + n0[0] * n1[1];

  double dnorm = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);

  if (dnorm < D_EPS)
    return false;

  d[0] /= dnorm;
  d[1] /= dnorm;
  d[2] /= dnorm;

  double move_dir[3];

  move_dir[0] = -d[2] * n0[1] + d[1] * n0[2];
  move_dir[1] = d[2] * n0[0] - d[0] * n0[2];
  move_dir[2] = -d[1] * n0[0] + d[0] * n0[1];

  double lambda = (p1[0] - p0[0]) * n1[0] + (p1[1] - p0[1]) * n1[1] +
                  (p1[2] - p0[2]) * n1[2];
  lambda /= (move_dir[0] * n1[0] + move_dir[1] * n1[1] + move_dir[2] * n1[2]);

  t[0] = d[0];
  t[1] = d[1];
  t[2] = d[2];

  o[0] = p0[0] + move_dir[0] * lambda;
  o[1] = p0[1] + move_dir[1] * lambda;
  o[2] = p0[2] + move_dir[2] * lambda;

  return true;
};

} // namespace ampl