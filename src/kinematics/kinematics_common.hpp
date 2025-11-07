#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#define QT_SIZE 7
namespace ampl {
template <typename REAL>
REAL distance_point_line3(const REAL *o, const REAL *n, const REAL *p) {
  double op[3] = {p[0] - o[0], p[1] - o[1], p[2] - o[2]};
  return std::pow(n[1] * op[0] - n[0] * op[1], 2) +
         std::pow(-(n[2] * op[0]) + n[0] * op[2], 2) +
         std::pow(n[2] * op[1] - n[1] * op[2], 2);
};
template <typename REAL>
bool intersect_line3(const REAL *os, const REAL *ns, uint32_t num_lines,
                     REAL *p) {
  Eigen::Matrix3d Q;
  Q.setZero();
  Eigen::Vector3d q;
  q.setZero();

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
} // namespace ampl

namespace ampl {
template <typename REAL>
static void xyzrpy2tf(const REAL *xyz, const REAL *rpy, Eigen::Matrix4d &Tf) {
  Tf.setIdentity();
  Tf(0, 3) = xyz[0];
  Tf(1, 3) = xyz[1];
  Tf(2, 3) = xyz[2];
  Tf.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()).matrix() *
      Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()).matrix() *
      Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()).matrix();
}

template <uint32_t K, typename REAL>
static const std::array<Eigen::Matrix4d, K> urdf2tfs(const REAL *urdf) {
  Eigen::Matrix4d tf_cache = Eigen::Matrix4d::Identity();
  std::array<Eigen::Matrix4d, K> tfs;
  for (auto k = 0U; k < K; k++) {
    const REAL *xyzrpy = &urdf[9 * k];
    xyzrpy2tf<REAL>(xyzrpy, xyzrpy + 3, tf_cache);
    tfs[k] = tf_cache;
  }
  return tfs;
};

template <uint32_t K, typename REAL>
static const std::array<Eigen::Vector<double, 6>, K>
urdf2twists(const REAL *urdf) {
  std::array<Eigen::Vector<double, 6>, K> out;
  for (auto k = 0U; k < K; k++) {
    out[k].template head<3>().setZero();
    out[k].template tail<3>() =
        Eigen::Vector3d(urdf[9 * k + 6], urdf[9 * k + 7], urdf[9 * k + 8]);
  }
  return out;
};

template <uint32_t K>
static const Eigen::Matrix4d tfs2eehome(const Eigen::Matrix4d *tfs_cache) {
  Eigen::Matrix4d tf_ee = Eigen::Matrix4d::Identity();
  for (auto k = 0U; k < K; k++)
    tf_ee *= tfs_cache[k];
  return tf_ee;
};

template <uint32_t K, typename REAL>
static const std::array<Eigen::Vector<double, 6>, K>
tfs2pn(const Eigen::Matrix4d *tfs_cache, const REAL *urdf) {
  Eigen::Matrix4d tf_ee = Eigen::Matrix4d::Identity();
  std::array<Eigen::Vector<double, 6>, K> out;

  for (auto k = 0U; k < K; k++) {
    tf_ee *= tfs_cache[k];

    out[k].template head<3>() = tf_ee.topRightCorner<3, 1>();
    out[k].template tail<3>() =
        tf_ee.topLeftCorner<3, 3>() *
        Eigen::Vector3d(urdf[9 * k + 6], urdf[9 * k + 7], urdf[9 * k + 8]);
  }

  return out;
};

template <typename REAL>
void so3_up(const Eigen::Vector3<REAL> &omg,
            Eigen::Ref<Eigen::Matrix3<REAL>> so3) {
  so3 << 0, -omg(2), omg(1), omg(2), 0, -omg(0), -omg(1), omg(0), 0;
};
template <typename REAL>
void SE3_Adj(const Eigen::Matrix4<REAL> &T, Eigen::Matrix<REAL, 6, 6> &A) {
  Eigen::Matrix3<REAL> so3 = Eigen::Matrix3<REAL>::Zero();
  so3_up<REAL>(T.template topRightCorner<3, 1>(), so3);
  A.setZero();
  A.template topLeftCorner<3, 3>() = T.template topLeftCorner<3, 3>();
  A.template bottomRightCorner<3, 3>() = T.template topLeftCorner<3, 3>();
  A.template topRightCorner<3, 3>() = so3 * T.template topLeftCorner(3, 3);
  A.template bottomLeftCorner<3, 3>().setZero();
};

template <uint32_t K>
static const std::array<Eigen::Vector<double, 6>, K>
find_poetwists(const Eigen::Matrix4d *tfs_cache,
               const Eigen::Vector<double, 6> *twists) {
  std::array<Eigen::Vector<double, 6>, K> out;
  Eigen::Matrix4d tf_ee = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 6, 6> A;
  for (auto k = 0U; k < K; k++) {
    tf_ee *= tfs_cache[k];
    SE3_Adj<double>(tf_ee, A);
    out[k] = A * twists[k];
  }
  return out;
};

static const Eigen::Vector3d find_wrist(const Eigen::Vector<double, 6> *pns,
                                        const uint32_t *ids) {
  Eigen::Vector3d p_wrist;
  double os[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  double ns[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

  for (int i = 0; i < 3; i++) {
    if (ids[i] > 7)
      continue;

    const double *oi = pns[ids[i]].head<3>().data();
    const double *ni = pns[ids[i]].tail<3>().data();
    for (int k = 0; k < 3; k++) {
      os[i * 3 + k] = oi[k];
      ns[i * 3 + k] = ni[k];
    }
  }

  if (ids[2] > 7) {
    intersect_line3<double>(os, ns, 2, p_wrist.data());
  } else
    intersect_line3<double>(os, ns, 3, p_wrist.data());
  Eigen::Vector3d dist_to_joint;

  dist_to_joint[0] = distance_point_line3<double>(pns[ids[0]].head<3>().data(),
                                                  pns[ids[0]].tail<3>().data(),
                                                  p_wrist.data());

  dist_to_joint[1] = distance_point_line3<double>(pns[ids[1]].head<3>().data(),
                                                  pns[ids[1]].tail<3>().data(),
                                                  p_wrist.data());

  if (ids[2] <= 7) {
    dist_to_joint[2] = distance_point_line3<double>(
        pns[ids[2]].head<3>().data(), pns[ids[2]].tail<3>().data(),
        p_wrist.data());
  }

  // std::cout << dist_to_joint << std::endl;
  // std::cout << p_wrist << std::endl;

  return p_wrist;
}

template <uint32_t K, typename REAL, typename REAL_QT>
static const std::array<REAL_QT, K * 7> urdf2qts(const REAL *urdf) {
  Eigen::Matrix4d tf_cache = Eigen::Matrix4d::Identity();
  std::array<REAL_QT, K * 7> qts;
  for (auto k = 0U; k < K; k++) {
    const REAL *xyzrpy = &urdf[9 * k];
    xyzrpy2tf<REAL>(xyzrpy, xyzrpy + 3, tf_cache);
    Eigen::Quaterniond quat(tf_cache.topLeftCorner<3, 3>());
    Eigen::Vector3d trans = tf_cache.topRightCorner<3, 1>();
    // rwt
    qts[k * 7 + 0] = quat.coeffs()(0);
    qts[k * 7 + 1] = quat.coeffs()(1);
    qts[k * 7 + 2] = quat.coeffs()(2);
    qts[k * 7 + 3] = quat.coeffs()(3);
    qts[k * 7 + 4] = trans(0);
    qts[k * 7 + 5] = trans(1);
    qts[k * 7 + 6] = trans(2);
  }
  return qts;
};

template <uint32_t K, typename REAL>
static const std::array<double, K * 3> urdf2axisarr(const REAL *urdf) {
  std::array<double, K * 3> out;
  for (auto k = 0U; k < K; k++) {
    out[3 * k + 0] = urdf[9 * k + 6];
    out[3 * k + 1] = urdf[9 * k + 7];
    out[3 * k + 2] = urdf[9 * k + 8];
  }
  return out;
};

} // namespace ampl