

#include "arm_database.hpp"
#include "arms.hpp"
#include "kinematics_common.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ampl/kinematics.hpp>
#include <iostream>
#include <memory>

namespace ampl {

void ArmR6::initialize_urdf(const double *urdf, const double *joint_limits,
                            const double *xyzrpy_tool0) {
  tfs_cache = urdf2tfs<K, double>(urdf);
  twists_home = urdf2twists<K, double>(urdf);
  link_end_fk_home = tfs2eehome<K>(tfs_cache.data());
  pns_joint = tfs2pn<K, double>(tfs_cache.data(), urdf);
  twists_poe = find_poetwists<K>(tfs_cache.data(), twists_home.data());

  constexpr uint32_t ids_wrist[3] = {3, 4, 5};
  q_wrist = find_wrist(pns_joint.data(), ids_wrist);
  qts_cache_f = urdf2qts<K, double, float>(urdf);
  qts_cache = urdf2qts<K, double, double>(urdf);
  twists_cache = urdf2axisarr<K, double>(urdf);

  if (joint_limits) {

    memcpy(q_lo.data(), joint_limits, K * sizeof(double));
    memcpy(q_hi.data(), joint_limits + K, K * sizeof(double));
  } else {
    q_lo.setConstant(-M_PI);
    q_hi.setConstant(M_PI);
  }

  tf_link_end_tool0.setIdentity();
  tf_link_end_tcp.setIdentity();
  if (xyzrpy_tool0) {
    set_link_end_tool0(xyzrpy_tool0);
  }
  tf_link_end_tcp = tf_link_end_tool0;
}
void ArmR6::initialize_preset(const std::string &name) {
  if (name == "abb_irb6700_150_320")
    initialize_urdf(ABB_IRB6700_150_320::urdf, ABB_IRB6700_150_320::bounds,
                    ABB_IRB6700_150_320::tool0);
  else if (name == "yaskawa_gp12")
    initialize_urdf(YASKAWA_GP12::urdf, YASKAWA_GP12::bounds,
                    YASKAWA_GP12::tool0);
  else if (name == "elfin_10l")
    initialize_urdf(ELFIN_10L::urdf, ELFIN_10L::bounds);
};

void ArmR6::set_joint_limits(const double *joint_limits_low,
                             const double *joint_limits_hi) {
  memcpy(q_lo.data(), joint_limits_low, K * sizeof(double));
  memcpy(q_hi.data(), joint_limits_hi, K * sizeof(double));
};

void ArmR6::set_link_end_tool0(const double *xyzrpy) {
  xyzrpy2tf<double>(xyzrpy, xyzrpy + 3, tf_link_end_tool0);

  quat_end_tool0 = quatd(tf_link_end_tool0.topLeftCorner<3, 3>());
  t_end_tool0 = tf_link_end_tool0.topRightCorner<3, 1>();
};

void ArmR6::get_pose_tool0(double *tf44, bool colmajor) {

};
void ArmR6::set_tcp(const double *tf44, bool colmajor) {

  if (colmajor) {

    Eigen::Map<const Mat4d> tf_tool0_tcp(tf44);
    tf_link_end_tcp = tf_link_end_tool0 * tf_tool0_tcp;
  } else {
    tf_link_end_tcp = Eigen::Map<const Mat4d>(tf44);
    tf_link_end_tcp.transposeInPlace();
    tf_link_end_tcp = tf_link_end_tool0 * tf_link_end_tcp;
  }
};

} // namespace ampl

namespace ampl {

void ArmR6::fk(const double *q, double *qts_link) {

  double *qt = qts_link;
  double *qtm1;
  Eigen::Map<const Eigen::Quaterniond> quat0_cache(qts_cache.data());
  Eigen::Map<const Vec3d> axis0_cache(twists_cache.data());
  Eigen::Map<Eigen::Quaterniond> quat0(qt);
  quat0 =
      quat0_cache * Eigen::Quaterniond(Eigen::AngleAxisd(q[0], axis0_cache));
  memcpy(qt + 4, qts_cache.data() + 4, 3 * sizeof(double));
  for (auto k = 1U; k < K; k++) {
    qtm1 = qt;
    qt += QT_SIZE;
    Eigen::Map<const Eigen::Quaterniond> quatk_cache(qts_cache.data() +
                                                     k * QT_SIZE);
    Eigen::Map<const Vec3d> transk_cache(qts_cache.data() + k * QT_SIZE + 4);
    Eigen::Map<const Vec3d> axisk_cache(twists_cache.data() + 3 * k);

    Eigen::Map<Eigen::Quaterniond> quatk(qt);
    Eigen::Map<Eigen::Quaterniond> quatkm1(qtm1);
    Eigen::Map<Vec3d> transk(qt + 4);
    Eigen::Map<Vec3d> transkm1(qtm1 + 4);
    quatk = quatkm1 * quatk_cache *
            Eigen::Quaterniond(Eigen::AngleAxisd(q[k], axisk_cache));
    transk = quatkm1 * transk_cache + transkm1;
  }

  qtm1 = qt;
  qt += QT_SIZE;

  Eigen::Map<Eigen::Quaterniond> quat_tool0(qt);
  Eigen::Map<Eigen::Quaterniond> quat_end(qtm1);
  std::cout << quat_end_tool0 << std::endl;
  Eigen::Map<Vec3d> trans_tool0(qt + 4);
  Eigen::Map<Vec3d> trans_end(qtm1 + 4);

  quat_tool0 = quat_end * quat_end_tool0;
  trans_tool0 = quat_end * t_end_tool0 + trans_end;
};

} // namespace ampl