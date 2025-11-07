

#include "arm_database.hpp"
#include "arms.hpp"
#include "kinematics_common.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ampl/kinematics.hpp>
#include <iostream>
#include <memory>

namespace ampl {

void ArmR6::initialize_urdf(const double *urdf, const double *joint_limits) {
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
}
void ArmR6::initialize_preset(const std::string &name) {
  if (name == "abb_irb6700_150_320")
    initialize_urdf(ABB_IRB6700_150_320::urdf, ABB_IRB6700_150_320::bounds);
};

void ArmR6::set_joint_limits(double *joint_limits){};

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
};

} // namespace ampl