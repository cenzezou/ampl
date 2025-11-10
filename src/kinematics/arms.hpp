

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ampl/kinematics.hpp>
#include <iostream>
#include <memory>

namespace ampl
{

class ArmNull : public ArmBase
{
 public:
  ArmNull() : dof_{ 0 } {};
  uint32_t dof_ = 0;
  std::string info() override { return std::string( "not implemented" ); };
  const uint32_t &dof() override { return dof_; };
  void initialize_urdf( const double *xyzrpyaxis, const double *joint_limits = nullptr,
                        const double *xyzrpy_tool0 = nullptr ) override{};
  void set_joint_limits( const double *, const double * ) override{};
  void initialize_preset( const std::string &name ) override{};
  void fk( const double *q, double *qts_link ) override{};
  void set_link_end_tool0( const double *xyzrpy ) override{};
  void set_tcp( const double *tf44, bool colmajor = true ) override{};
  void get_pose_tool0( double *tf44, bool colmajor = true ) override{};
  unsigned char ik( const double *tf44_tool0, double *q8 ) override { return static_cast<unsigned char>( 0 ); };
};

class ArmR6 : public ArmBase
{
 public:
  typedef Eigen::Vector<double, 6> Vec6d;
  typedef Eigen::Vector<double, 3> Vec3d;
  typedef Eigen::Matrix4d Mat4d;
  typedef Eigen::Quaterniond quatd;

  ArmR6( std::string name ) : info_{ name + "_industrial_R6" } {};
  std::string info() override { return info_; };
  const uint32_t &dof() override { return K; };

  void initialize_urdf( const double *xyzrpyaxis, const double *joint_limits = nullptr,
                        const double *xyzrpy_tool0 = nullptr ) override;
  void set_joint_limits( const double *, const double * ) override;
  void initialize_preset( const std::string &name ) override;
  void fk( const double *q, double *qts_link ) override;
  void set_link_end_tool0( const double *xyzrpy ) override;
  void set_tcp( const double *tf44, bool colmajor = true ) override;
  void get_pose_tool0( double *tf44, bool colmajor ) override;
  unsigned char ik( const double *tf44_tool0, double *q8 ) override;

 public:
  static constexpr uint32_t K = 6;
  std::string info_;
  std::array<Mat4d, K> tfs_cache;
  std::array<Vec6d, K> pns_joint;
  std::array<Vec6d, K> twists_home;
  std::array<Vec6d, K> twists_poe;

  std::array<float, K * 7> qts_cache_f;
  std::array<double, K * 7> qts_cache;
  std::array<double, K * 3> twists_cache;

  Vec3d q_wrist;
  Mat4d link_end_fk_home;
  Vec6d q_lo;
  Vec6d q_hi;
  Mat4d tf_link_end_tool0, tf_tool0_link_end;
  Mat4d tf_link_end_tcp;

  quatd quat_end_tool0;
  Vec3d t_end_tool0;
};

class ArmR7 : public ArmBase
{
 public:
  typedef Eigen::Vector<double, 6> Vec6d;
  typedef Eigen::Vector<double, 3> Vec3d;
  typedef Eigen::Matrix4d Mat4d;
  typedef Eigen::Quaterniond quatd;

 public:
  ArmR7( std::string name ) : info_{ name + "_humanoid_R7" } {};
  std::string info() override { return info_; };
  const uint32_t &dof() override { return K; };
  void initialize_urdf( const double *xyzrpyaxis, const double *joint_limits = nullptr,
                        const double *xyzrpy_tool0 = nullptr ) override;
  void set_joint_limits( const double *, const double * ) override;
  void initialize_preset( const std::string &name ) override;
  void fk( const double *q, double *qts_link ) override;
  void set_link_end_tool0( const double *xyzrpy ) override;
  void set_tcp( const double *tf44, bool colmajor = true ) override;
  void get_pose_tool0( double *tf44, bool colmajor ) override;
  unsigned char ik( const double *tf44_tool0, double *q8 ) override;

 public:
  static constexpr uint32_t K = 7;
  std::string info_;
  std::array<Mat4d, K> tfs_cache;
  std::array<Vec6d, K> pns_joint;
  std::array<Vec6d, K> twists_home;
  std::array<Vec6d, K> twists_poe;

  std::array<float, K * 7> qts_cache_f;
  std::array<double, K * 7> qts_cache;
  std::array<double, K * 3> twists_cache;

  Vec3d q_wrist;
  Vec3d q_shoulder;
  Mat4d link_end_fk_home;
  Vec6d q_lo;
  Vec6d q_hi;
  Mat4d tf_link_end_tool0, tf_tool0_link_end;
  Mat4d tf_link_end_tcp;

  quatd quat_end_tool0;
  Vec3d t_end_tool0;
};

}  // namespace ampl