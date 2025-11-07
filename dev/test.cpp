#include <Eigen/Dense>

#include <ampl/ampl.hpp>
#include <iostream>
int main() {

  float ns[6] = {1, 0, 0, 0, 1, 0};
  float os[6] = {0, 0, 0, 0, 0, 0};

  Eigen::Vector3f p;

  ampl::intersectf_line3(os, ns, 2, p.data());
  std::unique_ptr<ampl::ArmBase> m;
  m = ampl::ArmBase::create(std::string("abb_irb6700_150_320"),
                            ampl::ArmType::Industrial6, 6);

  std::cout << m->info() << std::endl;

  Eigen::Matrix<double, 7, 6> qts_link;
  m->initialize_preset("abb_irb6700_150_320");
  Eigen::Vector<double, 6> q;
  q << 0, 0, 0, 0, 0, 0;
  m->fk(q.data(), qts_link.data());
  std::cout << qts_link << std::endl;

  // Eigen::Matrix3d R;
  // Eigen::Vector3d w;
  // w << 0, 0, 1;

  // ampl::so3d_up(w, R);
  // std::cout << R << std::endl;
  return 0;
}