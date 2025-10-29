#include <Eigen/Dense>

#include <ampl/ampl.hpp>
#include <iostream>
int main() {

  float ns[6] = {1, 0, 0, 0, 1, 0};
  float os[6] = {0, 0, 0, 0, 0, 0};

  Eigen::Vector3f p;

  ampl::intersectf_line3(os, ns, 2, p.data());
  auto m = ampl::ArmBase::create(std::string("abc"), ampl::ArmType::Industrial6,
                                 6, nullptr);

  std::cout << m->info() << std::endl;

  std::cout << p << std::endl;

  Eigen::Matrix3d R;
  Eigen::Vector3d w;
  w << 0, 0, 1;

  ampl::so3d_up(w, R);
  std::cout << R << std::endl;
  return 0;
}