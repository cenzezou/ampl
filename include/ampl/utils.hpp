#ifndef AMPL_UTILS_HPP
#define AMPL_UTILS_HPP

#include <ampl/ampl_config.h>
#include <ampl/common.hpp>
#include <string>
#include <vector>

namespace ampl
{
void version();

// inline auto get_elapsed_ns( const std::chrono::time_point<std::chrono::steady_clock> &start ) -> std::size_t
// {
//   return std::chrono::duration_cast<std::chrono::nanoseconds>( std::chrono::steady_clock::now() - start ).count();
// }
// inline auto get_elapsed_ms( const std::chrono::time_point<std::chrono::steady_clock> &start ) -> std::size_t
// {
//   return std::chrono::duration_cast<std::chrono::microseconds>( std::chrono::steady_clock::now() - start ).count();
// }

// template <typename REAL>
// bool write_distancefield( const std::string &file_path, const REAL *sdf, uint32_t ni, uint32_t nj, uint32_t nk,
//                           REAL min_x, REAL min_y, REAL min_z, REAL d_vol, const REAL *sdf_grad = nullptr );

// template <typename REAL>
// bool read_distancefield( const std::string &file_path, std::vector<REAL> &sdf, uint32_t &ni, uint32_t &nj, uint32_t
// &nk,
//                          REAL &min_x, REAL &min_y, REAL &min_z, REAL &d_vol, std::vector<REAL> *sdf_grad = nullptr );

}  // namespace ampl

#endif