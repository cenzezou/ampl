#ifndef AMPL_GEOMETRY_HPP
#define AMPL_GEOMETRY_HPP

#include <Eigen/Core>
#include <ampl/ampl_config.h>
#include <map>
#include <vector>

namespace ampl
{

void trimesh_vhacd( const double *vertices, uint32_t num_vertices, const uint32_t *facets, uint32_t num_facets,
                    std::vector<std::vector<double>> &vertices_ch, std::vector<std::vector<uint32_t>> &faces_ch,
                    uint32_t maxConvexHulls, uint32_t resolution, double minimumVolumePercentErrorAllowed,
                    uint32_t maxRecursionDepth, bool shrinkWrap, std::string fillMode, uint32_t maxNumVerticesPerCH,
                    bool asyncACD, uint32_t minEdgeLength, bool findBestPlane );

template <typename REAL>
Eigen::Matrix<REAL, 3, -1> trimesh_sampler_barycentricysplit( const REAL *vertices, uint32_t nb_v,
                                                              const uint32_t *facets, uint32_t nb_f,
                                                              const uint32_t &nb_p_expect,
                                                              const REAL &sampling_length_manual );

template <typename REAL>
bool intersect_line3( const REAL *os, const REAL *ns, uint32_t nb_line, REAL *p );

template <typename REAL>
bool intersect_plane3( const REAL *o0, const REAL *n0, const REAL *o1, const REAL *n1, REAL *o, REAL *t );

template <typename REAL>
void convert_qt_to_tf( const REAL *qt7, REAL *tf44, bool colmajor = true, bool q_then_t = true );
template <typename REAL>
void convert_tf_to_qt( const REAL *tf44, REAL *qt7, bool colmajor = true, bool q_then_t = true );
template <typename REAL>
void so3_up( const Eigen::Vector3<REAL> &w, Eigen::Ref<Eigen::Matrix3<REAL>> so3 );
template <typename REAL>
void so3_up2( const Eigen::Vector3<REAL> &w, Eigen::Matrix3<REAL> &so3 );
template <typename REAL>
void so3_down( Eigen::Vector3<REAL> &w, const Eigen::Matrix3<REAL> &so3 );
template <typename REAL>
void SO3_downlog( Eigen::Vector3<REAL> &w, const Eigen::Matrix3<REAL> &SO3 );
template <typename REAL>
void so3_upexp( const Eigen::Vector3<REAL> &w, Eigen::Ref<Eigen::Matrix3<REAL>> SO3 );
template <typename REAL>
void SO3_Adj( const Eigen::Matrix3<REAL> &SO3 );
template <typename REAL>
void se3_upexp( const Eigen::Vector<REAL, 6> &x, Eigen::Ref<Eigen::Matrix4<REAL>> SE3 );
template <typename REAL>
void SE3_Adj( const Eigen::Matrix4<REAL> &SE3, Eigen::Matrix<REAL, 6, 6> &A );
template <typename REAL>
void SE3_inv( const Eigen::Matrix4<REAL> &SE3, Eigen::Matrix4<REAL> &inv );

template <typename REAL>
void transform_xyz( const REAL *rw, const REAL *t, const REAL *xyz, uint32_t nb_xyz, REAL *xyz_dst );

auto constexpr intersectd_line3  = &intersect_line3<double>;
auto constexpr intersectf_line3  = &intersect_line3<float>;
auto constexpr intersectd_plane3 = &intersect_plane3<double>;
auto constexpr intersectf_plane3 = &intersect_plane3<float>;
auto constexpr convertd_qt_to_tf = &convert_qt_to_tf<double>;
auto constexpr convertf_qt_to_tf = &convert_qt_to_tf<float>;
auto constexpr convertd_tf_to_qt = &convert_tf_to_qt<double>;
auto constexpr convertf_tf_to_qt = &convert_tf_to_qt<float>;

auto constexpr so3d_up = &so3_up<double>;
auto constexpr so3f_up = &so3_up<float>;
}  // namespace ampl

#endif