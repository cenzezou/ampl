#ifndef AMPL_GEOMETRY_HPP
#define AMPL_GEOMETRY_HPP

#include <Eigen/Core>
#include <ampl/common.hpp>
#include <map>
#include <vector>
#include <memory>

namespace ampl
{
/*
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ DISTANCE FIELD                                                          │
  └─────────────────────────────────────────────────────────────────────────┘
 */

template <typename REAL, typename UINT>
void distancefield_xyz2occ( const REAL *X, const REAL *Y, const REAL *Z, uint32_t num_xyz, UINT *occ, uint32_t ni,
                            uint32_t nj, uint32_t nk, REAL min_x, REAL min_y, REAL min_z, REAL d_vol );
template <typename REAL, typename UINT>
void distancefield_xyz2occ( const REAL *XYZ, uint32_t num_xyz, UINT *occ, uint32_t ni, uint32_t nj, uint32_t nk,
                            REAL min_x, REAL min_y, REAL min_z, REAL d_vol );

template <typename REAL, typename UINT>
void distancefield_occ2edf( const UINT *occ, REAL *edf, uint32_t ni, uint32_t nj, uint32_t nk, REAL d_vol,
                            uint32_t nb_worker = 4 );

template <typename REAL>
void distancefield_edf2trimesh( const REAL *df, REAL df_offset, uint32_t ni, uint32_t nj, uint32_t nk, REAL min_x,
                                REAL min_y, REAL min_z, REAL d_vol, Eigen::Matrix3X<REAL> &vertices,
                                Eigen::Matrix3X<uint32_t> &faces );

/*
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ TRIMESH                                                                 │
  └─────────────────────────────────────────────────────────────────────────┘
 */
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
bool trimesh_convexhull( const double *xyz, uint32_t nb_xyz, Eigen::Matrix3X<REAL> &vertices,
                         Eigen::Matrix3X<uint32_t> &faces, const double epsilon );
/*
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ PRIMATIVE                                                               │
  └─────────────────────────────────────────────────────────────────────────┘
 */

template <typename REAL>
bool intersect_line3( const REAL *os, const REAL *ns, uint32_t nb_line, REAL *p );

template <typename REAL>
bool intersect_plane3( const REAL *o0, const REAL *n0, const REAL *o1, const REAL *n1, REAL *o, REAL *t );

template <typename REAL>
REAL distance_ray3_tri3( const REAL *o, const REAL *r, const REAL *p0, const REAL *p1, const REAL *p2,
                         const REAL &nearest );

template <typename REAL>
void bounding_aabb_tri3( const REAL *v0, const REAL *v1, const REAL *v2, REAL *vmin, REAL *vmax, REAL epsilon = 1e-5 );

template <typename FT>
FT distance_point3_segment3( const FT *p, const FT *v0, const FT *v1, FT *v_closest, int dim );

template <typename FT>
FT distance_point3_segment3( const FT *p, const FT *v0, const FT *v1, FT *v_closest, int dim, FT *v_lambda );

/*
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ TRANSFORM                                                               │
  └─────────────────────────────────────────────────────────────────────────┘
 */

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

/*
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ SOME EXPLICIT TEMPLATE INITIALIZATION                                   │
  └─────────────────────────────────────────────────────────────────────────┘
 */
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

/*
  ┌─────────────────────────────────────────────────────────────────────────┐
  │ SPATIAL ACCELERATION STRUCTURE                                          │
  └─────────────────────────────────────────────────────────────────────────┘
 */

class OcTreed
{
 public:
  enum EntityType
  {
    Ver = 1,
    Edg,
    Tri
  };
  static std::shared_ptr<OcTreed> create( int num_vertices, const double *vertices, int num_facets, const int *facets,
                                          int num_edges, const int *edges, int num_thread = 1 );
  virtual int intersect_aabb( EntityType type, double *min_box, double *max_box, int *idx_intersect,
                              int max_num_intersect = 64 ) const            = 0;
  virtual int nearest_neighbour( EntityType type, double *position_query, double *distance_closest, double max_dist = 0,
                                 int id_thread = 0 ) const                  = 0;
  virtual int intersect_ray( const double *position_query, const double *direction_unit_query, double *distance_closest,
                             double max_dist = 0, int id_thread = 0 ) const = 0;
  virtual ~OcTreed()                                                        = default;
};

// class BVHd
// {
//  public:
//   static std::shared_ptr<BVHd> create( const double *vertices, int num_vertices, const int *facets, int num_facets );

//   virtual double intersect_ray( const std::array<double, 3> &o, const std::array<double, 3> &d, double near ) = 0;
//   // virtual double IntersectRay( const std::array<double, 3> &o, const std::array<double, 3> &d, double near,
//   //                              int *facet )                                                                  = 0;

//   // virtual double IntersectRay( const std::array<double, 3> &o, const std::array<double, 3> &d, double near, int
//   // *facet,
//   //                              double *facet_normal ) = 0;
//   // virtual std::vector<int> NearestNeighbour( const Eigen::Vector3d &p, double radius ) = 0;
//   virtual ~BVHd() = default;
// };
}  // namespace ampl

#endif