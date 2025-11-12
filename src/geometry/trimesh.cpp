
#include <Eigen/Dense>
#include <ampl/geometry.hpp>
#include <math.h>
#include <vector>
#define ENABLE_VHACD_IMPLEMENTATION 1
#include "vhacd.h"
#include "magic.hpp"
#include "quickhull.hpp"
namespace ampl
{

template bool trimesh_convexhull<float>( const double *xyz, uint32_t nb_xyz, Eigen::Matrix3X<float> &vertices,
                                         Eigen::Matrix3X<uint32_t> &faces, const double epsilon );

template bool trimesh_convexhull<double>( const double *xyz, uint32_t nb_xyz, Eigen::Matrix3X<double> &vertices,
                                          Eigen::Matrix3X<uint32_t> &faces, const double epsilon );

template Eigen::Matrix<float, 3, -1> trimesh_sampler_barycentricysplit<float>( const float *vertices, uint32_t nb_v,
                                                                               const uint32_t *facets, uint32_t nb_f,
                                                                               const uint32_t &nb_p_expect,
                                                                               const float &sampling_length_manual );
template Eigen::Matrix<double, 3, -1> trimesh_sampler_barycentricysplit<double>( const double *vertices, uint32_t nb_v,
                                                                                 const uint32_t *facets, uint32_t nb_f,
                                                                                 const uint32_t &nb_p_expect,
                                                                                 const double &sampling_length_manual );

template void distancefield_edf2trimesh<float>( const float *df, float df_offset, uint32_t ni, uint32_t nj, uint32_t nk,
                                                float min_x, float min_y, float min_z, float d_vol,
                                                Eigen::Matrix3X<float> &vertices, Eigen::Matrix3X<uint32_t> &faces );

template void distancefield_edf2trimesh<double>( const double *df, double df_offset, uint32_t ni, uint32_t nj,
                                                 uint32_t nk, double min_x, double min_y, double min_z, double d_vol,
                                                 Eigen::Matrix3X<double> &vertices, Eigen::Matrix3X<uint32_t> &faces );

template <typename REAL>
Eigen::Matrix<REAL, 3, -1> trimesh_sampler_barycentricysplit( const REAL *vertices, uint32_t nb_v,
                                                              const uint32_t *facets, uint32_t nb_f,
                                                              const uint32_t &nb_p_expect,
                                                              const REAL &sampling_length_manual )
{
  using Index       = uint32_t;
  using Scalar      = REAL;
  using VertexType  = Eigen::Matrix<Scalar, 3, 1>;
  using RowVectorS  = Eigen::Matrix<Scalar, 1, Eigen::Dynamic>;
  using RowVector3S = Eigen::Matrix<Scalar, 1, 3>;
  using RowVectorI  = Eigen::Matrix<Index, 1, Eigen::Dynamic>;
  using BoundingBox = Eigen::AlignedBox<Scalar, 3>;
  using Triangle    = Eigen::Matrix<Scalar, 3, 6>;

  auto facet_ij         = []( const Index *arr, const Index &i, const Index &j ) { return (Index)arr[ i * 3 + j ]; };
  auto get_triangle_pos = []( const Triangle &tri, const Index vert_offset ) -> RowVectorS
  { return tri.row( vert_offset ).template head<3>(); };
  auto get_triangle_bary = []( const Triangle &tri, const Index vert_offset ) -> RowVectorS
  { return tri.row( vert_offset ).template tail<3>(); };
  auto set_triangle_pos = []( Triangle &tri, const Index vert_offset, const RowVectorS &v ) -> void
  { tri.row( vert_offset ).template head<3>() = v; };
  auto set_triangle_bary = []( Triangle &tri, const Index vert_offset, const RowVectorS &v ) -> void
  { tri.row( vert_offset ).template tail<3>() = v; };

  Scalar sampling_length = sampling_length_manual;

  BoundingBox bounding_box;

  if ( sampling_length_manual > 0 )
  {
    sampling_length = sampling_length_manual;
    for ( Index i = 0; i < nb_v; i++ )
      bounding_box.extend( VertexType( vertices[ 3 * i + 0 ], vertices[ 3 * i + 1 ], vertices[ 3 * i + 2 ] ) );
  }
  else
  {
    // std::vector<Scalar> area( num_facets, 0 );
    Scalar total_mesh_area = 0;
    for ( Index i = 0; i < nb_f; i++ )
    {
      Index f0 = facets[ 3 * i + 0 ];
      Index f1 = facets[ 3 * i + 1 ];
      Index f2 = facets[ 3 * i + 2 ];
      VertexType v0( vertices[ 3 * f0 + 0 ], vertices[ 3 * f0 + 1 ], vertices[ 3 * f0 + 2 ] );
      VertexType v1( vertices[ 3 * f1 + 0 ], vertices[ 3 * f1 + 1 ], vertices[ 3 * f1 + 2 ] );
      VertexType v2( vertices[ 3 * f2 + 0 ], vertices[ 3 * f2 + 1 ], vertices[ 3 * f2 + 2 ] );
      VertexType a = ( v1 - v0 ).cross( v2 - v1 );
      total_mesh_area += a.norm();
      bounding_box.extend( v0 );
      bounding_box.extend( v1 );
      bounding_box.extend( v2 );
      // std::cout << f0 << "\t" << f1 << std::endl;
      // std::cout << v0.transpose() << "\t" << v1.transpose() << std::endl;
    }
    sampling_length = 1.5 * std::sqrt( ( total_mesh_area / ( nb_p_expect * M_PI ) ) );
  }
  printf_debug( "sampling_length = %f \n", sampling_length );

  std::list<VertexType> sample_positions;
  std::list<Index> sample_facet_ids;
  std::list<RowVector3S> sample_barycentrics;
  std::queue<Triangle> sub_triangles;
  Index num_samples = 0;

  const RowVectorS grid_lens = bounding_box.sizes();
  const RowVectorI grid_dims = ( grid_lens / sampling_length ).array().ceil().template cast<Index>();
  const Index num_grid_cells = grid_dims.prod();

  auto get_flattend_index = [ &grid_dims ]( const RowVectorI &index_nd ) -> Index
  { return index_nd( 2 ) * grid_dims( 1 ) * grid_dims( 0 ) + index_nd( 1 ) * grid_dims( 0 ) + index_nd( 0 ); };
  std::unordered_set<Index> is_grid_cell_marked_backend( num_grid_cells );
  auto is_grid_cell_marked = [ &is_grid_cell_marked_backend ]( const Index cell_index )
  { return is_grid_cell_marked_backend.count( cell_index ) != 0; };
  //
  auto mark_grid_cell = [ &is_grid_cell_marked_backend ]( const Index cell_index )
  { is_grid_cell_marked_backend.insert( cell_index ); };

  for ( Index facet_id = 0; facet_id < nb_f; facet_id++ )
  {
    sub_triangles         = std::queue<Triangle>();
    Triangle mother_facet = Triangle::Zero();

    Index vid = facet_ij( facets, facet_id, (Index)0 ) * 3;
    RowVector3S v;
    v << vertices[ vid ], vertices[ vid + 1 ], vertices[ vid + 2 ];
    set_triangle_pos( mother_facet, 0, v );

    vid = facet_ij( facets, facet_id, (Index)1 ) * 3;
    v << vertices[ vid ], vertices[ vid + 1 ], vertices[ vid + 2 ];
    set_triangle_pos( mother_facet, 1, v );

    vid = facet_ij( facets, facet_id, (Index)2 ) * 3;
    v << vertices[ vid ], vertices[ vid + 1 ], vertices[ vid + 2 ];
    set_triangle_pos( mother_facet, 2, v );

    set_triangle_bary( mother_facet, 0, RowVector3S( 1, 0, 0 ) );
    set_triangle_bary( mother_facet, 1, RowVector3S( 0, 1, 0 ) );
    set_triangle_bary( mother_facet, 2, RowVector3S( 0, 0, 1 ) );

    // // Push it to the queue
    sub_triangles.push( mother_facet );

    while ( !sub_triangles.empty() )
    {
      const Triangle current_facet = sub_triangles.front();
      sub_triangles.pop();

      // Find the longest edge in the triangle
      Scalar longest_edge_length = -std::numeric_limits<Scalar>::max();
      Index longest_edge_offset  = std::numeric_limits<Index>::max();
      for ( Index edge_offset = 0; edge_offset < 3; edge_offset++ )
      {
        const auto edge_offset_p1 = ( edge_offset + 1 ) % 3;
        const Scalar edge_length  = ( get_triangle_pos( current_facet, edge_offset )  //
                                     - get_triangle_pos( current_facet, edge_offset_p1 ) )
                                       .norm();
        if ( edge_length > longest_edge_length )
        {
          longest_edge_length = edge_length;
          longest_edge_offset = edge_offset;
        }
      }
      if ( longest_edge_length > 2 * sampling_length )
      {
        Triangle f1 = Triangle::Zero();
        Triangle f2 = Triangle::Zero();
        //
        f1.row( 0 ) =
            ( current_facet.row( longest_edge_offset ) + current_facet.row( ( longest_edge_offset + 1 ) % 3 ) ) / 2;
        f1.row( 1 ) = current_facet.row( longest_edge_offset );
        f1.row( 2 ) = current_facet.row( ( longest_edge_offset + 2 ) % 3 );
        //
        f2.row( 0 ) = f1.row( 0 );
        f2.row( 1 ) = current_facet.row( ( longest_edge_offset + 1 ) % 3 );
        f2.row( 2 ) = current_facet.row( ( longest_edge_offset + 2 ) % 3 );
        //
        sub_triangles.push( f1 );
        sub_triangles.push( f2 );
      }

      else
      {  // Otherwise, sample the centroid
        const RowVector3S point_barycentric =
            ( get_triangle_bary( current_facet, 0 ) + get_triangle_bary( current_facet, 1 ) +
              get_triangle_bary( current_facet, 2 ) ) /
            3.;
        const VertexType point_position =
            ( get_triangle_pos( current_facet, 0 ) + get_triangle_pos( current_facet, 1 ) +
              get_triangle_pos( current_facet, 2 ) ) /
            3.;

        const RowVectorI grid_cell =
            ( ( point_position - bounding_box.min() ) / sampling_length ).array().floor().template cast<Index>();
        const Index grid_cell_id = get_flattend_index( grid_cell );
        if ( !is_grid_cell_marked( grid_cell_id ) )
        {
          mark_grid_cell( grid_cell_id );
          // record sample facet
          sample_facet_ids.push_back( facet_id );
          // record barycetric (in mother facet)
          sample_barycentrics.push_back( point_barycentric );
          // record the position
          sample_positions.push_back( point_position );
          //
          ++num_samples;
        }  // end of grid check
      }    // end of size check
    }
  }
  Eigen::Matrix<REAL, 3, -1> out;
  out.resize( 3, sample_positions.size() );

  // if ( sample_facet_ids_out )
  // {
  //   sample_facet_ids_out->clear();
  //   sample_facet_ids_out->reserve( sample_facet_ids.size() );
  //   typename std::list<Index>::iterator it;
  //   // size_t i = 0;
  //   for ( it = sample_facet_ids.begin(); it != sample_facet_ids.end(); ++it )
  //   {
  //     sample_facet_ids_out->push_back( *it );
  //     // out.col( i ) = *it;
  //     // i++;
  //   }
  // }
  // typename std::list<Index>::iterator it;
  typename std::list<VertexType>::iterator it;
  size_t i = 0;
  for ( it = sample_positions.begin(); it != sample_positions.end(); ++it )
  {
    out.col( i ) = *it;
    i++;
  }

  // for ( size_t i = 0; i < sample_positions.size(); i++ )
  //   ;

  return out;
};

void trimesh_vhacd( const double *vertices, uint32_t num_vertices, const uint32_t *facets, uint32_t num_facets,
                    std::vector<std::vector<double>> &Vs, std::vector<std::vector<uint32_t>> &Fs,
                    uint32_t maxConvexHulls, uint32_t resolution, double minimumVolumePercentErrorAllowed,
                    uint32_t maxRecursionDepth, bool shrinkWrap, std::string fillMode, uint32_t maxNumVerticesPerCH,
                    bool asyncACD, uint32_t minEdgeLength, bool findBestPlane )
{
  double *ptr_points          = (double *)vertices;
  uint32_t *ptr_faces         = (uint32_t *)facets;
  size_t num_points           = num_vertices;
  size_t num_faces            = num_facets;
  size_t num_triangle_indices = num_faces * 3;
  std::vector<uint32_t> triangles( num_triangle_indices );
  for ( uint32_t i = 0; i < num_faces; i++ )
  {
    triangles[ 3 * i ]     = ptr_faces[ 3 * i + 0 ];
    triangles[ 3 * i + 1 ] = ptr_faces[ 3 * i + 1 ];
    triangles[ 3 * i + 2 ] = ptr_faces[ 3 * i + 2 ];
  }

  VHACD::IVHACD::Parameters p;
  p.m_maxConvexHulls                   = maxConvexHulls;
  p.m_resolution                       = resolution;
  p.m_minimumVolumePercentErrorAllowed = minimumVolumePercentErrorAllowed;
  p.m_maxRecursionDepth                = maxRecursionDepth;
  p.m_shrinkWrap                       = shrinkWrap;
  p.m_maxNumVerticesPerCH              = maxNumVerticesPerCH;
  p.m_asyncACD                         = asyncACD;
  p.m_minEdgeLength                    = minEdgeLength;
  p.m_findBestPlane                    = findBestPlane;
  if ( fillMode == "flood" )
  {
    p.m_fillMode = VHACD::FillMode::FLOOD_FILL;
  }
  else if ( fillMode == "raycast" )
  {
    p.m_fillMode = VHACD::FillMode::RAYCAST_FILL;
  }
  else if ( fillMode == "surface" )
  {
    p.m_fillMode = VHACD::FillMode::SURFACE_ONLY;
  }
  else
  {
    printf(
        "Invalid fill mode, only valid options are 'flood', 'raycast', and "
        "'surface'\n" );
  }

#if VHACD_DISABLE_THREADING
  VHACD::IVHACD *iface = VHACD::CreateVHACD();
#else
  VHACD::IVHACD *iface = p.m_asyncACD ? VHACD::CreateVHACD_ASYNC() : VHACD::CreateVHACD();
#endif

  /// The main computation ///////////////////////////////////////////////
  iface->Compute( ptr_points, num_points, triangles.data(), num_faces, p );

  while ( !iface->IsReady() )
  {
    std::this_thread::sleep_for( std::chrono::nanoseconds( 10000 ) );  // s
  }

  // std::vector<std::vector<double>> Vs;
  // std::vector<std::vector<uint32_t>> Fs;

  const int nConvexHulls = iface->GetNConvexHulls();
  Vs.reserve( nConvexHulls );
  Fs.reserve( nConvexHulls );

  if ( nConvexHulls )
  {
    // Exporting Convex Decomposition results of convex hulls

    for ( uint32_t i = 0; i < iface->GetNConvexHulls(); i++ )
    {
      VHACD::IVHACD::ConvexHull ch;
      iface->GetConvexHull( i, ch );

      /*  allocate the output buffers */
      std::vector<double> res_vertices = std::vector<double>( ch.m_points.size() * 3 );
      std::vector<uint32_t> res_faces  = std::vector<uint32_t>( ch.m_triangles.size() * 3 );

      // py::buffer_info buf_res_vertices = res_vertices.request();
      // py::buffer_info buf_res_faces    = res_faces.request();

      // double *ptr_res_vertices = (double *)buf_res_vertices.ptr;
      // uint32_t *ptr_res_faces  = (uint32_t *)buf_res_faces.ptr;

      for ( uint32_t j = 0; j < ch.m_points.size(); j++ )
      {
        const VHACD::Vertex &pos  = ch.m_points[ j ];
        res_vertices[ 3 * j ]     = pos.mX;
        res_vertices[ 3 * j + 1 ] = pos.mY;
        res_vertices[ 3 * j + 2 ] = pos.mZ;
      }

      //      const uint32_t num_v = 3;
      for ( uint32_t j = 0; j < ch.m_triangles.size(); j++ )
      {
        res_faces[ 3 * j ]     = ch.m_triangles[ j ].mI0;
        res_faces[ 3 * j + 1 ] = ch.m_triangles[ j ].mI1;
        res_faces[ 3 * j + 2 ] = ch.m_triangles[ j ].mI2;
      }

      // Reshape the vertices array to be (n, 3)

      // Push on our list
      Vs.emplace_back( res_vertices );
      Fs.emplace_back( res_faces );
      // res.emplace_back( std::move( res_vertices ), std::move( res_faces ) );
    }
  }
  return;
};
#undef ENABLE_VHACD_IMPLEMENTATION

template <typename REAL>
void distancefield_edf2trimesh( const REAL *df, REAL df_offset, uint32_t ni, uint32_t nj, uint32_t nk, REAL min_x,
                                REAL min_y, REAL min_z, REAL d_vol, Eigen::Matrix3X<REAL> &vertices,
                                Eigen::Matrix3X<uint32_t> &faces )
{
  std::vector<marching_cube::mc_vec3r> vs_mc;
  std::vector<marching_cube::mc_vec3ui> fs_mc;

  marching_cube::marching_cube<REAL>( df, df_offset, ni, nj, nk, vs_mc, fs_mc );

  vertices.resize( 3, vs_mc.size() );
  faces.resize( 3, fs_mc.size() );

  for ( Eigen::Index i = 0; i < faces.cols(); i++ )
  {
    faces.col( i ) = fs_mc[ i ].cast<uint32_t>();
  }
  for ( Eigen::Index i = 0; i < vertices.cols(); i++ )
  {
    vertices.col( i ) = vs_mc[ i ].cast<REAL>();
  }
  vertices *= d_vol;
  vertices.colwise() += Eigen::Vector3<REAL>( min_x, min_y, min_z );
};

template <typename REAL>
bool trimesh_convexhull( const double *xyz, uint32_t nb_xyz, Eigen::Matrix3X<REAL> &vertices,
                         Eigen::Matrix3X<uint32_t> &faces, const double epsilon )
{
  quickhull::QuickHull<double> qh;
  const double qhullEps = std::min( epsilon, quickhull::defaultEps<double>() );
  auto hull             = qh.getConvexHull( xyz, nb_xyz, false, false, qhullEps );
  auto ids              = hull.getIndexBuffer();
  auto vxs              = hull.getVertexBuffer();

  if ( ids.empty() ) return false;

  faces.resize( 3, ids.size() / 3 );
  vertices.resize( 3, vxs.size() );

  for ( auto i = 0; i < faces.size(); i++ ) faces.col( i ) << ids[ 3 * i + 0 ], ids[ 3 * i + 1 ], ids[ 3 * i + 2 ];

  for ( auto i = 0; i < vertices.size(); i++ ) vertices.col( i ) << vxs[ i ].x, vxs[ i ].y, vxs[ i ].z;
  printf_debug( "(v,f) = (%lu,%lu)\n", vertices.size(), faces.size() );
  // if (hull.)
  // std::cout << hull.getIndexBuffer().size() << std::endl;
  return true;
};

}  // namespace ampl