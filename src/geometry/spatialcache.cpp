#include <numeric>  // std::iota
#include <iostream>
#include <algorithm>
#include <cassert>
#include <vector>

#include "octree_unified.h"
#include <ampl/geometry.hpp>
#include <stdio.h>
#include <math.h>
#include <float.h>  // For FLT_MAX

namespace ampl
{

class OcTreed_OT : public OcTreed
{
 private:
  int64_t OctIdx = -1;

 public:
  OcTreed_OT( int num_vertices, const double *vertices, int num_facets, const int *facets, int num_edges,
              const int *edges, int num_thread )
  {
    OctIdx =
        OTNewOcTree( num_vertices, vertices, vertices + 3, num_edges, edges, edges + 2, num_facets, facets, facets + 3,
                     0, NULL, NULL, 0, NULL, NULL, 0, NULL, NULL, 0, NULL, NULL, 0, NULL, NULL, 0, num_thread );
  };
  ~OcTreed_OT() { OTFreeOcTree( OctIdx ); };

  int intersect_aabb( OcTreed::EntityType type, double *min_box, double *max_box, int *idx_intersect,
                      int max_num_intersect ) const
  {
    TypTag tag;
    switch ( type )
    {
      case OcTreed::EntityType::Edg:
        tag = TypTag::OTTypEdg;
        break;
      case OcTreed::EntityType::Tri:
        tag = TypTag::OTTypTri;
        break;
      default:
        tag = TypTag::OTTypVer;
    }
    return OTGetBoundingBox( OctIdx, tag, max_num_intersect, idx_intersect, min_box, max_box, 0 );
  }

  int nearest_neighbour( EntityType type, double *position_query, double *point_projection_closest, double max_dist,
                         int id_thread ) const
  {
    TypTag tag;
    switch ( type )
    {
      case OcTreed::EntityType::Edg:
        tag = TypTag::OTTypEdg;
        break;
      case OcTreed::EntityType::Tri:
        tag = TypTag::OTTypTri;
        break;
      default:
        tag = TypTag::OTTypVer;
    }
    // printf( "%d\n", id_thread );
    return OTGetNearest( OctIdx, tag, position_query, point_projection_closest, max_dist, 0, NULL, id_thread );
  };

  int intersect_ray( const double *position_query, const double *direction_unit_query, double *distance_closest,
                     double max_dist, int id_thread ) const override
  {
    return OTIntersectSurface( OctIdx, const_cast<double *>( position_query ),
                               const_cast<double *>( direction_unit_query ), distance_closest, max_dist, nullptr, NULL,
                               id_thread );
  };
};

std::shared_ptr<OcTreed> OcTreed::create( int num_vertices, const double *vertices, int num_facets, const int *facets,
                                          int num_edges, const int *edges, int num_thread )
{
  OcTreed *mTree = new OcTreed_OT( num_vertices, vertices, num_facets, facets, num_edges, edges, num_thread );  //
  std::shared_ptr<OcTreed> oct{ mTree };
  return oct;
}

// OcTree_TriMeshd::~OcTree_TriMeshd(){};
}  // namespace ampl
#ifdef ABC
namespace ampl
{

using point = std::array<double, 3>;

point operator-( const point &lhs, const point &rhs )
{
  return { lhs[ 0 ] - rhs[ 0 ], lhs[ 1 ] - rhs[ 1 ], lhs[ 2 ] - rhs[ 2 ] };
}

using vector = point;

double dot( const vector &lhs, const vector &rhs )
{
  return std::inner_product( lhs.begin(), lhs.end(), rhs.begin(), 0.f );
}

vector cross( const vector &lhs, const vector &rhs )
{
  vector result;

  vector subtract_me{ lhs[ 2 ] * rhs[ 1 ], lhs[ 0 ] * rhs[ 2 ], lhs[ 1 ] * rhs[ 0 ] };

  result[ 0 ] = ( lhs[ 1 ] * rhs[ 2 ] );
  result[ 0 ] -= subtract_me[ 0 ];
  result[ 1 ] = ( lhs[ 2 ] * rhs[ 0 ] );
  result[ 1 ] -= subtract_me[ 1 ];
  result[ 2 ] = ( lhs[ 0 ] * rhs[ 1 ] );
  result[ 2 ] -= subtract_me[ 2 ];

  return result;
}

class BVHd_Tri : public BVHd
{
 public:
  using point                     = std::array<double, 3>;
  using vector                    = point;
  static constexpr double NEAREST = 0.00001;

  struct Tri
  {
    Tri() : v0( nullptr ), v1( nullptr ), v2( nullptr ){};
    double intersect( const point &origin, const point &direction, double nearest ) const
    {
      const point p0( { v0[ 0 ], v0[ 1 ], v0[ 2 ] } );
      const point p1( { v1[ 0 ], v1[ 1 ], v1[ 2 ] } );
      const point p2( { v2[ 0 ], v2[ 1 ], v2[ 2 ] } );

      vector e1      = p1 - p0;
      vector e2      = p2 - p0;
      vector s1      = cross( direction, e2 );
      double divisor = dot( s1, e1 );
      if ( divisor == 0.f )
      {
        return nearest;
      }

      double inv_divisor = 1.f / divisor;

      // compute barycentric coordinates
      vector d  = origin - p0;
      double b0 = dot( d, s1 ) * inv_divisor;
      if ( b0 < 0.f || b0 > 1.f )
      {
        return nearest;
      }

      vector s2 = cross( d, e1 );
      double b1 = dot( direction, s2 ) * inv_divisor;
      if ( b1 < 0.f || b0 + b1 > 1.f )
      {
        return nearest;
      }

      // compute t
      double t = inv_divisor * dot( e2, s2 );
      if ( t < 0 )
      {
        return nearest;
      }

      return std::min( nearest, t );

      // Triangle tri = { .v0 = { v0[ 0 ], v0[ 1 ], v0[ 2 ] },
      //                  .v1 = { v1[ 0 ], v1[ 1 ], v1[ 2 ] },
      //                  .v2 = { v2[ 0 ], v2[ 1 ], v2[ 2 ] } };

      // // --- Test Case 1: Ray intersects the triangle ---
      // Ray intersecting_ray = {
      //     .origin    = { origin[ 0 ], origin[ 1 ], origin[ 2 ] },
      //     .direction = { direction[ 0 ], direction[ 1 ], direction[ 2 ] }  // Pointing straight down
      // };
      // float dist1 = ray_triangle_distance( intersecting_ray, tri );

      // if ( dist1 < 1e-3 )
      // {
      //   dist1 = point_to_triangle_distance( intersecting_ray.origin, tri );
      // }

      // return distance_ray3_tri3<double>( &origin[ 0 ], &direction[ 0 ], v0, v1, v2, nearest );
    }
    std::array<point, 2> bounding_box() const
    {
      std::array<point, 2> vv;
      bounding_aabb_tri3<double>( v0, v1, v2, &vv[ 0 ][ 0 ], &vv[ 1 ][ 0 ] );
      return vv;
    };
    const double *v0;
    const double *v1;
    const double *v2;
    int id = -1;
  };

  struct Intersection
  {
    Intersection() : hit_tri( nullptr ), hit_time( -1 ){};
    Intersection( double hit_time_, const Tri *mTri )
        : hit_tri( mTri ),
          hit_time( hit_time_ ){
              // printf( "Asdfasdf\n" );
          };
    const Tri *hit_tri;
    double hit_time;
  };

  static constexpr auto functor_intersect = []( const Tri &tri, point o, vector d, const Intersection &nearest_result )
  {
    // compute the time at which the ray defined by o & d hits tri
    // float t = ...

    // get the hit time of the nearest result encountered so far
    double t = tri.intersect( o, d, 0 );
    // double t         = distance_ray3_tri3( &o[ 0 ], &d[ 0 ], tri.v0, tri.v1, tri.v2, nearest_result.hit_time );
    double nearest_t = nearest_result.hit_time;

    // return the nearer of the two results
    return t > nearest_t ? nearest_result : Intersection( t, &tri );
  };

  static constexpr auto functor_result = []( const Intersection &t )
  {
    // the hit_time() function simply returns the .hit_time member of i
    return t.hit_time;
  };

  // double intersect_ray( const std::array<double, 3> &o, const std::array<double, 3> &d, double nearest,
  //                      int *facet ) override
  // {
  //   const Intersection initIntersection( nearest, nullptr );

  //   //  std::array<double, 3> aa( o );
  //   //  std::array<double, 3> dd( d );
  //   // double a = mEH->intersect( o, d, nearest );
  //   //    std::cout << a << std::endl;
  //   Intersection result = mBVH->intersect( o, d, initIntersection, functor_intersect, functor_result );

  //   if ( result.hit_tri == nullptr )
  //     *facet = -1;
  //   else
  //     *facet = (int)( result.hit_tri - mTris->data() /*&my_array[0]*/ );

  //   return result.hit_time;
  // };

  // double IntersectRay( const std::array<double, 3> &o, const std::array<double, 3> &d, double nearest, int *facet,
  //                      double *facet_normal ) override
  // {
  //   const Intersection initIntersection( nearest, nullptr );

  //   //  std::array<double, 3> aa( o );
  //   //  std::array<double, 3> dd( d );
  //   // double a = mEH->intersect( o, d, nearest );
  //   //    std::cout << a << std::endl;
  //   Intersection result = mBVH->intersect( o, d, initIntersection, functor_intersect, functor_result );

  //   if ( result.hit_tri == nullptr )
  //     *facet = -1;
  //   else
  //   {
  //     *facet = (int)( result.hit_tri - mTris->data() /*&my_array[0]*/ );

  //     const double *v0 = result.hit_tri->v0;
  //     const double *v1 = result.hit_tri->v1;
  //     const double *v2 = result.hit_tri->v2;

  //     facet_normal[ 0 ] = -( v0[ 2 ] * v1[ 1 ] ) + v0[ 1 ] * v1[ 2 ] + v0[ 2 ] * v2[ 1 ] - v1[ 2 ] * v2[ 1 ] -
  //                         v0[ 1 ] * v2[ 2 ] + v1[ 1 ] * v2[ 2 ];
  //     facet_normal[ 1 ] = v0[ 2 ] * v1[ 0 ] - v0[ 0 ] * v1[ 2 ] - v0[ 2 ] * v2[ 0 ] + v1[ 2 ] * v2[ 0 ] +
  //                         v0[ 0 ] * v2[ 2 ] - v1[ 0 ] * v2[ 2 ];
  //     facet_normal[ 2 ] = -( v0[ 1 ] * v1[ 0 ] ) + v0[ 0 ] * v1[ 1 ] + v0[ 1 ] * v2[ 0 ] - v1[ 1 ] * v2[ 0 ] -
  //                         v0[ 0 ] * v2[ 1 ] + v1[ 0 ] * v2[ 1 ];

  //     double len_d = std::sqrt( facet_normal[ 0 ] * facet_normal[ 0 ] + facet_normal[ 1 ] * facet_normal[ 1 ] +
  //                               facet_normal[ 2 ] * facet_normal[ 2 ] );
  //     facet_normal[ 0 ] /= len_d;
  //     facet_normal[ 1 ] /= len_d;
  //     facet_normal[ 2 ] /= len_d;
  //   }

  //   return result.hit_time;
  // };

  double intersect_ray( const std::array<double, 3> &o, const std::array<double, 3> &d, double nearest ) override
  {
    const Intersection initIntersection( nearest, nullptr );

    //  std::array<double, 3> aa( o );
    //  std::array<double, 3> dd( d );
    // double a = mEH->intersect( o, d, nearest );
    //    std::cout << a << std::endl;
    Intersection result = mBVH->intersect( o, d, initIntersection, functor_intersect, functor_result );
    return result.hit_time;
  };

  BVHd_Tri( const double *vertices, int num_vertices, const int *facets, int num_facets )
  {
    // std::vector<Tri> tris( num_facets );
    // printf( "here\n" );
    mTris = new std::vector<Tri>( num_facets );
    // mTris->resize( num_facets );
    // printf( "here\n" );
    for ( int f = 0; f < num_facets; ++f )
    {
      // for ( int i = 0; i < 3; ++i )

      ( *mTris )[ f ].v0 = &vertices[ 3 * facets[ 3 * f + 0 ] ];
      ( *mTris )[ f ].v1 = &vertices[ 3 * facets[ 3 * f + 1 ] ];
      ( *mTris )[ f ].v2 = &vertices[ 3 * facets[ 3 * f + 2 ] ];
      //( *mTris )[ f ]    = f;
    }

    mBVH = new bounding_box_hierarchy<Tri>( *mTris );
    // mEH  = new exhaustive_searcher<Tri>( *mTris );

    // bvh->make_tree()
    // bounding_box_hierarchy<triangle> bbh(triangles);
  };

  ~BVHd_Tri()
  {
    for ( size_t i = 0; i < mTris->size(); i++ )
    {
      ( *mTris )[ i ].v0 = nullptr;
      ( *mTris )[ i ].v1 = nullptr;
      ( *mTris )[ i ].v2 = nullptr;
    }

    if ( mTris ) delete mTris;
    if ( mBVH ) delete mBVH;
  }

  bounding_box_hierarchy<Tri> *mBVH = nullptr;
  // exhaustive_searcher<Tri> *mEH     = nullptr;
  std::vector<Tri> *mTris = nullptr;
  // public:
};

std::shared_ptr<BVHd> BVHd::create( const double *vertices, int num_vertices, const int *facets, int num_facets )
{
  BVHd *mTree = new BVHd_Tri( vertices, num_vertices, facets, num_facets );
  //
  std::shared_ptr<BVHd> bvh{ mTree };

  return bvh;

  // Eigen::Map<const Eigen::Matrix<double, 3, -1>> V( vertices, 3, num_vertices );
  // Eigen::Map<const Eigen::Matrix<int, 2, -1>> F( facets, 3, num_facets );
}

}  // namespace ampl

#endif