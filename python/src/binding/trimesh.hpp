#ifndef AMPL_BINDING_TRIMESH_HPP
#define AMPL_BINDING_TRIMESH_HPP

#include <iostream>
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>
#include <ampl/ampl.hpp>
#include <nanobind/trampoline.h>
namespace nb = nanobind;
using namespace nb::literals;

namespace ampl::binding
{

inline void init_trimesh( nanobind::module_ &pymodule )
{
  pymodule
      .def(
          "trimesh_vhacd",
          []( const nb::ndarray<double, nb::shape<-1, 3>, nb::device::cpu> &vertices,   // NOAQ
              const nb::ndarray<uint32_t, nb::shape<-1, 3>, nb::device::cpu> &indices,  // NOAQ

              uint32_t maxConvexHulls,                  //
              uint32_t resolution,                      //
              double minimumVolumePercentErrorAllowed,  //
              uint32_t maxRecursionDepth,               //
              bool shrinkWrap,                          //
              std::string fillMode,                     //
              uint32_t maxNumVerticesPerCH,             //
              bool asyncACD,                            //
              uint32_t minEdgeLength,                   //
              bool findBestPlane

              ) -> nb::object
          {
            std::vector<std::vector<double>> Vs;
            std::vector<std::vector<uint32_t>> Fs;

            ampl::trimesh_vhacd( vertices.data(), vertices.shape( 0 ), indices.data(), indices.shape( 0 ), Vs, Fs,

                                 maxConvexHulls, resolution, minimumVolumePercentErrorAllowed, maxRecursionDepth,
                                 shrinkWrap, fillMode, maxNumVerticesPerCH, asyncACD, minEdgeLength, findBestPlane );

            std::pair<std::vector<nb::ndarray<nb::numpy, double, nb::shape<-1, 3>>>,
                      std::vector<nb::ndarray<nb::numpy, uint32_t, nb::shape<-1, 3>>>>
                out;
            out.first.reserve( 1 );
            out.second.reserve( 1 );
            // std::vector<nb::ndarray<nb::numpy, double, nb::shape<-1, 3>>> sV;
            // std::vector<nb::ndarray<nb::numpy, uint32_t, nb::shape<-1, 3>>>
            // sF;
            for ( auto &V : Vs )
              out.first.push_back(
                  nb::ndarray<nb::numpy, double, nb::shape<-1, 3>>( V.data(), { V.size() / 3, 3 }, nb::handle() ) );
            for ( auto &F : Fs )
              out.second.push_back(
                  nb::ndarray<nb::numpy, uint32_t, nb::shape<-1, 3>>( F.data(), { F.size() / 3, 3 }, nb::handle() ) );

            return nb::cast( out, nb::rv_policy::copy );
          },

          "vertices"_a,                                    //
          "indices"_a,                                     //
          "maxConvexHulls"_a                   = 64,       //
          "resolution"_a                       = 400000,   //
          "minimumVolumePercentErrorAllowed"_a = 1.0,      //
          "maxRecursionDepth"_a                = 10,       //
          "shrinkWrap"_a                       = true,     //
          "fillMode"_a                         = "flood",  //
          "maxNumVerticesPerCH"_a              = 64,       //
          "asyncACD"_a                         = true,     //
          "minEdgeLength"_a                    = 2,        //
          "findBestPlane"_a                    = false,    //
          "This function wraps around vhacd." )
      .def(
          "trimesh_sampler_barycentricysplit",
          []( const nb::ndarray<float, nb::shape<-1, 3>, nb::device::cpu> &vertices,    // NOAQ
              const nb::ndarray<uint32_t, nb::shape<-1, 3>, nb::device::cpu> &indices,  // NOAQ
              const uint32_t nb_samples_expect, const float sample_distance_expect )
          {
            Eigen::Matrix3Xf xyz = ampl::trimesh_sampler_barycentricysplit<float>(
                vertices.data(), vertices.shape( 0 ), indices.data(), indices.shape( 0 ), nb_samples_expect,
                sample_distance_expect );

            float *data = new float[ 3 * xyz.cols() ];

            memcpy( data, xyz.data(), 3 * xyz.cols() * sizeof( float ) );

            nb::capsule owner( data, []( void *p ) noexcept { delete[]( float * ) p; } );

            return nb::ndarray<nb::numpy, float, nb::ndim<2>>(
                /* data = */ data,
                /* shape = */ { xyz.cols(), 3 },
                /* owner = */ owner );

            // pamp:: enormal<float>(vertices.data(), vertices.shape(0),
            //                                 indices.data(), indices.shape(0),
            //                                 facenormals.data()

            // );
          },

          "vertices"_a,                // NOAQ,
          "indices"_a,                 // NOAQ,
          "nb_samples_expect"_a,       // NOAQ,
          "sample_distance_expect"_a,  // NOAQ,
          "This function samples pointcloud from a mesh by barycentric face-splitting." )
      .def(
          "trimesh_raycast",
          []( const nb::ndarray<double, nb::shape<-1, 3>, nb::device::cpu> &vertices,        // NOAQ
              const nb::ndarray<uint32_t, nb::shape<-1, 3>, nb::device::cpu> &indices,       // NOAQ
              const nb::ndarray<double, nb::shape<-1, 3>, nb::device::cpu> &origins_ray,     // NOAQ
              const nb::ndarray<double, nb::shape<-1, 3>, nb::device::cpu> &directions_ray,  // NOAQ
              bool ret_hit_indics,                                                           // NOAQ
              uint32_t nb_worker                                                             // NOAQ
          )
          {
            const double *ptr_dir  = directions_ray.data();
            const double *ptr_o    = origins_ray.data();
            constexpr double MAX_D = 10000.0;

            double *distance = new double[ directions_ray.shape( 0 ) ];
            auto tree        = ampl::OcTreed::create( vertices.shape( 0 ), vertices.data(), indices.shape( 0 ),
                                                      reinterpret_cast<const int *>( indices.data() ), 0, NULL, nb_worker );
            for ( uint32_t i = 0; i < directions_ray.shape( 0 ); i++ )
            {
              tree->intersect_ray( ptr_o, ptr_dir, &distance[ i ], MAX_D );
              ptr_dir += 3;
              ptr_o += 3;
            }

            // auto tree = ampl::BVHd::create( vertices.data(), vertices.shape( 0 ),
            //                                 reinterpret_cast<const int *>( indices.data() ), indices.shape( 0 ) );

            // for ( uint32_t i = 0; i < directions_ray.shape( 0 ); i++ )
            // {
            //   double dist =
            //       tree->intersect_ray( std::array<double, 3>( { ptr_o[ 0 ], ptr_o[ 1 ], ptr_o[ 2 ] } ),
            //                            std::array<double, 3>( { ptr_dir[ 0 ], ptr_dir[ 1 ], ptr_dir[ 2 ] } ), MAX_D
            //                            );
            //   distance[ i ] = dist;
            //   ptr_dir += 3;
            //   ptr_o += 3;
            // }

            nb::capsule owner_distance( distance, []( void *p ) noexcept { delete[]( double * ) p; } );
            return std::tuple<nb::ndarray<nb::numpy, double>, nb::ndarray<nb::numpy, uint32_t>>(
                nb::ndarray<nb::numpy, double>( distance, { directions_ray.shape( 0 ) }, owner_distance ),
                nb::ndarray<nb::numpy, uint32_t>() );
          },
          "vertices"_a,                // NOAQ
          "indices"_a,                 // NOAQ,
          "origins_ray"_a,             // NOAQ
          "directions_ray"_a,          // NOAQ,
          "ret_hit_indice"_a = false,  // NOAQ,
          "nb_worker"_a      = 1,
          "This function performs distance queries of a point cloud against a "
          "distance field" )

      ;
}
}  // namespace ampl::binding
#endif