#ifndef AMPL_ALGO_SIMD_HPP
#define AMPL_ALGO_SIMD_HPP
#include <immintrin.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#ifdef _WIN32
#include <intrin.h>
#define AMPL_ALIGNED(n) __declspec(align(n))
#define AMPL_FORCEINLINE __forceinline
#else
#define AMPL_ALIGNED(n) __attribute__((aligned(n)))
#define AMPL_FORCEINLINE inline __attribute__((always_inline))
#endif

// #ifdef __AVX2__
// #define AMPL_SIMDV 256
// #define AMPL_SIMDW_F 8
// #else
#define AMPL_SIMDV 128
#define AMPL_SIMDW_F 4
// #endif

namespace ampl {
typedef __m128 VecfSimd;
typedef __m128i VecuSimd;

AMPL_FORCEINLINE void print_128(__m128 &a) {
  float output[4];
  _mm_storeu_ps(output, a);

  printf("%+08.8f %+08.8f %+08.8f %+08.8f\n", output[0], output[1], output[2],
         output[3]);
};
AMPL_FORCEINLINE void print_128i(__m128i &a, bool d = true) {
  int output[4];
  _mm_storeu_si128((__m128i *)output, a); // Store result in an array
  if (d)
    printf("%+03d %+03d %+03d %+03d\n", output[0], output[1], output[2],
           output[3]);
  else
    printf("%03u %03u %03u %03u\n", output[0], output[1], output[2], output[3]);
};

__m128 load_stride_3_shuffle(const float *base_ptr, size_t i) {
  // We need { base[i*3], base[i*3+3], base[i*3+6], base[i*3+9] }
  // This requires three loads to cover the memory span.
  __m128 v1 = _mm_loadu_ps(&base_ptr[i * 3]);     // { x0,y0,z0, x1 }
  __m128 v2 = _mm_loadu_ps(&base_ptr[i * 3 + 4]); // { y1,z1,x2,y2 }
  __m128 v3 = _mm_loadu_ps(&base_ptr[i * 3 + 8]); // { z2,x3,y3,z3 }

  // We need to extract { v1[0], v1[3], v2[2], v3[1] }
  // This requires a complex sequence of shuffles.
  // First, get the first two elements {v1[0], v1[3]} into the low part of a
  // register.
  __m128 low = _mm_shuffle_ps(
      v1, v1, _MM_SHUFFLE(0, 0, 3, 0)); // {v1[0], v1[3], v1[0], v1[3]}
                                        // {x0, x1, x0, x1}

  // Next, get the last two elements {v2[2], v3[1]} into the low part of a
  // register.
  __m128 high = _mm_shuffle_ps(v2, v3,
                               _MM_SHUFFLE(0, 0, 1,
                                           2)); // {v2[2], v3[1], v2[1], v3[0]}
                                                // x2, x3 , z1, z2

  // Finally, combine the low parts of the two shuffled registers.
  __m128 result = _mm_movelh_ps(low, high); // {low[0], low[1], high[0],
                                            // high[1]} {v1[0], v1[3], v2[2],
                                            // v3[1]} {x0,x1,x2,x3}
  return result;
}

AMPL_FORCEINLINE void cross_soa(const VecfSimd ax, const VecfSimd ay,
                                const VecfSimd az, const VecfSimd bx,
                                const VecfSimd by, const VecfSimd bz,
                                VecfSimd &rx, VecfSimd &ry, VecfSimd &rz) {
  rx = ay * bz - az * by;
  ry = az * bx - ax * bz;
  rz = ax * by - ay * bx;
}

AMPL_FORCEINLINE void transform_points_soa(const float *x, const float *y,
                                           const float *z, const uint32_t n,
                                           const float *q, const float *t,
                                           float *x_dst, float *y_dst,
                                           float *z_dst) {

  const VecfSimd qx4 = _mm_set1_ps(q[0]);
  const VecfSimd qy4 = _mm_set1_ps(q[1]);
  const VecfSimd qz4 = _mm_set1_ps(q[2]);
  const VecfSimd qw4 = _mm_set1_ps(q[3]);
  const VecfSimd tx4 = _mm_set1_ps(t[0]);
  const VecfSimd ty4 = _mm_set1_ps(t[1]);
  const VecfSimd tz4 = _mm_set1_ps(t[2]);

  size_t simd_end = n & ~0x3;      // Number of elements in full vectors
  size_t remainder = n - simd_end; // 0, 1, 2, or 3

  VecfSimd rx4;
  VecfSimd ry4;
  VecfSimd rz4;

  const float *ptr_x = x;
  const float *ptr_y = y;
  const float *ptr_z = z;
  float *ptr_x_dst = x_dst;
  float *ptr_y_dst = y_dst;
  float *ptr_z_dst = z_dst;

  // for (auto i = 0U; i < n; i += AMPL_SIMDW_F) {
  // printf("%d\n", remainder);

  for (auto i = 0U; i < simd_end; i += AMPL_SIMDW_F) {
    VecfSimd x4 = _mm_loadu_ps(ptr_x);
    VecfSimd y4 = _mm_loadu_ps(ptr_y);
    VecfSimd z4 = _mm_loadu_ps(ptr_z);
    cross_soa(qx4, qy4, qz4, x4, y4, z4, rx4, ry4, rz4);

    rx4 += rx4;
    ry4 += ry4;
    rz4 += rz4;
    x4 += qw4 * rx4;
    y4 += qw4 * ry4;
    z4 += qw4 * rz4;
    cross_soa(qx4, qy4, qz4, rx4, ry4, rz4, rx4, ry4, rz4);
    x4 += rx4;
    y4 += ry4;
    z4 += rz4;
    x4 += tx4;
    y4 += ty4;
    z4 += tz4;
    _mm_storeu_ps(ptr_x_dst, x4);
    _mm_storeu_ps(ptr_y_dst, y4);
    _mm_storeu_ps(ptr_z_dst, z4);
    ptr_x += AMPL_SIMDW_F;
    ptr_y += AMPL_SIMDW_F;
    ptr_z += AMPL_SIMDW_F;
    ptr_x_dst += AMPL_SIMDW_F;
    ptr_y_dst += AMPL_SIMDW_F;
    ptr_z_dst += AMPL_SIMDW_F;
  }
  // return;
  if (remainder > 0) {
    // Create a mask: e.g., for remainder=3, mask is 0b111 (7)
    int mask = (1 << remainder) - 1;

    // printf("re%d\n", mask);

    __m128i mask_vec = _mm_setr_epi32(
        -1,                        // 0xFFFFFFFF (MSB is 1)
        (remainder >= 2) ? -1 : 0, // 0xFFFFFFFF if remainder >= 2, else 0
        (remainder >= 3) ? -1 : 0, // 0xFFFFFFFF if remainder >= 3, else 0
        0);
    // print_128i(mask_vec);

    VecfSimd x4 = _mm_maskload_ps(ptr_x, mask_vec);
    VecfSimd y4 = _mm_maskload_ps(ptr_y, mask_vec);
    VecfSimd z4 = _mm_maskload_ps(ptr_z, mask_vec);
    // print_128(x4);
    cross_soa(qx4, qy4, qz4, x4, y4, z4, rx4, ry4, rz4);

    rx4 += rx4;
    ry4 += ry4;
    rz4 += rz4;
    x4 += qw4 * rx4;
    y4 += qw4 * ry4;
    z4 += qw4 * rz4;
    cross_soa(qx4, qy4, qz4, rx4, ry4, rz4, rx4, ry4, rz4);
    x4 += rx4;
    y4 += ry4;
    z4 += rz4;
    x4 += tx4;
    y4 += ty4;
    z4 += tz4;
    // _mm_storeu_ps(ptr_x_dst, x4);
    // _mm_storeu_ps(ptr_y_dst, y4);
    // _mm_storeu_ps(ptr_z_dst, z4);

    _mm_maskstore_ps(ptr_x_dst, mask_vec, x4);
    _mm_maskstore_ps(ptr_y_dst, mask_vec, y4);
    _mm_maskstore_ps(ptr_z_dst, mask_vec, z4);
  }
};

} // namespace ampl

#endif