#ifndef TEST_UTIL_HPP
#define TEST_UTIL_HPP

#include <pamp/pamp.hpp>

#define print_bits( x )                                                \
  do                                                                   \
  {                                                                    \
    unsigned long long a__ = ( x );                                    \
    size_t bits__          = sizeof( x ) * 8;                          \
    printf( #x ": " );                                                 \
    while ( bits__-- ) putchar( a__ &( 1ULL << bits__ ) ? '1' : '0' ); \
    putchar( '\n' );                                                   \
  } while ( 0 )

inline static std::chrono::time_point<std::chrono::steady_clock> timer;
inline static auto start = pamp::get_elapsed_ns( timer );
inline static void tic( bool print = false )
{
  start = pamp::get_elapsed_ns( timer );
  if ( print ) printf( "# [INFO] TIC\n" );
}
inline static int toc( bool print = false )
{
  auto d = pamp::get_elapsed_ns( timer ) - start;
  ;
  if ( print ) printf( "# [INFO] TOC = %f micro seconds\n", (double)d / 1000.0 );

  return d;
}
#endif
