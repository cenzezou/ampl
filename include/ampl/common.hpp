#ifndef AMPL_COMMON_H
#define AMPL_COMMON_H

#ifndef __FUNCTION_NAME__
#ifdef WIN32  // WINDOWS
#define __FUNCTION_NAME__ __FUNCTION__
#else  //*NIX
#define __FUNCTION_NAME__ __func__
#endif
#endif

#include <ampl/ampl_config.h>
#include <cstdint>
#if AMPL_DEBUG == 1
#include <stdio.h>
#define FIRST_ARG( arg1, ... )       arg1
#define GET_FIRST( ... )             FIRST_ARG( __VA_ARGS__ )
#define SKIP_FIRST_ARG( first, ... ) __VA_ARGS__
#define printf_debug( fmt, ... )               \
  do                                           \
  {                                            \
    if ( AMPL_DEBUG )                          \
    {                                          \
      printf( "[%-34s] ", __FUNCTION_NAME__ ); \
      printf( fmt, __VA_ARGS__ );              \
    }                                          \
  } while ( 0 )

#else
#define printf_debug( fmt, ... ) \
  do                             \
  {                              \
  } while ( 0 )
#endif

#endif