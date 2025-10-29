
#include <ampl/utils.hpp>
#include <stdio.h>

namespace ampl {

void version() {
  printf("%s = %d.\t%d.\t%d\n", __FUNCTION__, AMPL_VERSION_MAJOR,
         AMPL_VERSION_MINOR, AMPL_VERSION_PATCH);
}

} // namespace ampl
