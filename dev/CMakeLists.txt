cmake_minimum_required(VERSION 3.16)
set(SOMELIB_ROOT_DIR ${PROJECT_SOURCE_DIR})
set(SOMELIB_INCLUDE_DIR ${SOMELIB_ROOT_DIR}/include)
project(samples LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(tinycgl)
find_package(OpenMP)
#find_package(pamp REQUIRED )

if(OPENMP_CXX_FOUND)
    #message("FOUND OPENMP SUPPORT")
    add_definitions(-DAMPL_WITH_OPENMP)
    # find_package(OpenMP)
endif()

set(TARGET_NAME_EXE test)
add_executable(${TARGET_NAME_EXE} ./${TARGET_NAME_EXE}.cpp)
target_compile_options(${TARGET_NAME_EXE} PRIVATE
$<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wall -O3 -march=native  -fopenmp>
)
target_include_directories(${TARGET_NAME_EXE} PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(${TARGET_NAME_EXE} PUBLIC  ampl )

if(OpenMP_CXX_FOUND)
    target_link_libraries(${TARGET_NAME_EXE} PUBLIC OpenMP::OpenMP_CXX)
endif()

