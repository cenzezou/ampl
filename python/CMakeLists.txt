cmake_minimum_required(VERSION 3.6)
project(${SKBUILD_PROJECT_NAME} LANGUAGES CXX)


option(BUILD_OPENMP "Build with openmp" ON)
#message(STATUS "CMake executable path: ${CMAKE_COMMAND}")
#message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib:/usr/local/lib")
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

#list(APPEND CMAKE_PREFIX_PATH "/home/czhou/miniconda3/envs/pt_py310")
#list(APPEND CMAKE_PREFIX_PATH "/home/czhou/anaconda3/envs/d2l")



set( CMAKE_CXX_STANDARD 17 )

# set(CMAKE_DEPFILE_FLAGS_CXX "")
# set(CMAKE_DEPFILE_FLAGS_C "")

find_package( Eigen3 REQUIRED )


find_package(ampl)

if (BUILD_OPENMP)
  find_package(OpenMP)
endif()
find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)
find_package(nanobind CONFIG REQUIRED)


#enable_language(CUDA)
nanobind_add_module(_core_ext
src/binding/python.cc



)


target_link_libraries(_core_ext PRIVATE  Eigen3::Eigen ampl::ampl  )
target_compile_options(_core_ext PRIVATE
	$<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wall -Ofast -fopenmp -march=native>
	# $<$<COMPILE_LANG_AND_ID:CXX,Clang>:-Wall -O3>
	$<$<COMPILE_LANG_AND_ID:CXX,MSVC>:/wd4819>
	# $<$<COMPILE_LANGUAGE:CUDA>:-lineinfo>
)

if (OPENMP_CXX_FOUND)
  add_definitions(-DAMPL_OPENMP)
  message("DEBUG OPENMP = ${OpenMP_CXX_INCLUDE_DIRS}")
  target_link_libraries(_core_ext PRIVATE OpenMP::OpenMP_CXX)
endif ()



set(STUB_PREFIX "${CMAKE_BINARY_DIR}/stubs/")


nanobind_add_stub(
  ampl_stub
  MODULE _core_ext
  OUTPUT "${STUB_PREFIX}__init__.pyi"
  PYTHON_PATH $<TARGET_FILE_DIR:_core_ext>
  DEPENDS _core_ext
  VERBOSE
)

#message(${STUB_PREFIX})

install(
  TARGETS _core_ext
  LIBRARY
  DESTINATION ampl/_core
)


install(
  FILES "${STUB_PREFIX}__init__.pyi"
  DESTINATION "src/ampl/_core"
)
# install(
#    FILES "${CMAKE_SOURCE_DIR}__init__.pyi"
#    DESTINATION "${CMAKE_SOURCE_DIR}/src/pamp/core"
# )

#install(FILES "${CMAKE_SOURCE_DIR}/src/imgui/imgui_ext.pyi" DESTINATION imgui)

