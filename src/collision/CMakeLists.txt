cmake_minimum_required(VERSION 3.16)

set(THISLIB_ROOT_DIR ${PROJECT_SOURCE_DIR})
set(THISLIB_INCLUDE_DIR ${THISLIB_ROOT_DIR}/include)
set(MODULE_NAME ${ROOT_PROJECT_NAME}-collision)


set(THISLIB_TYPE STATIC)

if(LIBAMPL_SHARED)
	set(THISLIB_TYPE SHARED)
endif()



# target configuration
file(GLOB SRCS ./*.cpp ./*.c ./*.cu ./*.h*)
add_library(${MODULE_NAME} ${THISLIB_TYPE})
target_sources(${MODULE_NAME} PRIVATE ${SRCS})
target_link_libraries(${MODULE_NAME} PUBLIC Eigen3::Eigen)
target_include_directories(${MODULE_NAME} PUBLIC
	"$<BUILD_INTERFACE:${THISLIB_INCLUDE_DIR}>"
	$<INSTALL_INTERFACE:include>
)

#target_link_libraries(${MODULE_NAME} PRIVATE tinycgl::tinycgl)
# if(OpenMP_CXX_FOUND)
# 	target_link_libraries(${MODULE_NAME} PUBLIC OpenMP::OpenMP_CXX)
# endif()
target_compile_features(${MODULE_NAME} PRIVATE cxx_std_17)

# target_link_libraries(${PROJECT_NAME} PUBLIC CUDA::cudart $<$<BOOL:${BUILD_OPENCV_WRAPPER}>:${OpenCV_LIBS}>)
# set_target_properties(${PROJECT_NAME} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${THISLIB_INCLUDE_DIR})
# set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_ARCHITECTURES ${CUDA_ARCHS})
target_compile_options(${MODULE_NAME} PRIVATE
	$<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wall -Ofast -march=native -DRELEASE -funroll-loops -ffast-math>
	# $<$<COMPILE_LANG_AND_ID:CXX,Clang>:-Wall -O3>
	$<$<COMPILE_LANG_AND_ID:CXX,MSVC>:/wd4819>
	# $<$<COMPILE_LANGUAGE:CUDA>:-lineinfo>
)

