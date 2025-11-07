cmake_minimum_required(VERSION 3.16)

#option(ENABLE_SAMPLES "Build samples" OFF)
#option(ENABLE_TESTS "Test library" OFF)
option(LIBAMPL_SHARED "Build a shared library" ON)
option(BUILD_OPENMP "Build with openmp" ON)
#option(BUILD_MARCH_NATIVE "USE MARCH NATIVE" OFF)
option(BUILD_ONE_LIB "Build one shared library or not" ON)

FILE(STRINGS "VERSION" AMPL_VERSION)
project(ampl VERSION ${AMPL_VERSION})

set(ROOT_PROJECT_NAME ${PROJECT_NAME})

set( CMAKE_CXX_STANDARD 17 )
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(CMAKE_DEBUG_POSTFIX "d")

set(CONFIG_CMAKE_PATH lib/cmake/ampl)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})


message("${PROJECT_SOURCE_DIR}/cmake","${PROJECT_SOURCE_DIR}/cmake")
configure_file(
  ${PROJECT_SOURCE_DIR}/include/ampl/ampl_config.h.in
  ${PROJECT_SOURCE_DIR}/include/ampl/ampl_config.h
)


# configure_file(
# ${PROJECT_SOURCE_DIR}/include/libsgm_config.h.in
# ${PROJECT_SOURCE_DIR}/include/libsgm_config.h
# )
find_package(Eigen3 3.4 REQUIRED)
#find_package(tinycgl REQUIRED )
if (EIGEN_USE_LAPACKE)
  add_definitions(-DEIGEN_USE_LAPACKE=1)
  #find_package(LAPACK REQUIRED )
  message("DEBUG EIGEN_USE_LAPACKE = ${EIGEN_USE_LAPACKE}")
endif()
#find_package(LAPACKE REQUIRED )
if (BUILD_OPENMP)
  find_package(OpenMP)
endif()
if (OPENMP_CXX_FOUND)
  add_definitions(-DAMPL_OPENMP)
  message("DEBUG OPENMP = ${OpenMP_CXX_INCLUDE_DIRS}")
endif ()

if (WIN32)
  add_compile_options(/d2Zi+ /bigobj /wd4251 /wd4275 /wd4267 /wd4244)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
  add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
  add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

  set(CMAKE_DEPFILE_FLAGS_CXX "")
  set(CMAKE_DEPFILE_FLAGS_C "")

  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
  add_definitions(-DUSE_DLL)
  add_definitions(-DAMPL_EXPORTING)
  # else()
  # add_compile_options(-Wa,-mbig-obj)
endif ()




# uninstall target
if (NOT TARGET uninstall)
  configure_file(
    "cmake/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

  add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake)
endif ()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
add_subdirectory(src)



set(AMPL_LIBRARIES ampl-utils ampl-geometry ampl-kinematics)


if (BUILD_ONE_LIB)
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/world.cpp "")
  add_library(${PROJECT_NAME} SHARED ${CMAKE_CURRENT_BINARY_DIR}/world.cpp)

  foreach (LIB ${AMPL_LIBRARIES})
    target_link_libraries(${PROJECT_NAME} PRIVATE $<TARGET_OBJECTS:${LIB}>)
  endforeach ()
  if (EIGEN_USE_LAPACKE)
    target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen lapacke lapack)
  else ()
    target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen )
  endif()
  target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    $<INSTALL_INTERFACE:include>)
else ()
  add_library(${PROJECT_NAME} INTERFACE)
  target_link_libraries(${PROJECT_NAME} INTERFACE ${AMPL_LIBRARIES})
endif()

include(CMakePackageConfigHelpers)
set(LIB_DESTINATION lib)
set(INCLUDE_INSTALL_DIR include/ampl)




configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/amplConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/amplConfig.cmake
  INSTALL_DESTINATION ${CONFIG_CMAKE_PATH}
  PATH_VARS INCLUDE_INSTALL_DIR LIB_DESTINATION)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/amplConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${CONFIG_CMAKE_PATH}
)



set(INSTALL_TARGETS ${AMPL_LIBRARIES})
if (BUILD_ONE_LIB)
  set(INSTALL_TARGETS ${PROJECT_NAME})
endif ()



install(TARGETS ${INSTALL_TARGETS} 
  EXPORT ${PROJECT_NAME}-targets
  PUBLIC_HEADER DESTINATION include
  INCLUDES DESTINATION include
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)


export(EXPORT ${PROJECT_NAME}-targets NAMESPACE "ampl::"
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake"
)

install(EXPORT ${PROJECT_NAME}-targets
  FILE
  ${PROJECT_NAME}-targets.cmake
  DESTINATION
  ${CONFIG_CMAKE_PATH}
  NAMESPACE "ampl::"
)

if (WIN32)
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.targets"
    "<?xml version=\"1.0\"?>
<Project>
    <ItemGroup>
        <Content Include=\"$(MSBuildThisFileDirectory)..\\bin\\**\">
            <Visible>false</Visible>
            <PublishState>Included</PublishState>
            <CopyToOutputDirectory>Always</CopyToOutputDirectory>
            <Link>%(FileName)%(Extension)</Link>
            <Pack>false</Pack>
        </Content>
    </ItemGroup>
</Project>"
  )

  FILE(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Net.dll)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Net.dll DESTINATION lib/net48)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.targets DESTINATION build)
endif ()

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")


# if(ENABLE_SAMPLES)
# add_subdirectory(sample)
# endif()
# if (ENABLE_TESTS)
#   add_subdirectory(test)
# endif ()

add_subdirectory(dev)

include(cmake/Packing.cmake)




