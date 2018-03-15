cmake_minimum_required(VERSION 3.0)

project( min_snap )

set(version_major 2)
set(version_minor 1)
set(version_patch 0)
set(version "${version_major}.${version_minor}.${version_patch}")
set(package-contact "utopcu@utexas.edu")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_MODULE_PATH "/${PROJECT_SOURCE_DIR}/cmake/modules;${CMAKE_MODULE_PATH}")

set(${PROJECT_NAME}_INCLUDE_DIRS
  ${PROJECT_SOURCE_DIR}/include
  CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE
)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/openblas)

set(OpenBLAS_DIR
  "${CMAKE_CURRENT_BINARY_DIR}/openblas/lib/cmake/openblas"
)

SET(OpenBLAS_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/openblas/include)
SET(OpenBLAS_LIBRARIES ${CMAKE_CURRENT_BINARY_DIR}/openblas/lib/libopenblas.so)

if(NOT EXISTS "${CMAKE_CURRENT_BINARY_DIR}/openblas/lib/libopenblas.so")
  add_custom_command(
    OUTPUT ${OpenBLAS_DIR}
    COMMAND make FC=gfortran -C ${CMAKE_CURRENT_SOURCE_DIR}/external/OpenBLAS
    COMMAND make -C ${CMAKE_CURRENT_SOURCE_DIR}/external/OpenBLAS PREFIX=${CMAKE_CURRENT_BINARY_DIR}/openblas install
    COMMAND make -C ${CMAKE_CURRENT_SOURCE_DIR}/external/OpenBLAS clean
    COMMAND ${CMAKE_COMMAND} -E touch ${OpenBLAS_DIR}
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/openblas"
    COMMENT "Building OPENBLAS"
    VERBATIM
  )

  add_custom_target(
    OpenBLAS_INSTALL ALL
    DEPENDS  ${OpenBLAS_DIR}
  )
endif()


set(ENV{EIGEN3_ROOT_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/external/eigen-git-mirror)
find_package(Eigen3 3.3 REQUIRED)

find_package(Cholmod REQUIRED)
find_package(SPQR REQUIRED)
find_package(GSL REQUIRED)


set(CMAKE_CXX_FLAGS "-std=c++11 -msse3 -fopenmp -DNDEBUG -fpic ${BLAS_LINKER_FLAGS}  ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include_directories(
  ${PROJECT_NAME}_INCLUDE_DIRS
  ${OpenBLAS_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GSL_INCLUDE_DIRS}
  ${CHOLMOD_INCLUDES}
  ${SPQR_INCLUDES}
)

add_definitions( -DEIGEN_USE_BLAS -DEIGEN_USE_LAPACKE)

add_library( ${PROJECT_NAME} SHARED
  min_snap.cpp
)

target_link_libraries( ${PROJECT_NAME}
  ${CHOLMOD_LIBRARIES}
  ${SPQR_LIBRARIES}
  ${GSL_LIBRARY}
  ${OpenBLAS_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PUBLIC include)

# INSTALL STEPS
INSTALL(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib)

INSTALL(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include)

INSTALL(
  FILES ${CMAKE_BINARY_DIR}/Find${PROJECT_NAME}.cmake
  DESTINATION ${CMAKE_INSTALL_PREFIX})


add_executable(test_solver test_solver.cpp ${EIGEN3_INCLUDE_DIR} ${min_snap_INCLUDE_DIRS})
target_link_libraries(test_solver ${PROJECT_NAME})

#add_definitions(${BLAS_DEFINITIONS} ${LAPACK_DEFINITIONS})
#add_definitions( -DEIGEN_USE_BLAS -DEIGEN_USE_LAPACKE)
#add_definitions( -DEIGEN_USE_LAPACKE)
#add_executable(test_solver test_solver.cpp ${SRCS} ${HEADERS})
#target_link_libraries(test_solver ${CHOLMOD_LIBRARIES} ${SPQR_LIBRARIES} ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})
#target_link_libraries(test_solver ${CHOLMOD_LIBRARIES} ${SPQR_LIBRARIES} ${GSL_LIBRARY}  ${CBLAS_LIBRARIES})