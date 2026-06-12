find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)
find_library(LAPACKE_LIBRARY NAMES lapacke REQUIRED)
find_library(MATH_LIBRARY m)

add_library(hddc2b SHARED
  config.c
  wheel.c
  drive.c
  platform.c
  solver.c
)

target_include_directories(hddc2b
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(hddc2b
  PRIVATE
    ${LAPACKE_LIBRARY}
    LAPACK::LAPACK
    BLAS::BLAS
    ${MATH_LIBRARY}
)

install(
  TARGETS hddc2b
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

add_subdirectory(example)
