cmake_minimum_required(VERSION 3.11)
project(Diff-Geom-Points)

# Options ----------------------------------------------------------------------
set(CMAKE_CXX_FLAGS                "-Wall -Wextra -pthread -msse3 -Wno-unknown-pragmas -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG          "-DPCP_DEBUG -D_DEBUG -g3 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE        "-DNDEBUG -O3")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-DNDEBUG -O2 -g3")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/install)

# DGtal ------------------------------------------------------------------------
SET(BUILD_EXAMPLES OFF CACHE BOOL "DGtal examples")
add_subdirectory("${PROJECT_SOURCE_DIR}/../external/DGtal" "${PROJECT_BINARY_DIR}/external/DGtal")
include("${PROJECT_BINARY_DIR}/external/DGtal/DGtalConfig.cmake")
include_directories("${DGTAL_INCLUDE_DIRS}")

# CLI11 ------------------------------------------------------------------------
mark_as_advanced(CLI_CXX_STD CLI_EXAMPLES CLI_SINGLE_FILE CLI_SINGLE_FILE_TESTS CLI_TESTING)
SET(CLI11_SINGLE_FILE ON CACHE BOOL "Single File")
add_subdirectory("${PROJECT_SOURCE_DIR}/../external/CLI11" "${PROJECT_BINARY_DIR}/external/CLI11")
include_directories("${PROJECT_BINARY_DIR}/external/CLI11/include/")

# CGAL -------------------------------------------------------------------------
find_package(CGAL REQUIRED)

# OpenMP -----------------------------------------------------------------------
if(APPLE)
    message("-- Setting OpenMP flags on MacOs. Assuming `brew install libomp`")
    if(CMAKE_C_COMPILER_ID MATCHES "Clang\$")
        set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp -I/usr/local/opt/libomp/include" CACHE INTERNAL "OpenMP flags for Xcode toolchain.")
        set(OpenMP_C_LIB_NAMES "omp" CACHE INTERNAL "OpenMP lib name for Xcode toolchain.")
        set(OpenMP_omp_LIBRARY "/usr/local/opt/libomp/lib/libomp.dylib" CACHE INTERNAL "OpenMP lib name for Xcode toolchain.")
    endif()

    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang\$")
        set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp -I/usr/local/opt/libomp/include" CACHE INTERNAL "OpenMP flags for Xcode toolchain.")
        set(OpenMP_CXX_LIB_NAMES "omp" CACHE INTERNAL "OpenMP lib name for Xcode toolchain.")
        set(OpenMP_omp_LIBRARY "/usr/local/opt/libomp/lib/libomp.dylib" CACHE INTERNAL "OpenMP lib name for Xcode toolchain.")
    endif()
endif()

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    message(STATUS "OpenMP found")
else()
    message(WARNING "OpenMP not found")
endif()

# Library ----------------------------------------------------------------------
file(GLOB_RECURSE lib_headers src/*.hpp src/*.h)
file(GLOB_RECURSE lib_sources src/*.cpp)
file(GLOB_RECURSE lib_inlines src/*.inl)
set(lib_source_files ${lib_headers} ${lib_sources} ${lib_inlines})

add_library(pcplib STATIC ${lib_source_files})

target_include_directories(pcplib PUBLIC src/
                                         ../external/Eigen/
                                         ../external/Ponca/
                                         OpenMP::OpenMP_CXX)

# Applications -----------------------------------------------------------------
file(GLOB_RECURSE app_sources app/*.cpp)

foreach(file ${app_sources})
    get_filename_component(name ${file} NAME_WE)
    file(RELATIVE_PATH relative_file ${CMAKE_CURRENT_SOURCE_DIR} ${file})
    get_filename_component(relative_dir ${relative_file} DIRECTORY)

    # uncomment this to generate app executables in a hierarchy folder
    # set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${relative_dir})

    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(name "dgp${name}d")
    else()
        set(name "dgp${name}")
    endif()

    message(STATUS "Build app ${name}")

    add_executable(${name} ${file})
    add_dependencies(${name} pcplib DGtal ${DGTAL_LIBRARIES} CLI11-generate-single-file  CLI11::CLI11)
    target_link_libraries(${name} pcplib DGtal ${DGTAL_LIBRARIES} CGAL::CGAL CLI11::CLI11 pthread OpenMP::OpenMP_CXX)
endforeach()

