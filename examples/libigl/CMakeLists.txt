cmake_minimum_required(VERSION 3.16)
project(aso-example-libigl)

set(CMAKE_CXX_STANDARD 17)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Libigl
option(LIBIGL_USE_STATIC_LIBRARY  ""            OFF)
option(LIBIGL_WITH_OPENGL         "Use OpenGL"  ON)
option(LIBIGL_WITH_OPENGL_GLFW    "Use GLFW"    ON)
include(libigl)

include_directories("${PROJECT_SOURCE_DIR}/../../include" # AlgebraicShapeOperator.h
                    ../common)

# Add your project files
file(GLOB SRC_FILES *.cpp)
add_executable(displayASO ${SRC_FILES})
target_link_libraries(displayASO PUBLIC igl::core igl::opengl_glfw)
