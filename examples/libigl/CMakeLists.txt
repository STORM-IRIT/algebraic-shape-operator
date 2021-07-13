cmake_minimum_required(VERSION 3.16)
project(aso-libigl)

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
add_executable(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC igl::core igl::opengl_glfw)
