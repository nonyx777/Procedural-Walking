cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(proto VERSION 1.0)

set(OpenGL_GL_PREFERENCE "LEGACY")

find_package(SFML 2 REQUIRED network audio graphics window system)
find_package(OpenGL REQUIRED)

file(GLOB_RECURSE SRC_FILES src/*.cpp library/UI/*.cpp)
add_executable(proto ${SRC_FILES})

target_include_directories(proto PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
    ${CMAKE_CURRENT_SOURCE_DIR}/library/UI 
    ${SFML_INCLUDE_DIR})
target_link_libraries(proto PRIVATE 
    -lsfml-window -lsfml-graphics -lsfml-system
    ${OPENGL_gl_LIBRARY})
# target_compile_options(proto PRIVATE -g)