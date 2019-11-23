cmake_minimum_required (VERSION 3.1)
project (mesh_deformation)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

set( SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set( DEPENDENCIES_DIR ${PROJECT_SOURCE_DIR}/dependencies)

#pmp
set(PMP_BUILD_APPS OFF CACHE BOOL "")
set(PMP_BUILD_EXAMPLES OFF CACHE BOOL "")
set(PMP_BUILD_DOCS OFF CACHE BOOL "")
set(PMP_BUILD_TESTS OFF CACHE BOOL "")
add_subdirectory ("dependencies/pmp-library")
include_directories("dependencies/pmp-library/src")

#pmp libraries
include_directories(${DEPENDENCIES_DIR}/pmp-library/src)
include_directories(${DEPENDENCIES_DIR}/pmp-library/external/imgui)
include_directories(${DEPENDENCIES_DIR}/pmp-library/external/eigen)
include_directories(${DEPENDENCIES_DIR}/pmp-library/external/glfw/include)
include_directories(${DEPENDENCIES_DIR}//pmp-library/external/glew/include)

include_directories( ${SRC_DIR})

#EXECUTABLES
file(GLOB_RECURSE SOURCE_FILES "${SRC_DIR}/*.hpp" "${SRC_DIR}/*.cpp" "${SRC_DIR}/*.inl")
add_executable(modeller ${SOURCE_FILES})
target_link_libraries( modeller pmp_vis)
#target_link_libraries(modeller ${OPENGL_gl_LIBRARY})
#target_link_libraries (modeller glew_s) # use static version
#target_link_libraries (modeller glfw)