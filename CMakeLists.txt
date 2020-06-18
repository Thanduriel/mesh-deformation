cmake_minimum_required (VERSION 3.1)
project (mesh_deformation)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

set( SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set( DEPENDENCIES_DIR ${PROJECT_SOURCE_DIR}/dependencies)

if (MSVC)
	# disable type conversion warnings globaly to silence pmp
	add_compile_options(/wd4267)
	add_compile_options(/wd4244)
	add_compile_options(/wd4305)
endif()

#pmp
set(PMP_BUILD_APPS OFF CACHE BOOL "")
set(PMP_BUILD_EXAMPLES OFF CACHE BOOL "")
set(PMP_BUILD_DOCS OFF CACHE BOOL "")
set(PMP_BUILD_TESTS OFF CACHE BOOL "")
add_subdirectory (${DEPENDENCIES_DIR}/pmp-library)


#pmp libraries
#include_directories(${DEPENDENCIES_DIR}/pmp-library/src)
include_directories(${DEPENDENCIES_DIR}/pmp-library/external/imgui)
include_directories(${DEPENDENCIES_DIR}/pmp-library/external/eigen)
include_directories(${DEPENDENCIES_DIR}/pmp-library/external/glfw/include)
include_directories(${DEPENDENCIES_DIR}//pmp-library/external/glew/include)

include_directories( ${SRC_DIR})

#EXECUTABLES
file(GLOB_RECURSE SOURCE_FILES "${SRC_DIR}/*.hpp" "${SRC_DIR}/*.cpp" "${SRC_DIR}/*.inl")

# organize the files according to the organisation in the file system
# from: http://cmake.3232098.n2.nabble.com/Keep-folder-structure-in-Visual-Studio-Project-generated-with-CMake-td7586044.html
macro(GroupSources curdir)
   file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/${curdir} ${PROJECT_SOURCE_DIR}/${curdir}/*)
   foreach(child ${children})
          if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${curdir}/${child})
                  GroupSources(${curdir}/${child})
          else()
                  string(REPLACE "/" "\\" groupname ${curdir})
            source_group(${groupname} FILES ${PROJECT_SOURCE_DIR}/${curdir}/${child})
          endif()
   endforeach()
endmacro()

# execute tha macro
GroupSources("src")

add_executable(modeller ${SOURCE_FILES})
target_link_libraries( modeller pmp_vis )

if(NOT MSVC)
	target_compile_options(modeller PRIVATE "$<$<CONFIG:DEBUG>:-Wall;-pedantic>")
	target_compile_options(modeller PRIVATE "$<$<CONFIG:RELEASE>:-Wall;-pedantic;-O3;-march=native>")
endif()
