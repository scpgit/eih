cmake_minimum_required(VERSION 2.8)
project(eih)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

option(DEVEL_MODE "build with compiler more errors" OFF)
if (DEVEL_MODE) 
  set(CMAKE_CXX_FLAGS "-Werror -Wall -Wno-sign-compare")
endif(DEVEL_MODE)

set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-isystem ")        

# http://cmake.3232098.n2.nabble.com/Default-value-for-CMAKE-BUILD-TYPE-td7550756.html
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

find_package(Eigen REQUIRED)
find_package(Boost COMPONENTS system filesystem python thread date_time iostreams program_options REQUIRED)
find_package(OpenRAVE REQUIRED)
find_package(PCL 1.7 COMPONENTS kdtree simulation common io visualization REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(BUILD_SHARED_LIBS true)

set(TRAJOPT_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/ext/trajopt)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
add_definitions(${Boost_LIB_DIAGNOSTIC_DEFINITIONS})

include_directories(SYSTEM ${VTK_INCLUDE_DIRECTORIES})
link_directories(${VTK_LINK_DIRECTORIES})
set(VTK_IO_TARGET_LINK_LIBRARIES vtkCommon vtkWidgets vtkIO vtkImaging)

include("${CMAKE_SOURCE_DIR}/cmake/FindGLEW.cmake")
include("${CMAKE_SOURCE_DIR}/cmake/FindEigen.cmake")
include("${CMAKE_SOURCE_DIR}/cmake/boost-python.cmake")

add_subdirectory(ext)
add_subdirectory(src)
 