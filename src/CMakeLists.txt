include_directories(${GLUT_INCLUDE_DIR} ${GLEW_INCLUDE_DIR})
include_directories(${VTK_INCLUDE_DIRS})

set(OSG_LIBRARIES osg osgViewer OpenThreads osgDB osgGA osgUtil)       
set(PCL_LIBRARIES kdtree simulation common io visualization ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})
       
include_directories(
  SYSTEM  
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
  ${Boost_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS}
  ${OSG_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${TRAJOPT_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
                    
add_library(simulation_io
  simulation_io.cpp
)
target_link_libraries(simulation_io pcl_simulation pcl_common pcl_io ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${VTK_IO_TARGET_LINK_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})

add_executable(sim_test_simple sim_test_simple.cpp)
target_link_libraries(sim_test_simple ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${VTK_IO_TARGET_LINK_LIBRARIES} pcl_simulation pcl_common pcl_io pcl_visualization ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES})

#add_library(sensorsim
#  sim_kinect.cpp
#)
#target_link_libraries(sensorsim ${OSG_LIBRARIES} ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${Boost_SYSTEM_LIBRARY})

add_executable(sim_kinect_expt sim_kinect_expt.cpp)
target_link_libraries(sim_kinect_expt osgviewer ${Boost_SYSTEM_LIBRARY} ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} sensorsim utils)

#include_directories(${PYTHON_NUMPY_INCLUDE_DIR})
#boost_python_module(sensorsimpy sensorsimpy.cpp)
#target_link_libraries(sensorsimpy ${Boost_SYSTEM_LIBRARY})
