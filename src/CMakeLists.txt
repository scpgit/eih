set(OSG_LIBRARIES osg osgViewer OpenThreads osgDB osgGA osgUtil)       
set(PCL_LIBRARIES kdtree common io visualization ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})
       
include_directories(
  SYSTEM  
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
  ${GLUT_INCLUDE_DIR} 
  ${GLEW_INCLUDE_PATH}
  ${Boost_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS}
  ${OSG_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${DISTORTION_INCLUDE_DIRS}
  ${TRAJOPT_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PCL_SIMULATION_INCLUDE_DIRS}
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
                   
add_executable(sim_kinect main.cpp sim_kinect.cpp)
target_link_libraries(sim_kinect ${Boost_SYSTEM_LIBRARY} ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES}  pcl_simulation pcl_common pcl_io ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} utils)