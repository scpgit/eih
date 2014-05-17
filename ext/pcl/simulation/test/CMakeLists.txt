include_directories(
  SYSTEM  
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
  ${GLUT_INCLUDE_DIR} 
  ${GLEW_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

add_library(simulation_io  simulation_io.cpp)
target_link_libraries(simulation_io pcl_simulation pcl_common pcl_io ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${VTK_IO_TARGET_LINK_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})

add_executable(sim_test_simple sim_test_simple.cpp)
target_link_libraries(sim_test_simple ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${VTK_IO_TARGET_LINK_LIBRARIES} ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} pcl_simulation pcl_common pcl_io pcl_visualization)