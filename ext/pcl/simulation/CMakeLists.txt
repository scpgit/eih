set(PCL_LIBRARIES kdtree common io visualization ${GLEW_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations") 

set(srcs 
    src/camera.cpp
    src/model.cpp
    src/range_likelihood.cpp
    src/scene.cpp
    src/glsl_shader.cpp
    src/sum_reduce.cpp
)

set(incs 
    include/pcl/simulation/camera.h
    include/pcl/simulation/model.h
    include/pcl/simulation/range_likelihood.h
    include/pcl/simulation/scene.h
    include/pcl/simulation/glsl_shader.h
    include/pcl/simulation/sum_reduce.h
)

include_directories(
  SYSTEM  
  ${GLEW_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(pcl_simulation ${srcs} ${incs})
target_link_libraries(pcl_simulation pcl_common pcl_io ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES})

#add_subdirectory(tools)
