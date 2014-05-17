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
  ${GLEW_INCLUDE_PATH}
  ${PCL_INCLUDE_DIRS}
  ${PCL_SIMULATION_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(pcl_simulation ${srcs} ${incs})
target_link_libraries(pcl_simulation ${GLEW_LIBRARY} ${OPENGL_LIBRARIES} pcl_common pcl_io)

add_subdirectory(test)
