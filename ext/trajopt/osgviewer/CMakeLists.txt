find_package(OpenThreads)
find_package(osg)
find_package(osgDB)
find_package(osgUtil)
find_package(osgViewer)
find_package(osgGA)       
       
set(OSG_LIBRARIES osg osgViewer OpenThreads osgDB osgGA osgUtil osgText)       
       
include_directories(
  SYSTEM  
  ${TRAJOPT_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenRAVE_INCLUDE_DIRS}
  ${OSG_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
                    
add_library(osgviewer
  osgviewer.cpp 
  robot_ui.cpp
)
target_link_libraries(osgviewer ${OSG_LIBRARIES} ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${Boost_SYSTEM_LIBRARY} utils)

