cmake_minimum_required(VERSION 2.8.3)
project(delta_robot)

add_compile_options(-std=c++11)

find_package(Eigen3)

set(${PROJECT_NAME}_DEPENDS
  #Package dependencies
  roscpp
  std_msgs
  sensor_msgs
)
    
set(${PROJECT_NAME}_INCLUDE_DIRS
  #Include folders
  include
  ${EIGEN3_INCLUDE_DIRS}
)

find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_DEPENDS}
)

#What to export (include dirs, libraries, etc)
catkin_package(
  INCLUDE_DIRS ${catkin_INCLUDE_DIRS} ${${PROJECT_NAME}_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_DEPENDS}
)

include_directories( ${catkin_INCLUDE_DIRS} ${${PROJECT_NAME}_INCLUDE_DIRS})

# Create library
add_library(${PROJECT_NAME}_model src/delta_model.cpp)
add_dependencies(${PROJECT_NAME}_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_model ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_ros_interface src/delta_ros_interface.cpp)
add_dependencies(${PROJECT_NAME}_ros_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_ros_interface ${PROJECT_NAME}_model ${catkin_LIBRARIES})

# Create executable (node)
add_executable(${PROJECT_NAME}_node src/delta_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_ros_interface ${catkin_LIBRARIES})


## Tests
#if (CATKIN_ENABLE_TESTING)
#  catkin_add_gtest(${PROJECT_NAME}_test src/tests/${PROJECT_NAME}_test.cpp)
#  target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES})
#endif()

