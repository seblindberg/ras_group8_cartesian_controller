cmake_minimum_required(VERSION 2.8.3)
project(ras_group8_cartesian_controller)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  #ras_group8_util
)

catkin_package(
  INCLUDE_DIRS
    include

  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    #ras_group8_util
# DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Main node
add_executable(${PROJECT_NAME}
  src/cartesian_controller_node.cpp
  src/CartesianController.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  #ras_group8_util_reloadable
  ${catkin_LIBRARIES}
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_tests
    test/test_cartesian_controller.cpp
    src/CartesianController.cpp
  )
  
  target_link_libraries(${PROJECT_NAME}_tests
    ${catkin_LIBRARIES}
  )
endif()


# Make main executable
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Teleop node
add_executable(teleop
  src/teleop_node.cpp
  src/Teleop.cpp
)

add_dependencies(teleop
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(teleop
  ${catkin_LIBRARIES}
)

# Make Teleop
install(
  TARGETS teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)