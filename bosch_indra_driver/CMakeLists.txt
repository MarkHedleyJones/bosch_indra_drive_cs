cmake_minimum_required(VERSION 2.8.3)
project(bosch_indra_driver)

find_package(catkin REQUIRED COMPONENTS odva_ethernetip roscpp sensor_msgs)

find_package(Boost 1.47 REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS odva_ethernetip roscpp sensor_msgs
  LIBRARIES bosch_indra
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(bosch_indra src/indra.cpp)
target_link_libraries(bosch_indra
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(scanner_node src/scanner_node.cpp)
target_link_libraries(scanner_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(bosch_indra_node src/indra_node.cpp)
target_link_libraries(bosch_indra_node
  bosch_indra
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## Mark executables and libraries for installation
install(TARGETS bosch_indra bosch_indra_node scanner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
