cmake_minimum_required(VERSION 2.8.3)
project(ros2_time)

# Add support for C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# List of all necessary header directories
set(INCLUDE_DIRS
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)
include_directories(${INCLUDE_DIRS})

set(LIBS
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

#### ros2_time library
add_library(${PROJECT_NAME} SHARED
  src/time.cpp
  src/duration.cpp
)
target_link_libraries(${PROJECT_NAME} ${LIBS})
set(LIBS ${LIBS} ${PROJECT_NAME})


############################################################
# Define tests
#

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    
    ament_add_gtest(ros2_time_tests test/time_tests.cpp)
    target_link_libraries(ros2_time_tests ${LIBS})
    
    ament_add_gtest(ros2_duration_tests test/duration_tests.cpp)
    target_link_libraries(ros2_duration_tests ${LIBS})
endif()


############################################################
# Define installation rules
#

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

# Export include dependencies, directories, and libraries
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})

# Must be at the bottom
ament_package()
