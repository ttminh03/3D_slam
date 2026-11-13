cmake_minimum_required(VERSION 3.8)
project(amr_3d_demo)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree)
find_package(Eigen3 REQUIRED)

add_library(demo_algorithm src/algorithm.cpp)
target_compile_features(demo_algorithm PUBLIC cxx_std_17)
target_include_directories(demo_algorithm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(demo_algorithm PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})

add_executable(demo_node src/demo_node.cpp)
target_compile_features(demo_node PUBLIC cxx_std_17)
target_link_libraries(demo_node demo_algorithm)
ament_target_dependencies(demo_node rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs
  tf2 tf2_ros pcl_conversions)

install(TARGETS demo_algorithm demo_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_algorithm test/test_algorithm.cpp)
  target_link_libraries(test_algorithm demo_algorithm)
endif()

ament_export_include_directories(include)
ament_export_libraries(demo_algorithm)
ament_package()
