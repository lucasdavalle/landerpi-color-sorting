cmake_minimum_required(VERSION 3.8)
project(landerpi_color_sorting)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

# Pure logic, no rclcpp dependency: kept as their own targets so they can be
# gtest-ed without bringing up rclcpp.
add_library(color_block_detector src/color_block_detector.cpp)
target_include_directories(color_block_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(color_block_detector PUBLIC ${OpenCV_LIBS})
target_compile_features(color_block_detector PUBLIC cxx_std_17)

install(TARGETS color_block_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_color_block_detector test/test_color_block_detector.cpp)
  target_link_libraries(test_color_block_detector color_block_detector)
  target_compile_definitions(test_color_block_detector PRIVATE
    TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
endif()

ament_package()
