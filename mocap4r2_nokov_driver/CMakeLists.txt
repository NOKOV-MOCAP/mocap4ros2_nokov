cmake_minimum_required(VERSION 3.10)
project(mocap4r2_nokov_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(mocap4r2_control REQUIRED)
find_package(mocap4r2_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# The Nokov SDK is not distributed as a ROS package, it comes with the driver.
set(nokov_sdk_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/nokov_sdk/include)
if(${ARCHITECTURE} MATCHES "arm")
  set(nokov_sdk_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/nokov_sdk/lib/armhf)
elseif(${ARCHITECTURE} MATCHES "aarch")
  set(nokov_sdk_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/nokov_sdk/lib/aarch64)
else()
  set(nokov_sdk_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/nokov_sdk/lib/x64)
endif()
set(nokov_sdk_LIBRARIES ${nokov_sdk_LIB_DIR}/libnokov_sdk.so)

add_library(${PROJECT_NAME} SHARED
  src/nokov_frame.cpp
  src/nokov_driver.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
# The headers of the SDK are needed by the public header of the driver.
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  $<BUILD_INTERFACE:${nokov_sdk_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}/nokov_sdk>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  mocap4r2_control::mocap4r2_control
  ${mocap4r2_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
target_link_libraries(${PROJECT_NAME} PRIVATE ${nokov_sdk_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${nokov_sdk_LIB_DIR}"
  INSTALL_RPATH "$ORIGIN")

# ROS 2 Galactic exports the component library as component_manager, the newer
# distributions as component.
if(TARGET rclcpp_components::component)
  target_link_libraries(${PROJECT_NAME} PRIVATE rclcpp_components::component)
else()
  target_link_libraries(${PROJECT_NAME} PRIVATE rclcpp_components::component_manager)
endif()
rclcpp_components_register_nodes(${PROJECT_NAME} "mocap4r2_nokov_driver::NokovDriver")

add_executable(nokov_driver src/nokov_driver_main.cpp)
target_link_libraries(nokov_driver ${PROJECT_NAME})
set_target_properties(nokov_driver PROPERTIES
  BUILD_RPATH "${nokov_sdk_LIB_DIR}"
  INSTALL_RPATH "$ORIGIN/..")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS nokov_driver
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES ${nokov_sdk_LIBRARIES}
  DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY nokov_sdk/include/
  DESTINATION include/${PROJECT_NAME}/nokov_sdk
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The Nokov SDK is third party code that is distributed with this package.
  # It is skipped by the linters through the AMENT_IGNORE file it contains.
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_nokov_frame test/test_nokov_frame.cpp)
  if(TARGET test_nokov_frame)
    target_include_directories(test_nokov_frame PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(test_nokov_frame
      ${PROJECT_NAME}
      mocap4r2_control::mocap4r2_control
      ${mocap4r2_interfaces_TARGETS}
      rclcpp::rclcpp)
  endif()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  mocap4r2_control mocap4r2_interfaces rclcpp rclcpp_lifecycle)

ament_package()
