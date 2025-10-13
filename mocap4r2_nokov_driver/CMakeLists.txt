cmake_minimum_required(VERSION 3.5)

project(mocap4r2_nokov_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(nokov_sdk_INCLUDE_DIRS nokov_sdk/include)
set(nokov_sdk_LIB_DIRS nokov_sdk/lib)

if(${ARCHITECTURE} MATCHES "arm")
  set(nokov_sdk_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/${nokov_sdk_LIB_DIRS}/armhf/libnokov_sdk.so)
elseif(${ARCHITECTURE} MATCHES "aarch")
  set(nokov_sdk_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/${nokov_sdk_LIB_DIRS}/aarch64/libnokov_sdk.so)
else()
  set(nokov_sdk_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/${nokov_sdk_LIB_DIRS}/x64/libnokov_sdk.so)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mocap4r2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mocap4r2_control_msgs REQUIRED)
find_package(mocap4r2_control REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  mocap4r2_msgs
  mocap4r2_control
  mocap4r2_control_msgs
)

include_directories(
  include
  ${nokov_sdk_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/mocap4r2_nokov_driver/mocap4r2_nokov_driver.cpp)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "NOKOV_BUILDING_LIBRARY")

add_executable(mocap4r2_nokov_driver_main
  src/mocap4r2_nokov_driver_main.cpp
)
ament_target_dependencies(mocap4r2_nokov_driver_main ${dependencies})
target_link_libraries(mocap4r2_nokov_driver_main
  ${PROJECT_NAME}
  ${nokov_sdk_LIBRARIES}
)

ament_target_dependencies(mocap4r2_nokov_driver ${dependencies})
target_link_libraries(mocap4r2_nokov_driver ${nokov_sdk_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  mocap4r2_nokov_driver_main
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# if(BUILD_TESTING)
#   find_package(ament_cmake_copyright REQUIRED)
#   find_package(ament_cmake_cppcheck REQUIRED)
#   # find_package(ament_cmake_cpplint REQUIRED)
#   # find_package(ament_cmake_lint_cmake REQUIRED)
#   find_package(ament_cmake_xmllint REQUIRED)
#   find_package(ament_cmake_uncrustify REQUIRED)

#   ament_copyright(EXCLUDE ${EXCLUDE_FILES})
#   ament_cppcheck(
#     EXCLUDE ${EXCLUDE_FILES}
#     LANGUAGE c++
#   )
#   # ament_cpplint(EXCLUDE ${EXCLUDE_FILES})
#   # ament_lint_cmake()
#   ament_uncrustify(
#     EXCLUDE ${EXCLUDE_FILES}
#     LANGUAGE c++
#   )

#   find_package(ament_cmake_gtest REQUIRED)
# endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()