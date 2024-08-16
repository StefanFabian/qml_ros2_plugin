cmake_minimum_required(VERSION 3.10)
project(qml_ros2_plugin VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)

#set(CMAKE_BUILD_TYPE "Debug")

option(GLOBAL_INSTALL "Install the plugin globally instead of as a ROS2 overlay." OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(ros_babel_fish REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Qt5 COMPONENTS Core Multimedia Qml Quick REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP yaml-cpp)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Multimedia_INCLUDE_DIRS}
  ${Qt5Qml_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
)

set(SOURCES
  include/qml_ros2_plugin/conversion/message_conversions.hpp
  include/qml_ros2_plugin/conversion/qml_ros_conversion.hpp
  include/qml_ros2_plugin/array.hpp
  include/qml_ros2_plugin/action_client.hpp
  include/qml_ros2_plugin/ament_index.hpp
  include/qml_ros2_plugin/babel_fish_dispenser.hpp
  include/qml_ros2_plugin/goal_handle.hpp
  include/qml_ros2_plugin/goal_status.hpp
  include/qml_ros2_plugin/image_buffer.hpp
  include/qml_ros2_plugin/image_transport_manager.hpp
  include/qml_ros2_plugin/image_transport_subscription.hpp
  include/qml_ros2_plugin/io.hpp
  include/qml_ros2_plugin/logger.hpp
  include/qml_ros2_plugin/publisher.hpp
  include/qml_ros2_plugin/qobject_ros2.hpp
  include/qml_ros2_plugin/ros2.hpp
  include/qml_ros2_plugin/service_client.hpp
  include/qml_ros2_plugin/subscription.hpp
  include/qml_ros2_plugin/tf_transform.hpp
  include/qml_ros2_plugin/tf_transform_listener.hpp
  include/qml_ros2_plugin/topic_info.hpp
  include/qml_ros2_plugin/time.hpp
  src/array.cpp
  src/action_client.cpp
  src/ament_index.cpp
  src/babel_fish_dispenser.cpp
  src/goal_handle.cpp
  src/image_buffer.cpp
  src/image_transport_manager.cpp
  src/image_transport_subscription.cpp
  src/io.cpp
  src/logger.cpp
  src/message_conversions.cpp
  src/publisher.cpp
  src/qml_ros2_plugin.cpp
  src/qobject_ros2.cpp
  src/ros2.cpp
  src/service_client.cpp
  src/subscription.cpp
  src/tf_transform.cpp
  src/tf_transform_listener.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Multimedia Qt5::Qml Qt5::Quick ${YAML_CPP_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} ament_index_cpp rclcpp image_transport ros_babel_fish tf2_ros)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_babel_fish_test_msgs REQUIRED)
  find_package(example_interfaces REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(Qt5Test REQUIRED)

  set(TEST_DEPENDENCIES example_interfaces ros_babel_fish_test_msgs std_srvs)

  ament_add_gtest(test_communication test/communication.cpp)
  ament_target_dependencies(test_communication ${TEST_DEPENDENCIES})
  target_link_libraries(test_communication ${PROJECT_NAME} Qt5::Test)

  ament_add_gtest(test_image_conversions test/image_conversions.cpp)
  ament_target_dependencies(test_image_conversions  ${TEST_DEPENDENCIES})
  target_link_libraries(test_image_conversions ${PROJECT_NAME})

  ament_add_gtest(test_image_transport_subscriber test/image_transport_subscriber.cpp)
  ament_target_dependencies(test_image_transport_subscriber  ${TEST_DEPENDENCIES})
  target_link_libraries(test_image_transport_subscriber ${PROJECT_NAME})

  ament_add_gtest(test_message_conversions test/message_conversions.cpp)
  ament_target_dependencies(test_message_conversions  ${TEST_DEPENDENCIES})
  target_link_libraries(test_message_conversions ${PROJECT_NAME})

  ament_add_gtest(test_ros_life_cycle test/ros_life_cycle.cpp)
  ament_target_dependencies(test_ros_life_cycle  ${TEST_DEPENDENCIES})
  target_link_libraries(test_ros_life_cycle ${PROJECT_NAME})

  ament_add_gtest(test_logging test/logging.cpp)
  target_link_libraries(test_logging ${PROJECT_NAME})
  #
  ament_add_gtest(test_io test/io.cpp)
  ament_target_dependencies(test_io  ${TEST_DEPENDENCIES})
  target_link_libraries(test_io ${PROJECT_NAME})

  install(DIRECTORY test/test_io DESTINATION share/qml_ros2_plugin/test/)
endif()

# to run: catkin build --this --no-deps -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug -v --catkin-make-args ${PROJECT_NAME}_coverage
# Path to results overview will be printed in the build process
# Big thanks to the moveit people from whose docs I've obtained the information on how to get the coverage
if(BUILD_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)   # catkin package ros-*-code-coverage
  include(CodeCoverage)
  append_coverage_compiler_flags()
  set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*" "*/build/*" "*/src/qml_ros2_plugin.cpp")
  add_code_coverage(NAME ${PROJECT_NAME}_coverage)
endif()

#############
## Install ##
#############


if(${GLOBAL_INSTALL})
  message(STATUS "Installing plugin globally.")
  # Install Qml plugin as found here
  # https://github.com/4rtzel/cmake-qml-plugin-example/issues/1
  set(URI Ros2)
  string(REPLACE "." "/" TARGETPATH ${URI})
  execute_process(COMMAND qmake -qt5 -query QT_INSTALL_QML
    OUTPUT_VARIABLE QT_INSTALL_QML_RAW)
  string(REPLACE "\n" "" QT_INSTALL_QML ${QT_INSTALL_QML_RAW})
  if("${QT_INSTALL_QML}" STREQUAL "**Unknown**")
    message(FATAL_ERROR "Could not find qml plugin dir. Is qml installed?")
  endif()
  message(STATUS "Plugin will be installed to ${QT_INSTALL_QML}")
  set(DESTDIR "${QT_INSTALL_QML}/${TARGETPATH}")
  install(TARGETS ${PROJECT_NAME} DESTINATION ${DESTDIR})
  install(FILES ${CMAKE_CURRENT_LIST_DIR}/qmldir DESTINATION ${DESTDIR})
else()
  message(STATUS "Installing as part of a ROS2 workspace.")
  # Register plugin to be found by QML
  ament_environment_hooks(environment_setup.dsv.in)
  install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} DESTINATION lib/Ros2)
  install(FILES qmldir DESTINATION lib/Ros2)
  # The export is only necessary if part of a ROS2 workspace
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
endif()


ament_package()
