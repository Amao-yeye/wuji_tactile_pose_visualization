cmake_minimum_required(VERSION 3.16)
project(wuji_rviz_panel)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wuji_tactile_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES include/wuji_rviz_panel/hand_control_panel.hpp)

add_library(wuji_hand_control_panel SHARED
  src/hand_control_panel.cpp
  ${MOC_FILES}
)
target_include_directories(wuji_hand_control_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(wuji_hand_control_panel PUBLIC
  Qt5::Widgets
  pluginlib::pluginlib
  rclcpp::rclcpp
  rviz_common::rviz_common
  ${std_msgs_TARGETS}
  ${wuji_tactile_msgs_TARGETS}
)

install(TARGETS wuji_hand_control_panel
  EXPORT export_wuji_hand_control_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES rviz_common_plugins.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_display_controls test/test_display_controls.cpp)
  target_link_libraries(test_display_controls
    wuji_hand_control_panel
    Qt5::Widgets
  )
  set_tests_properties(test_display_controls PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
  )
endif()

pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)
ament_export_include_directories(include)
ament_export_targets(export_wuji_hand_control_panel)
ament_export_dependencies(pluginlib rclcpp rviz_common std_msgs wuji_tactile_msgs)
ament_package()
