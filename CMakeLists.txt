cmake_minimum_required(VERSION 3.8)
project(motion_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(design_patterns REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(RobotBaseModel include/motion_controller/RobotBaseModel.hpp)
set_target_properties(RobotBaseModel PROPERTIES LINKER_LANGUAGE CXX)
target_compile_features(RobotBaseModel PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(RobotBaseModel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  RobotBaseModel
  "rclcpp"
  "geometry_msgs"
  "design_patterns"
)
ament_target_dependencies(RobotBaseModel Eigen3)
target_include_directories(RobotBaseModel PUBLIC ${EIGEN3_INCLUDE_DIRS})


add_executable(model_example src/main.cpp)
target_link_libraries(model_example RobotBaseModel)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(RobotBaseModel PRIVATE "MOTION_CONTROLLER_BUILDING_LIBRARY")

install(
  DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS RobotBaseModel
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

install(TARGETS
        model_example
        DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
        include
)
ament_export_libraries(
  RobotBaseModel
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
