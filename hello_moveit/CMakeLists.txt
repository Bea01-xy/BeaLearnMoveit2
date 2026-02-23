cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)

# --- 定义所有可执行文件 ---
add_executable(hello_moveit src/hello_moveit.cpp)
add_executable(try_moveit src/try_moveit.cpp)

# --- 批量配置头文件路径 ---
target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(try_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# --- 批量配置 C++ 标准 ---
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(try_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# --- 批量链接依赖库 ---
# 建议每个 target 分开写，
# 以免未来不同 node 需要不同的依赖
ament_target_dependencies(
  try_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_visual_tools"
)
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_visual_tools"
)

# --- 批量安装 ---
install(TARGETS hello_moveit try_moveit
  DESTINATION lib/${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

ament_package()
