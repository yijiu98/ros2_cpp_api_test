cmake_minimum_required(VERSION 3.8)
project(component)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include/)

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

# -----------------------------------------------------------------------------------------
# pub
# 生成库
ament_auto_add_library(component_demo_pub SHARED
        src/component_demo_pub.cpp
)
# 注册
rclcpp_components_register_node(component_demo_pub
        PLUGIN ne::ComponentDemoPub
        EXECUTABLE component_demo_pub_node
)

# 安装库文件
install(
  TARGETS
  component_demo_pub
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# -----------------------------------------------------------------------------------------
# sub
# 生成库
ament_auto_add_library(component_demo_sub SHARED
  src/component_demo_sub.cpp
)

# 注册
rclcpp_components_register_nodes(component_demo_sub
        PLUGIN ne::ComponentDemoSub
        EXECUTABLE component_demo_sub_node)

# 安装库文件
install(
  TARGETS
  component_demo_sub
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#-----------------------------------------------------------------------------------------------

# 安装头文件目录
install(
  DIRECTORY include/
  DESTINATION include/
)

# 安装Launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()