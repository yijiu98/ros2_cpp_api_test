cmake_minimum_required(VERSION 3.8)
project(example_topic_rclcpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies用于查找并加载其他软件包或库的配置文件
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
#用于将源代码文件编译为可执行文件
add_executable(topic_publisher_01 src/topic_publisher_01.cpp)
add_executable(topic_subscribe_01 src/topic_subscribe_01.cpp)
#用于指定目标的依赖关系
ament_target_dependencies(topic_publisher_01 rclcpp std_msgs) 
ament_target_dependencies(topic_subscribe_01 rclcpp std_msgs)
#安装位置
install(TARGETS
  topic_publisher_01
  topic_subscribe_01
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

ament_package()

