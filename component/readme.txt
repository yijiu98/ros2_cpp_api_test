src目录下
ros2 pkg create --build-type ament_cmake component

使用:
命令行 进行加载

注意：在一切使用命令行使用ros2的操作前，请务必 source。

ros2 component types // 列出所有可以加载的组件

使用上述指令，我们可以查看我们的组件是否编译和安装成功，也可以查看一会我们要用于加载的名字。

ros2 run rclcpp_components component_container // 启动组件容器

因为组件没有 main 函数，不能单独运行，因此需要启动一个空的进程用来作为组件执行的容器。利用这个指令来启动 ros2 提供给我们的默认的容器。

ros2 component load [包名] [组件名] // 该指令将组件挂在至默认的组件容器，注，执行此指令需新开一个命令行

使用上述指令，可以将你想要加载的组件挂载于默认的组件容器，随后，组件将会自动执行。


这里component相当与一个空进程，组件添加进去后就是在一个进程中执行的。
可以通过launch中的component的参数设置是多线程还是单线程执行。