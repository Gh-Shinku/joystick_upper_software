# Joystick-Upper-Software
> 本项目在队库 [Joystick-Upper-Software](https://github.com/BUPT-RobotTeam/joystick_upper_software) 做了适配于24届 RC 的修改
手柄上位机C++版

## 构建指南

本工程依赖boost-asio和boost-crc。暂时仅有在ROS上运行的版本。

可选依赖spdlog。

### ROS环境下编译

目前仅支持在ROS2环境下编译。

由于colcon build封装了CMake的构建流程，因此为了正确寻找vcpkg安装的依赖，这需要一些额外的努力。

具体地，当你在构建本工程时，你需要输入这一条指令：

```sh
colcon build --packages-select joystick_upper_software --cmake-args --preset=default
```

其中，`default`应该被修改为你在CMakeUserPresets.json中配置的`name`字段对应的值。

## 使用指南

```
ros2 launch joystick_upper_software joystick_launch.py 
```

这样将会启动整个手柄程序，并且开始对外发送数据。

### 参数定义

- ip ：手柄MCU的IP地址，字符串："192.168.4.1"；
- port ：手柄MCU的端口，数字：3456；
- serial_port : 手柄串口设备名，字符串："/dev/ttyUSB0"；
- controller : 是否需要启动底盘控制器，布尔值：false；

示例：你需要将串口设备名设置为`/dev/ttyACM0`

```
ros2 launch joystick_upper_software joystick_launch.py serial_port:="/dev/ttyACM0" 
```


## Troubleshooting

- ROS节点的程序中引入spdlog可能会导致段错误，预计为版本冲突
  - 解决方法：在ROS环境下不使用spdlog
  - 目前已不存在这个问题，程序应当可以自动检测环境并使用合适的日志接口
- 无法解析--preset参数
  - 解决方法：升级cmake到最新版本（3.20以上）
