# Robotic Fish

`robotic_fish` 是基于 ROS 1 的机器鱼控制包，提供手柄遥控、键盘测试、舵机/扩展电机控制和 USB 相机标定辅助功能。舵机及 IMU 数据通过 `spinal` 与下位机通信。

## 1. 工作空间

本文默认使用以下 catkin 工作空间：

```text
~/ros/jsk_aerial_robot_ws
├── src/jsk_aerial_robot/robots/robotic_fish  # 本包源码，应在这里修改
├── build                                     # 编译中间文件，不要手动修改
├── devel                                     # 编译后的 ROS 环境
└── logs                                      # catkin build 日志
```

打开新终端后，先加载 ROS 和工作空间环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/ros/jsk_aerial_robot_ws/devel/setup.bash
```

可将以上两行加入 `~/.bashrc`，也可以在每个新终端中手动执行。

确认 ROS 使用的是当前工作空间中的代码：

```bash
rospack find robotic_fish
```

预期输出：

```text
/home/khadas/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/robotic_fish
```

## 2. 环境与依赖

推荐环境：

- Ubuntu 20.04
- ROS Noetic（ROS 1）
- `catkin_tools`
- `spinal`、`rosserial_server`
- `joy`
- 可选：`usb_cam`、`image_view`、`pynput`

首次配置工作空间时安装依赖：

```bash
cd ~/ros/jsk_aerial_robot_ws
source /opt/ros/noetic/setup.bash

rosdep install -y -r \
  --from-paths src \
  --ignore-src \
  --rosdistro noetic
```

如需手动补充常用依赖：

```bash
sudo apt update
sudo apt install -y \
  python3-catkin-tools \
  python3-numpy \
  python3-pynput \
  ros-noetic-joy \
  ros-noetic-rosserial-client \
  ros-noetic-rosserial-server \
  ros-noetic-usb-cam \
  ros-noetic-image-view
```

项目的顶层 `aerial_robot_noetic.rosinstall` 指定了带 ROS service 支持的定制 `rosserial`。操作真实硬件时，优先使用工作空间中由该文件获取的 `rosserial` 源码版本。

## 3. 编译

只编译本包及其依赖：

```bash
cd ~/ros/jsk_aerial_robot_ws
source /opt/ros/noetic/setup.bash

catkin build robotic_fish
source devel/setup.bash
```

切换 Git 分支、修改 `CMakeLists.txt`/`package.xml` 或首次加入本包后，建议强制重新配置：

```bash
catkin build robotic_fish --force-cmake
source devel/setup.bash
```

检查包和消息是否可用：

```bash
rospack find robotic_fish
rospack find spinal
rosmsg show spinal/ServoControlCmd
```

## 4. 硬件连接检查

默认设备如下：

| 设备 | PC | VIM4 |
| --- | --- | --- |
| 下位机串口 | `/dev/ttyUSB0` | `/dev/ttyS4` |
| 串口波特率 | `921600` | `921600` |
| 手柄 | `/dev/input/js0` | `/dev/input/js0` |

检查设备：

```bash
ls -l /dev/ttyUSB* /dev/ttyS4 2>/dev/null
ls -l /dev/input/js* 2>/dev/null
```

如当前用户没有串口或手柄权限：

```bash
sudo usermod -aG dialout,input "$USER"
```

执行后注销并重新登录，使用户组设置生效。

在启动执行器之前，先检查手柄消息：

```bash
rosrun joy joy_node _dev:=/dev/input/js0
rostopic echo /joy
```

不同手柄的 axes/buttons 映射可能不同。本包直接使用固定数组索引，必须确认手柄至少提供代码所需的轴和按钮，否则节点可能发生索引越界或产生错误动作。

## 5. 启动方式

### 5.1 PC 串口模式

PC 模式会自动启动 `spinal` 串口桥、手柄节点和控制节点：

```bash
roslaunch robotic_fish teleop.launch dev:=pc
```

默认连接 `/dev/ttyUSB0`，波特率为 `921600`。

### 5.2 VIM4 串口模式

当前 teleop launch 文件在 `dev:=vim4` 时不会自动包含 `spinal/bridge.launch`，因此需要分两个终端启动。

终端 1，启动 `/dev/ttyS4` 通信桥：

```bash
source /opt/ros/noetic/setup.bash
source ~/ros/jsk_aerial_robot_ws/devel/setup.bash
roslaunch spinal bridge.launch dev:=vim4
```

终端 2，启动手柄和控制节点：

```bash
source /opt/ros/noetic/setup.bash
source ~/ros/jsk_aerial_robot_ws/devel/setup.bash
roslaunch robotic_fish teleop.launch dev:=vim4
```

### 5.3 控制程序选择

| 启动文件 | 用途 | 主要输出 |
| --- | --- | --- |
| `teleop.launch` | 普通双舵机机器鱼 | `/servo/target_states` |
| `spring_teleop.launch` | 带弹簧和锁止机构的机器鱼 | `/servo/target_states` |
| `advance_teleop.launch` | 普通舵机加扩展电机版本 | `/servo/target_states`、`/servo/extended_cmds` |

使用 spring 版本：

```bash
roslaunch robotic_fish spring_teleop.launch dev:=pc
```

使用 advance 版本：

```bash
roslaunch robotic_fish advance_teleop.launch dev:=pc
```

使用 sonic 总体程序并同时启动 ADC/DAC：

```bash
roslaunch robotic_fish sonic_teleop.launch \
  dev:=vim4 \
  enable_sensor_io:=true
```

`sonic_teleop.launch` 默认同时启动 rosbag，文件保存在用户主目录，名称类似 `~/sonic_2026-09-01-18-30-00.bag`。录制内容包括：

- ADC 三通道批数据（原始电压、校准电压、校准标识和时间戳）
- DAC 命令与状态、节点诊断
- 手柄输入、IMU、舵机目标命令与状态反馈

改变保存目录或文件名前缀时，目标目录需要预先创建：

```bash
mkdir -p ~/bags
roslaunch robotic_fish sonic_teleop.launch \
  dev:=vim4 \
  enable_sensor_io:=true \
  bag_prefix:=$HOME/bags/sonic_
```

如本次运行不需要录制，可使用 `record_bag:=false`。停止整个 launch 时，rosbag 会同步结束并正常关闭文件。录制后可检查内容：

```bash
rosbag info ~/sonic_*.bag
```

## 6. 手柄控制

控制逻辑使用 `sensor_msgs/Joy` 的数组索引，而不是统一的按键名称。下表描述代码中的默认索引；实际物理按键请通过 `rostopic echo /joy` 确认。

### 6.1 `teleop.py`

| Joy 输入 | 功能 |
| --- | --- |
| `axes[1]` | 前进/后退速度 |
| `buttons[4]` + `axes[2]` | 连续转向及转向角度 |
| `buttons[6]` + `axes[3]` | 单舵机控制 |
| `buttons[7]` + `axes[4]` | 单舵机控制 |
| `buttons[0]` | 与扳机组合时选择另一舵机 |
| `axes[5]` | 鳍角控制输入 |
| `buttons[5]` | 保持当前速度 |
| `axes[9]` | 左/右姿态 |
| `axes[10]` | 两个舵机回中同步 |
| `buttons[1]` | 返回普通模式并回中 |
| `buttons[2]` | 记录当前 IMU 目标角 |
| `buttons[3]` | 启用 IMU 航向反馈 |
| `buttons[10]`、`buttons[11]` | 演示模式切换 |

### 6.2 `spring_teleop.py`

| Joy 输入 | 功能 |
| --- | --- |
| `axes[1]` | 前进/后退速度 |
| `buttons[6]`/`buttons[7]` | 单舵机控制 |
| `axes[9]` | 左/右姿态 |
| `axes[10]` | 舵机回中同步 |
| `buttons[2]` | 锁止 |
| `buttons[1]` | 解锁 |
| `buttons[5]` | 弹簧蓄能（受舵机负载限制） |

### 6.3 `advance_teleop.py`

| Joy 输入 | 功能 |
| --- | --- |
| `buttons[0]` | 进入巡航模式 |
| `buttons[1]` | 停止并发送停止命令 |
| `buttons[2]` | 用当前 IMU/电机状态初始化参考值 |
| `buttons[3]` | 启动自动快速起步 |
| `axes[2]` | 舵机方向输入 |
| `buttons[5]` | 转向控制模式 |
| `buttons[4]` + `buttons[5]` | 快速转向 |
| `buttons[6]`/`buttons[7]` + `axes[3]`/`axes[4]` | 电机加速/减速 |

## 7. ROS 话题与参数

### 7.1 主要话题

| 方向 | 话题 | 消息类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/joy` | `sensor_msgs/Joy` | 手柄输入 |
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist` | 外部速度命令 |
| 订阅 | `/servo/states` | `spinal/ServoStates` | 舵机状态反馈 |
| 订阅 | `/imu` | `spinal/Imu` | IMU 状态 |
| 发布 | `/servo/target_states` | `spinal/ServoControlCmd` | 舵机目标命令 |
| 订阅（spring） | `/servo/torque_states` | `spinal/ServoTorqueStates` | 舵机扭矩状态 |
| 订阅（advance） | `/servo/extended_states` | `spinal/ServoExtendedStates` | 扩展电机状态 |
| 发布（advance） | `/servo/extended_cmds` | `spinal/ServoExtendedCmds` | 扩展电机命令 |

常用检查命令：

```bash
rostopic list
rostopic echo /joy
rostopic hz /servo/states
rostopic echo /servo/states
rostopic echo /imu
```

### 7.2 常用私有参数

三个控制节点均使用私有参数，例如：

- `~joy_dead_zone`：手柄死区，默认 `0.1`
- `~vel_rate`：速度缩放，普通/spring launch 默认 `250`
- `~max_val`：最大命令值，launch 默认 `250`
- `~max_turn_angle`：最大转向位置偏移，默认 `1024`

可通过 launch 参数或命令行私有参数调整。首次调试真实机构时，应从较小的 `max_val` 开始。

## 8. 键盘和舵机测试工具

启动通信桥后，可以直接运行键盘测试节点：

```bash
rosrun robotic_fish servo_keyboard_cmd.py
```

按键说明会在终端显示，使用 `Ctrl+C` 退出。该程序需要在真实终端中运行，不能将标准输入重定向到文件。

使用 `pynput` 监听键盘的测试程序：

```bash
rosrun robotic_fish loop_send_servo_cmd.py
```

循环发送正反舵机命令（参数为循环次数）：

```bash
rosrun robotic_fish loop_send_servo_cmd.bash 10
```

这些测试程序会直接向执行器发布命令，运行前必须确认舵机编号、机械限位和急停方式。

## 9. USB 相机

相机辅助 launch 默认使用 `/dev/video2`、1280×720、30 FPS，并打开 `image_view`：

```bash
roslaunch robotic_fish camera_for_calibration.launch
```

检查相机设备：

```bash
v4l2-ctl --list-devices
ls -l /dev/video*
```

如果摄像头不是 `/dev/video2`，请修改 `launch/camera_for_calibration.launch` 中的 `video_device`。

## 10. 常见问题

### 找不到 `robotic_fish`

```text
[rospack] Error: package 'robotic_fish' not found
```

重新编译并 source 正确工作空间：

```bash
cd ~/ros/jsk_aerial_robot_ws
catkin build robotic_fish
source devel/setup.bash
rospack find robotic_fish
```

### 构建 `spinal` 时找不到 `rosserial_client`

```text
ModuleNotFoundError: No module named 'rosserial_client'
```

确认工作空间中存在 rosserial 源码，或安装对应的 Noetic 包：

```bash
sudo apt install ros-noetic-rosserial-client ros-noetic-rosserial-server
catkin build spinal robotic_fish --force-cmake
```

### 无法打开串口

检查设备名、用户组和是否被其他进程占用：

```bash
ls -l /dev/ttyUSB0 /dev/ttyS4 2>/dev/null
groups
lsof /dev/ttyUSB0 2>/dev/null
```

### 没有 `/joy` 数据

```bash
ls -l /dev/input/js0
rosrun joy joy_node _dev:=/dev/input/js0
rostopic echo /joy
```

如果手柄对应 `js1`，需要修改 launch 文件中的 `joy_node/dev` 参数或单独启动 `joy_node`。

## 11. 安全注意事项

- 首次测试时抬离水面、拆除高风险传动件或对机构进行可靠固定。
- 先确认 `/joy`、`/servo/states` 和 `/imu` 正常，再启动控制节点。
- 从较小的 `max_val` 开始，确认方向、零位、舵机编号和机械限位。
- 保持可立即断电的急停手段，不要仅依赖软件停止。
- 键盘测试和 `rostopic pub` 会绕过部分控制保护，只应用于低功率、有人值守的调试。
- 修改手柄映射后，先观察发布命令，再连接实际执行器。

## 12. 开发流程

只修改 `src` 下的源码：

```bash
cd ~/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot
git status
git diff -- robots/robotic_fish
```

修改后重新构建：

```bash
cd ~/ros/jsk_aerial_robot_ws
catkin build robotic_fish
source devel/setup.bash
```

确认无误后提交：

```bash
cd ~/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot
git add robots/robotic_fish
git commit -m "Document or update robotic fish"
```
