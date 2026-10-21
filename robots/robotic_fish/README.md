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
| 下位机串口 | `/dev/robotic_fish_spinal` | `/dev/robotic_fish_spinal` |
| 串口波特率 | `921600` | `921600` |
| 手柄 | `/dev/input/js0` | `/dev/input/js0` |

检查设备：

```bash
ls -l /dev/robotic_fish_spinal
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

默认连接 `/dev/robotic_fish_spinal`，波特率为 `921600`。现场需核实设备链接。

### 5.2 VIM4 串口模式

当前 teleop launch 文件在 `dev:=vim4` 时不会自动包含 `spinal/bridge.launch`，因此需要分两个终端启动。

终端 1，启动 `/dev/robotic_fish_spinal` 通信桥：

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
| `joy.launch` | PC 单独连接手柄 | `/joy` |
| `sonic_vim4.launch` | VIM4 集中运行声学 IO、串口桥和单舵机控制 | `/servo/target_states`，接收 PC 的 `/joy` |
| `sonic_teleop.launch` | 单舵机控制，可选本机手柄 | `/servo/target_states` |

使用 spring 版本：

```bash
roslaunch robotic_fish spring_teleop.launch dev:=pc
```

使用 advance 版本：

```bash
roslaunch robotic_fish advance_teleop.launch dev:=pc
```

声学采集与处理已经合并到 `robotic_fish_sonic`。趋声实验推荐采用
[VIM4 集中控制、PC 只运行手柄](#65-周期趋声控制)的分工。
`sonic_vim4.launch` 组合启动 IO、串口桥、运动和 Share 显示，不启动手柄或录包。
`sonic_teleop.launch` 默认同时启动运动和手柄，使用 `enable_joy:=false` 时只启动运动。
只运行一个运动命令发布者、一个手柄发布者和一个串口桥。

录包建议在 VIM4 的另一终端运行，让录制时间与 ADC、运动状态使用同一台主机的时间：

```bash
roslaunch robotic_fish record.launch
```

`record.launch` 默认将 bag 保存在用户主目录，前缀为 `sonic`。录制内容包括：

- ADC 三通道批数据（原始电压、校准电压、校准标识和时间戳）
- DAC 执行结果、增益状态和有效配置
- 每个控制周期的运动状态、周期趋声判断、Share 快照及在线调参事件
- 完整运动／声学控制配置快照，以及诊断和 ROS 日志
- 手柄输入、IMU、舵机目标命令与状态反馈

诊断默认录制，可用 `record_diagnostics:=false` 关闭。固定窗口的通道比较、方向概率话题
也在订阅列表中；只有另外启动 `quadrant.launch` 等发布者时才有消息，周期趋声不依赖它们。

新增记录话题如下，不需要按 Share 才能保存：

| 话题 | 内容 | 发布时机 |
| --- | --- | --- |
| `/robotic_fish/sonic/motion_state` | 模式、原因、输入错误、目标位置及整数命令、请求／实际偏置与摆幅、摆频、相位、Joy 年龄、ADC 时序与质量 | 每个控制周期，默认 50 Hz，首次 Joy 前也发布空闲状态 |
| `/robotic_fish/acoustic/steering_state` | 三通道周期均值、L-R 电压差、归一化／补偿后判据、窗口、参数、偏置、当前方向及历史决策标记 | 声学功能启用时，每个控制周期 |
| `/robotic_fish/sonic/config` | 舵机范围、初始摆幅、键位、控制频率、Joy／ADC 时间限制、阈值／偏置／补偿、待生效请求及主机信息 | 启动与配置变更时，锁存，支持中途开始录包 |
| `/robotic_fish/acoustic/steering_events` | 调参请求的接受、拒绝、被替换、实际生效及关机取消；含请求编号、数值、生效周期 | 对应事件发生时 |
| `/rosout`、`/diagnostics` | 模式切换日志和硬件／增益诊断 | 按原节点发布节奏 |

`motion_state.command_published=false` 表示本次没有发送舵机命令，此时目标位置为 NaN；
手动／停止时没有摆尾相位，相位与摆尾时长也为 NaN。目标和摆幅等使用舵机刻度，
相位单位为弧度、范围 `[0,2*pi)`。ROS 时间戳与本机 `monotonic_s` 一并保存，
接收年龄、控制间隔用单调时钟，不能拿不同主机的单调时钟直接相减。

`adc_source_age_at_receipt_s` 是控制节点接收该组时的 ROS 时间减采样时间，
`adc_source_age_s` 是当前控制周期的 ROS 时间减采样时间；未来样本可出现负值。
`adc_receipt_age_s` 与 `adc_receipt_interval_s` 分别是本机接收年龄和最近两组接收间隔。
无数据／结构无效时不可计算的值为 NaN；`adc_quality_flags` 使用 `acoustic_core.Status`
的采样质量位，时间问题另外由 `adc_quality` 标出。

`steering_state.has_decision=false` 表示尚无有效周期结果，均值和差值为 NaN。
`decision_current=false` 时保留的均值与 `decision_direction` 是历史结果；
当前动作以 `direction` 和运动状态为准，例如 `holding`、`warmup` 或 `stopped`。
调参请求编号在每个运动进程内递增，事件只覆盖录包期间；锁存配置包含当前参数和待生效请求，
便于中途录包恢复当时状态。关机阶段的最后消息按 ROS 通信尽力发送。

本次新增消息并扩展了 `AcousticSteeringState`，两端需重新编译
`robotic_fish_sonic robotic_fish`、source 并重启相关节点。旧 bag 仍保留原有嵌入式消息定义。

改变保存目录或文件名前缀时，目标目录需要预先创建：

```bash
mkdir -p ~/bags
roslaunch robotic_fish record.launch bag_prefix:=$HOME/bags/sonic
```

不需录制时不启动 `record.launch`，也可使用该入口的 `enabled:=false`。
停止录包终端的 launch 后检查：

```bash
rosbag info ~/sonic_*.bag
```

## 6. 手柄控制

控制逻辑使用 `sensor_msgs/Joy` 的数组索引，索引从 0 开始。
以下记录本次提供的 DS4 手柄映射；更换驱动或连接方式后，可通过
`rostopic echo /joy` 核对实际映射。后续各控制程序的功能表使用这些索引。

### DS4 输入索引映射

#### 轴：`axes`

| Joy 输入 | 物理输入／传感量 | 数值约定 |
| --- | --- | --- |
| `axes[0]` | 左摇杆左右 | 左 `+1`，右 `-1` |
| `axes[1]` | 左摇杆前后 | 前 `+1`，后 `-1` |
| `axes[2]` | 右摇杆左右 | 左 `+1`，右 `-1` |
| `axes[3]` | L2 扳机模拟量 | `+1` → `-1` |
| `axes[4]` | R2 扳机模拟量 | `+1` → `-1` |
| `axes[5]` | 右摇杆前后 | 前 `+1`，后 `-1` |
| `axes[6]` | IMU_roll，x-z 平面 | `[-0.5, 0.5]` |
| `axes[7]` | IMU_pitch，x-y 平面 | `[-0.5, 0.5]` |
| `axes[8]` | IMU_pitch，z-y 平面 | `[-0.5, 0.5]` |
| `axes[9]` | 十字键左右 | 左 `+1`，松开 `0`，右 `-1` |
| `axes[10]` | 十字键前后（上／下） | 前／上 `+1`，松开 `0`，后／下 `-1` |
| `axes[11]`～`axes[13]` | IMU ACC，加速度输入 | 本次未指定范围 |

摇杆中立值为 `0`。L2、R2 分别同时提供模拟轴 `axes[3]`、`axes[4]`
和下表的数字按键 `buttons[6]`、`buttons[7]`，两种输入不可混淆。
IMU 名称、平面和范围按本次提供的映射记录。

#### 按键：`buttons`

按键值为 `1` 表示按下，`0` 表示松开。

| Joy 输入 | 物理按键 |
| --- | --- |
| `buttons[0]` | Square，方块 `□` |
| `buttons[1]` | Cross，叉号 `×` |
| `buttons[2]` | Round，圆圈 `○` |
| `buttons[3]` | Triangle，三角 `△` |
| `buttons[4]` | L1 |
| `buttons[5]` | R1 |
| `buttons[6]` | L2 |
| `buttons[7]` | R2 |
| `buttons[8]` | Share |
| `buttons[9]` | Option |
| `buttons[10]` | 左摇杆按下（L3） |
| `buttons[11]` | 右摇杆按下（R3） |
| `buttons[12]` | PS |
| `buttons[13]` | Space（原映射名称） |

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

### 6.4 `sonic_teleop.py`：单舵机运动接口

`sonic_teleop` 只输出舵机 0 的位置目标。默认关闭声学控制，
可通过 `acoustic_steering:=true` 启用下节的声源跟踪功能，再按圆圈进入跟踪模式。
摆尾公式为 `center + applied_bias + effective_amplitude * sin(2*pi*f*t)`。
`sonic_motion.py` 独立计算轨迹和模式，ROS 脚本统一发布
`/servo/target_states`。不要同时运行其他向该话题发布运动命令的程序。

| 默认 Joy 输入 | 物理输入 | 功能 |
| --- | --- | --- |
| `axes[2]` | 右摇杆左右 | 手动／普通自动模式的目标偏置为 `-axis * servo_range`，死区内为零；声源跟踪模式中忽略该方向输入 |
| `buttons[3]` 的新按下 | 三角 `△` | 进入／重新定相普通自动摆尾，由右摇杆调整中心偏置 |
| `buttons[2]` 的新按下 | 圆圈 `○` | 进入声源跟踪模式；已在跟踪时再次按下保持模式和相位，不执行开关切换 |
| `buttons[0]` 的新按下 | 方块 `□` | 进入手动位置模式，或从停止恢复手动 |
| `buttons[1]` | 叉号 `×` | 停止摆尾、回中并锁存；优先于其他输入 |
| `buttons[8]` 的新按下 | Share | 发布状态快照，由 VIM4 的 IO 终端显示对齐表格；长按只打印一次 |
| `buttons[6]` 或 `buttons[7]` 按住 | L2 或 R2 | 用 R2 模拟轴 `axes[4]` 增幅、L2 模拟轴 `axes[3]` 减幅，沿用原映射 |

各模式通过独立按键选择：方块手动、三角普通自动、圆圈声源跟踪；
叉号 `×`（Cross）统一终止当前运动、回中并锁存，优先于所有模式键。

启动后默认手动模式；首个有效手柄输入之前不发送舵机命令。
手动模式下，摇杆直接决定位置目标 `servo_center - axis * servo_range`，
死区内回到中心，不施加偏置变化速率限制；目标在下一控制周期发布。
普通自动模式下，摇杆直接调整正弦摆尾的中心偏置，不施加偏置变化速率限制。
声源跟踪模式采用声学偏置并忽略方向摇杆；手动模式保持直接映射。
`acoustic_steering:=true` 只使跟踪功能可用，不改变三角键的普通自动功能。
未启用跟踪功能时按圆圈会停止并报告 `tracking_unavailable`，不会进入其他模式。
声学偏置按周期直接更新，不增加限速或平滑。
手动、普通自动和声源跟踪模式均以默认 50 Hz 的墙上时间循环输出。
轨迹和超时使用 `time.monotonic()`，不依赖 ROS `/clock`；
ROS 仿真时钟暂停或回拨不会暂停运动或手柄超时检查。
这也意味着不能依靠暂停 `/clock` 停止这个硬件控制节点，应使用停止键。

停止、输入异常或手柄超时后进入 `stopped`：目标偏置和实际偏置清零，
取消正弦摆动，从下一控制周期起发送中心位置。
需要先松开停止／普通自动／手动／跟踪模式键和调幅键，将方向轴回到死区，
收到一个完整有效的释放帧，再在回中状态重新按三角、方块或圆圈选择模式。
保持原来的按键、恢复连接或单纯收到中立消息均不会恢复运动。
这相对原版“停止后直接回到手动模式”是有意的行为变化。

| 参数／launch 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `joy_timeout_s` | 0.5 | 最后一个有效 Joy 消息的接收超时，秒 |
| `update_rate` | 50 | 控制循环目标频率，Hz；应大于摆尾频率的两倍 |
| `joy_autorepeat_rate` | 20 | launch 为 joy_node 设置的重复发布频率 |
| `enable_joy` | true | `sonic_teleop.launch` 是否同时启动本机手柄；VIM4 分机控制时关闭 |
| `joy_dev` | `/dev/input/js0` | 手柄所在主机的设备路径；也用于独立的 `joy.launch` |
| `steering_axis` | 2 | 方向轴索引 |
| `amplitude_decrease_axis` / `amplitude_increase_axis` | 3 / 4 | 调幅轴索引 |
| `stop_button` / `auto_button` / `manual_button` | 1 / 3 / 0 | 模式按键索引 |
| `tracking_button` | 2 | 声源跟踪模式按键索引，默认圆圈 |
| `status_button` | 8 | 状态查看按键索引，默认 Share；不能与运动按键重叠 |
| `status_local_echo` | false | 同时在运动节点所在终端显示 Share 快照 |
| `amplitude_buttons` | `[6, 7]` | 调幅使能按键列表 |

中心 2048、软件范围 ±1024、启动文件摆频 1.5 Hz、默认摆幅 512 和
调幅速率 64 刻度／秒保留。直接 rosrun 时的摆频默认仍为 2 Hz。
位置、幅值和偏置使用舵机刻度，软件范围不等于已验证的机械限位。
实际摆幅受当前中心位置的剩余余量限制；靠近位置边界时摆幅会缩小。
常规日志显示模式切换和停止原因；详细数据通过 Share 查看。
不再每秒打印 `Sonic manual: bias...`，逐周期声学日志改为 debug 级别。

```bash
roslaunch robotic_fish sonic_teleop.launch
```

上述新增默认值是软件调试起点，尚未经本次水中实验确认。
Joy 只检查映射到的轴和按键；短消息、非有限轴、越界轴或无效按键使运动停止，
不刷新有效输入时间。看门狗检查消息是否继续到达，不能证明无线手柄的物理
链路仍有效；现场应实际验证手柄断连时驱动的行为。进程／串口完全失联时，
主机回中命令的送达也不能由此节点保证。

在仓库根目录验证（编译并 source 后）：

```bash
python3 -m unittest discover -s robots/robotic_fish/test -v
python3 robots/robotic_fish/test/sonic_ros_smoke.py
```

通信测试只启动本机端口 11428 的独立 master、合成 Joy 发布者和运动节点，
命令重映射到测试话题，不启动桥或执行器。该端口需空闲。
支持 `--node-script` 指定安装后的 `sonic_teleop.py` 进行安装空间验证。
本次检查结果和现场待验证项见 [运动验证记录](MOTION_VALIDATION.md)。

### 6.5 周期趋声控制

推荐部署为：**PC 只连接手柄，VIM4 运行 ADC／DAC、增益控制、串口桥、
周期趋声及运动节点、Share 显示**。采样时间戳与控制节点的 ROS 当前时间均来自 VIM4；
摆尾相位、接收超时仍用 VIM4 的单调时钟。Joy 消息只按 VIM4 本机接收时间计时，
不会使用 PC 的 Joy 时间戳计算超时。

两端更新代码、编译 `robotic_fish_sonic robotic_fish` 并 source 各自工作空间。
网络配置继续使用 `rosfish`：两端 `ROS_MASTER_URI` 指向 VIM4（本次为
`http://192.168.1.168:11311`），每端 `ROS_IP`／已设置的 `ROS_HOSTNAME`
应为该端本机地址。关闭上一轮 PC 运动 launch 和重复的 VIM4 IO／桥后，按以下方式启动。

VIM4：

```bash
roslaunch robotic_fish sonic_vim4.launch acoustic_bias:=128
```

该入口默认允许声源跟踪，手动增益模式的初始目标为 0 V；仍需用十字键调节增益，
或通过 `manual_gain_voltage:=...` 设置已经确认的现场值。启动后仍为手动模式。

PC（手柄保持连接）：

```bash
roslaunch robotic_fish joy.launch
```

PC 上不再启动运动节点。手柄设备可用 `joy_dev:=/dev/input/js1` 选择，重复发布默认 20 Hz。
VIM4 启动日志打印 `Sonic host=...`、实际 ADC 最大年龄及未来时间戳容差；
也可在任一端执行 `rosnode info /sonic_teleop_node`，确认节点 URI 指向 VIM4。

若 VIM4 已单独运行 `robotic_fish_sonic io.launch`，可保留 IO，只在 VIM4 新终端运行：

```bash
roslaunch robotic_fish sonic_teleop.launch enable_joy:=false \
  acoustic_steering:=true acoustic_bias:=128
```

此时不要再启动组合入口 `sonic_vim4.launch`。原 `sonic_teleop.launch` 的默认组合方式仍可用于单机调试。

默认仍是手动模式。按圆圈 `○`（`buttons[2]`）进入声源跟踪后，使用同方向的正向过零点
（相位 `2*pi*k`）判断，首个完整周期内偏置为零。
直接订阅 `/robotic_fish/adc/samples`，按 `W=1/f` 统计上一完整周期的
校准左右电压均值，施加电压补偿后再归一化。无需另启 `quadrant.launch`；
它原有的固定窗口比较与本功能相互独立。
本次硬件前端已确认输出整流／检波后的包络或幅值电压，周期均值用于比较
平均声学响应幅值，不是物理声强。更换为原始交流波形输入时需重新设计特征提取。

令 `delta=acoustic_lr_voltage_offset_v`（单位 V，默认 0），对上一完整周期均值计算：

```text
L_corrected = L + delta
R_corrected = R - delta
mean = (L_corrected + R_corrected) / 2 = (L + R) / 2
d_corrected = (L_corrected - R_corrected) / mean
```

`threshold=0.15` 表示左右补偿电压差达到两侧平均值的 **15%** 时转向，
按 `d_corrected>=threshold` 左转、`d_corrected<=-threshold` 右转，其余直行。
例如 `L_corrected=1.075 V`、`R_corrected=0.925 V`，平均值为 `1 V`，差值为 `0.15 V`，
恰好达到 15% 时也左转。这里的百分比以两侧平均值为基准；此时强侧约为弱侧的 1.162 倍。
左右完全相等时始终直行，包括 `threshold=0`。
相同阈值下，新公式比原来的除以 `L+R` 更灵敏；若要保持原来 `0.15` 的转向边界，改用 `0.30`。

正值使判断偏向左侧，负值使判断偏向右侧；左右电压差总共改变 `2*delta`。
仅在声源居中、左右应相等的条件下，用多个稳定周期估计 `delta=(R-L)/2`。
例如居中时 `L=0.07145 V`、`R=0.04772 V`，对应 `delta=-0.011865 V`。
这是固定电压差补偿；更换 DAC 增益或工作条件后应重新检查基线。
原始／校准 ADC 电压与测得周期均值保持原值；左右电压补偿只用于判据计算，
派生结果单独记录，Share 在判据区显示补偿前后的差异。
补偿后的相对差值达到正／负阈值时取左／右转偏置，其余取零。
每次取固定 `-C/0/+C`，不累加；整个周期保持该偏置，不重置摆尾相位。
目标按原控制循环约 50 Hz 发布，决策频率为摆频 `f`，实际过零处理发生在
边界后的首个控制周期。接受中心偏置跳变。

`left_bias_sign=1` 表示正偏置用于左转，`-1` 表示负偏置用于左转；
该符号、阈值和偏置初值都需要现场验证，不代表已确认的水中转向效果。
偏置使用舵机刻度，靠近软件边界时仍按原规则缩小摆幅。

现场通过同一条命令调整阈值、转向偏置及左右补偿（无需重启，在下一次有效周期决策生效）：

```bash
rosservice call /sonic_teleop_node/set_acoustic_steering \
  "{threshold: 0.15, bias: 128.0, lr_voltage_offset_v: -0.01}"
```

命令终端直接返回请求处理时的状态快照。例如当前正使用阈值 0.1、偏置 256、无补偿，
且正在左转时，返回内容类似：

```yaml
success: True
message: "accepted; applies at next valid full-cycle decision"
request_id: 1
difference_reference: channel_mean
motion_mode: tracking
status: active
reason: tracking_button
direction: left
adc_quality: valid
command_bias: 256.0
active_threshold: 0.1
active_bias: 256.0
active_lr_voltage_offset_v: 0.0
parameters_pending: True
pending_request_id: 1
pending_threshold: 0.15
pending_bias: 128.0
pending_lr_voltage_offset_v: -0.01
```

`success` 表示请求被接受；`active_*` 是正在使用的参数，`pending_*` 是等待生效的参数，
`command_bias` 是控制器当前偏置。服务查询不推进摆尾周期、不发送舵机命令，也不自动进入跟踪。
手动／停止或数据预热期间可以接受参数，继续等待下一次有效完整周期。
参数实际生效后，VIM4 运动终端打印一次 `Acoustic steering applied: request=... cycle=...`
以及本次生效的三项参数，可用请求编号与调用返回、录包事件对应。
拒绝请求时也返回当前状态和此前保留的待生效参数；没有待生效请求时，
`parameters_pending=false`、`pending_request_id=0`，三个 `pending_*` 数值为 NaN。
本次替换了服务补偿字段并扩展状态消息，两端需重新编译两包、source 并重启相关节点后使用。
旧的无量纲 `lr_offset` 和启动参数 `acoustic_lr_offset` 已移除，旧数值不能直接沿用为伏特。

每次调用请同时填写三项：阈值范围 `[0,2]`（0%～200%），偏置范围 `[0,servo_range]`，
电压补偿须为有限有符号数，`lr_voltage_offset_v: 0.0` 表示取消补偿。
整组校验通过后，在下一有效周期决策一起应用；任何一项无效都不覆盖此前已接受的配置。
多次有效请求以最新一组为准；`parameters_pending` 表示等待生效，预热或质量等待期间继续暂存。
每个周期还要求 `L+delta>=0`、`R-delta>=0`，即 `delta` 位于该周期的 `[-L,R]` 内。
补偿越界时显示 `voltage_compensation_out_of_range`／`HOLDING`，保持摆尾和上一偏置，
整组三项待生效参数继续暂存；后续完整周期恢复有效或替换参数后自动继续判断。
不会把负电压截成零。尚无有效判断时保持零偏置；Cross、Joy／ADC 超时及饱和等检查继续生效。
补偿保持摆尾相位和转向偏置上限，头部电压保持原值。
重启后使用启动参数；例如在 VIM4 的启动命令中增加 `acoustic_lr_voltage_offset_v:=-0.01`。
这三项支持运行时服务；`rosparam set` 不会更新控制器，
摆频和方向符号通过 launch 设置，修改后重启并重新积累周期数据。

进入声源跟踪模式前需要已有新鲜、结构有效且未饱和的三通道数据。
校准或 DAC 状态尚未就绪时，可以进入跟踪并以零偏置摆尾等待；
只有校准成功、DAC 增益已确认的数据才用于声学方向判断。
`sonic_vim4.launch` 与 `sonic_teleop.launch` 默认最大数据年龄为 0.5 秒，通过 `acoustic_max_age_s` 设置；
启动时显式写入节点参数，无需先单独执行 `rosparam set`。例如沿用本次现场设置：

```bash
roslaunch robotic_fish sonic_vim4.launch acoustic_bias:=128 \
  acoustic_max_age_s:=0.5 acoustic_future_tolerance_s:=0.05
```

该值仅在启动时读取，修改后需重启运动 launch。直接运行脚本而未设置私有参数时，
仍沿用 0.2 秒初值。最大采样间隔为 0.2 秒，未来时间戳容差默认 0.05 秒，
通过 `acoustic_future_tolerance_s` 显式写入；同机采集和控制继续保留该检查。
左右电压和的低信号阈值为 0.001 V。
采样不必恰好命中周期端点，但两端及内部的间隔均不得超过允许间隔，
且至少有两组数据。启动时等待完整 `1/f` 周期，不采用原比较节点的 80% 预热规则。

跟踪中可直接使用方向键调整增益，无需停止或重新按圆圈。已确认的 DAC 增益变化时，
保持跟踪模式、摆尾相位和上一有效偏置（尚无决策时为零），清空旧增益的窗口。
三个通道增益不一致的过渡组只用于检查数据是否持续到达，不参与电压均值。
状态为 `gain_warmup`；新增益停止变化后，等待只含该增益的完整 `1/f` 周期，
在原来的同方向过零点恢复方向判断。受周期对齐影响，通常需等待一到两个摆尾周期。
连续调整会持续重新积累窗口，声学阈值／偏置的待生效配置也等到下一有效决策才应用。

校准越界回退或 DAC 增益未知时同样保持跟踪和上一偏置，清空统计窗口，
跳过这些采样组。Share 显示 `HOLDING`，状态及 ADC quality 保留具体原因
`calibration_fallback`／`gain_unknown`；均值仍是历史决策，不使用回退原始电压判断方向。
数据恢复有效后进入 `data_warmup`，等待完整对齐周期再自动恢复判断，无需重新按键。

声源跟踪模式下，格式／数值无效、饱和、有效信号过低、数据超时、有效校准标识变化、
非窗口重建导致的周期不完整或错过整个周期时，
立即停止并锁存，不等下一次过零，也不会随数据恢复自行重启。
新数据先于控制循环到达时，仍保留已发生的超时／采样间隔异常，不能覆盖停止条件。
释放按键、摇杆回中后重新按圆圈恢复跟踪，重新积累完整周期。
三角可直接选择普通自动模式，方块可直接选择手动模式；圆圈不用于关闭跟踪。
手动和普通自动模式不受声学输入故障阻断，原有 Joy 超时和 Cross 停止键始终生效。
Cross、Joy 超时和 ADC 时间检查在上述等待状态下仍生效。
状态中的 `motion_mode` 分别为 `manual`、`auto`、`tracking`、`stopped`。

状态话题包含模式／原因、周期序号、实际窗口、左右均值、差值、参数及输出偏置；
`normalized_difference` 保留补偿前的归一化差值，`lr_voltage_offset_v` 为已生效电压补偿，
`compensated_left_mean`／`compensated_right_mean` 保存补偿后的均值，
`compensated_voltage_difference` 保存补偿后的电压差，
`compensated_difference` 为最近决策使用的补偿后相对差值，两个差值均以两侧平均值为分母。
消息和服务反馈通过 `difference_reference=channel_mean` 标明基准；数值仍为比例，`0.15` 表示 15%。
锁存配置 `schema_version=3`，保存相同基准和 `threshold_inclusive=true`。
`decision_updated` 仅在新决策时为真。停止后可保留上一决策的数值和窗口时间，
必须结合 `motion_mode`、`status` 和 `sample_stamp` 解读。
`record.launch` 已包含该状态话题。详细软件验证见
[周期趋声验证](../robotic_fish_sonic/docs/cycle_steering_validation.md)。

### 6.6 Share 查看系统状态

按一下 **Share（`buttons[8]`）**，VIM4 运动节点生成一次快照，经
`/robotic_fish/sonic/status_snapshot`（`std_msgs/String`）发送，
由 **VIM4 的 `sonic_vim4.launch` 终端**打印（单独启动 IO 时为 `io.launch` 终端）。
长按不重复打印，松开后再按可再次查看。
快照不锁存，显示节点启动后等待下一次 Share，避免把旧结果当作新状态。
周期统计、判断及快照均在 VIM4 完成，显示节点只打印快照。输出包括：

- 当前运动模式、声学状态和判断方向；方向行同时显示 `bias=(当前输出偏置/修改上限)`。
- ADC 数据质量，以及 ADC0／ADC1／ADC2（头／左／右）的最新原始电压。
- `Raw cycle mean` 为上一完整摆尾周期内三个通道的原始电压均值。
- `Calibrated cycle mean` 为同一批采样经校准文件处理后的电压均值，先校准每次采样再平均。
- `LR offset [V]` 为最近决策使用的电压补偿；有待生效补偿时才增加 `Pending LR [V]`。
- `Diff before [%]` 为左右电压补偿前的相对差，已经使用上述校准文件后的左右周期均值。
- `Diff after/T [%]` 为左右电压补偿后的相对差／阈值百分数，例如 `+18.00 / 15.00` 表示左侧达到判据。

校准文件修正通道测量值；`lr_voltage_offset_v` 仅调整左右差异判据，不写回 ADC 数据，
也不改变电压表里的三行数据。原始与校准周期均值使用同一窗口、同一批有效采样；
若历史消息没有原始电压，原始均值显示 `--`，不会用校准值冒充原始值。
正常跟踪输出 16 行。停止时额外显示 `Reason`；无完整周期或保留历史结果时显示 `Cycle` 提示。
时间戳、数据年龄、周期编号／窗口／样本数和上次方向移出终端显示，
原有 ADC、周期状态、连续运动状态、配置与事件话题仍完整录包。

例如，进入跟踪并完成一个周期后：

```text
+----------------------------------------------------------------------------+
|                            SONIC STATUS / SHARE                            |
+----------------------------------------------------------------------------+
| Mode / state    : tracking / active                                        |
| Direction       : LEFT; bias=(+128.0/1024.0)                               |
| ADC quality     : valid                                                    |
+----------------------------------------------------------------------------+
| Voltage [V]            |     Head (ADC0) |     Left (ADC1) |  Right (ADC2) |
| Raw instant            |         0.50000 |         0.70000 |       0.40000 |
| Raw cycle mean         |         0.49000 |         0.68000 |       0.38000 |
| Calibrated cycle mean  |         0.50000 |         0.70000 |       0.40000 |
+----------------------------------------------------------------------------+
| Diff before [%] : +54.55                                                   |
| Diff after/T [%]: +50.91 / 15.00                                           |
| LR offset [V]   : -0.01000                                                 |
+----------------------------------------------------------------------------+
```

表格宽 78 列，电压统一保留 5 位小数。颜色用于辅助识别：数值青色，
`LEFT` 黄色、`RIGHT` 洋红色、`STRAIGHT` 绿色，异常／停止红色，预热／历史结果黄色。
`io.launch` 默认 `enable_status_console:=true status_color:=auto`：
交互终端启用颜色；重定向输出、`TERM=dumb` 或设置了 `NO_COLOR` 时输出纯文本。
可显式设置 `status_color:=always` 或 `status_color:=never`，例如在 VIM4 强制彩色：

```bash
roslaunch robotic_fish sonic_vim4.launch acoustic_bias:=128 status_color:=always
```

需要单独运行显示节点时，执行 `roslaunch robotic_fish_sonic status_console.launch color:=auto`；
已有组合入口或 `io.launch` 显示节点时直接按 Share 即可。单机调试可给运动 launch 设置
`status_local_echo:=true`。快照话题始终是纯文本，`record.launch` 已包含它。

周期均值与控制判断使用同一批数据，不是 ADC 的 `volt_raw_mean_1s`，也不会因查看而重算或重置相位。
偏置分子为当前实际输出偏置；分母为允许设置的偏置绝对值上限 `servo_range`（默认 1024），
各模式使用同一上限。在线调整声学 `bias` 不会改变分母。
停止时显示 `(+0.0/1024.0)`（默认配置）。
首次进入跟踪尚未完成周期时显示 `UNAVAILABLE` 和 `WARMUP`，均值单元格为 `--`。
停止后保留的周期结果标为 `HISTORICAL`，当前方向为 `STOPPED`；
增益调整期间同样标为 `HISTORICAL`，当前方向为 `HOLDING`，表示继续摆尾并保持旧偏置；
增益重建状态为 `gain_warmup`，校准回退／未知增益时显示具体原因；
数据恢复后状态为 `data_warmup`，恢复有效判断后回到 `active`。
`HOLDING` 及预热状态为黄色，校准回退和未知增益的质量标记仍为红色。
手动／普通自动时为 `NOT_TRACKING`。`HISTORICAL` 表示均值、差异及其补偿来自历史决策，不能作为当前判断。
声学功能未启用时，Share 仍可查看瞬时电压，方向为 `DISABLED`，不产生周期判断。
无 ADC、校准回退、数据过期等情况会明确标出；瞬时回退电压不作为有效校准均值使用。
数据年龄及源时间戳保存在连续状态话题；推荐部署下来自 VIM4。
若仍在 PC 录包，PC 的录制时间与 VIM4 采样时间之差仍受跨机时钟偏差影响，
不能直接将该差值解释为控制节点的样本年龄；两端校时或在 VIM4 录包可方便后续分析。

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
ls -l /dev/robotic_fish_spinal
groups
lsof /dev/robotic_fish_spinal
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
