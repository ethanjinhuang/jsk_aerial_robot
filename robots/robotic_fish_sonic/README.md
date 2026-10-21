# robotic_fish_sonic

机器鱼声学系统的 ROS 1 包，合并原 `robotic_fish_io` 和
`robotic_fish_acoustic`。提供 ADC 采集与校准、DAC 公共增益控制、
原始 ADC 过压保护、无模型通道比较、可选方向模型和历史 bag 工具。
运动及手柄控制仍属于 `robotic_fish`，下位机通信属于 `spinal`。
DS4 的完整轴／物理按键索引见
[手柄输入映射](../robotic_fish/README.md#ds4-输入索引映射)。

## 构建与迁移

在工作空间根目录执行：

```bash
catkin build robotic_fish_sonic robotic_fish --force-cmake
source devel/setup.bash
rospack find robotic_fish_sonic
rosmsg show robotic_fish_sonic/AdcSample
```

清理旧包的构建产物，并在所有运行节点的终端重新 source、重启相关节点。
旧源码包已移除，不提供旧包名的启动或 Python 导入兼容层。

| 原接口 | 新接口 |
| --- | --- |
| `robotic_fish_io.msg` / `.srv` | `robotic_fish_sonic.msg` / `.srv` |
| `robotic_fish_acoustic.msg` | `robotic_fish_sonic.msg` |
| `robotic_fish_io.<module>` | `robotic_fish_sonic.<module>` |
| `robotic_fish_acoustic.core` | `robotic_fish_sonic.acoustic_core` |

ROS 话题和服务地址、消息字段、校准 JSON 和默认保护参数保留。
`AdcSample` 包含 14 个字段，含一秒原始电压均值及其就绪标记。
发布者和消费者必须一起更新消息类型。历史 bag 保留原类型，离线工具按
内嵌定义读取；已验证范围见 [VALIDATION.md](VALIDATION.md)。

## 启动

统一启动 ADC、DAC、增益控制和声学比较：

```bash
roslaunch robotic_fish_sonic sonic.launch
```

默认增益模式为 `off`；增益控制节点运行时，原始 ADC 过压保护仍然生效。
模型默认为空，比较结果可用，方向概率保持不可用。
此入口不启动 spinal 桥、手柄、摆尾或录包。

| 入口 | 启动内容 |
| --- | --- |
| `sensor_io.launch` | ADC、DAC、增益控制 |
| `quadrant.launch` | 声学比较和可选方向模型 |
| `sonic.launch` | IO 和声学处理 |
| `io.launch` | spinal 桥和 IO，沿用原用途 |
| `robotic_fish/sonic_vim4.launch` | 在 VIM4 组合运行 IO、桥、运动控制和 Share 显示 |

`sonic.launch` 的 `enable_adc`、`enable_dac`、`enable_gain_control`、
`enable_acoustic` 可分别关闭模块。仅处理外部 ADC 消息时：

```bash
roslaunch robotic_fish_sonic sonic.launch enable_adc:=false enable_dac:=false
```

关闭增益控制节点会同时移除该节点提供的过压降增益保护。

趋声实验采用 VIM4 同机采集和控制，PC 只连接手柄。
在 VIM4 上启动（已包含 IO 和串口桥）：

```bash
roslaunch robotic_fish sonic_vim4.launch acoustic_bias:=128
```

在 PC 上启动：

```bash
roslaunch robotic_fish joy.launch
```

两端使用同一 VIM4 ROS master，并 source 各自更新后的工作空间。
周期统计、运动相位、时间检查及 Share 快照都在 VIM4；PC 的 Joy 时间戳不参与控制超时计算。
手动增益默认目标为 0 V，按原十字键方式调整，或指定已经确认的 `manual_gain_voltage`。
仍由 Circle 进入跟踪、Cross 停止。原 `sonic_teleop.launch` 的组合方式保留；
VIM4 已有 `io.launch` 时，可仅启动运动：`sonic_teleop.launch enable_joy:=false acoustic_steering:=true`。
每台设备的操作与切换说明见[周期趋声控制](../robotic_fish/README.md#65-周期趋声控制)。
只运行一个手柄发布者、一个运动命令发布者和一个 spinal 桥。
录包建议在 VIM4 另开终端运行 `roslaunch robotic_fish record.launch`，
让录制时间与采样、控制时间同源；PC 录包仍需要考虑两端时钟偏差。
录包默认包含连续运动状态、三通道周期结果、锁存控制配置、在线调参事件、诊断和 ROS 日志，
不依赖 Share。详见[录包字段与时序说明](../robotic_fish/README.md#53-控制程序选择)。

## 配置与声学输出

- `config/io.yaml`：ADC、校准表、DAC、增益及保护参数。
- `config/acoustic.yaml`：声学节点私有参数，默认历史窗口 0.5 秒、
  最大间隔 0.2 秒、低电压数值阈值 0.001 V，以及未来时间戳容差 0.05 秒。
- `config/calibration/`：保留的三份独立通道校准表。
- `config/udev/`：DAC 设备规则；spinal 当前路径为
  `/dev/robotic_fish_spinal`，现场需核实对应规则。

IO 配置由 `config:=...` 选择，声学配置由 `acoustic_config:=...` 选择。
`quadrant.launch` 单独使用 `config:=...`。`window_s`、`gap_s`、
`low_signal_v` 的非空启动参数覆盖所选声学 YAML；`model` 通过启动参数选择。
IO 中已有的启动参数覆盖 YAML 的优先级保持不变。

```bash
roslaunch robotic_fish_sonic quadrant.launch window_s:=0.2
rostopic echo /robotic_fish/acoustic/channel_comparison
```

约定 ADC0=头部、ADC1=左侧、ADC2=右侧。输出左右差值
`(L-R)/(L+R)` 和头部与两侧平均差值；电压先求历史窗口均值，再计算比值。
正的左右差值仅表示左通道更强。下游须检查 `has_comparison`、`reliable`、
`status` 和 `sample_stamp`；保留旧结果时采集时间不刷新。
`reliable` 是软件质量标记，不是经过实验验证的测向准确性。
测试目录的合成模型不能用于真实方向判断。
固定窗口比较会拒绝超过 `future_tolerance_s` 的未来采样组，保留旧结果和原时间戳，
标记 `INVALID_INPUT`；心跳发现 ROS 时间回退也会清空窗口并标记 `TIME_RESET`。
两种情况都需要重新积累有效数据完成预热，才会恢复 `reliable=true`。
此处时间容差可在声学 YAML 中设置，周期趋声控制仍使用其独立的时间检查。

## 按摆尾周期进行左右趋声控制

`cycle_steering.py` 提供独立于 ROS 的完整周期统计、输入质量检查和
阈值／固定偏置判定；`robotic_fish/sonic_teleop.py` 提供摆尾相位、手柄
模式和唯一舵机目标出口。该功能默认关闭，启用后由圆圈选择跟踪模式；
三角仍选择普通自动摆尾，方块选择手动，Cross 统一停止。开启方式和现场调参见
[运动包说明](../robotic_fish/README.md#65-周期趋声控制)。

每次同方向过零时读取上一完整 `1/f` 周期的校准 ADC 数据，比较左右
电压均值，选取 `-C/0/+C`，保持一周期。该路径直接使用 ADC 数据，
不经过 `quadrant_node` 的固定窗口，不需要方向训练模型。
电压补偿 `delta=lr_voltage_offset_v` 单位为 V、默认零，使用 `L'=L+delta`、`R'=R-delta`，
再以 `d=(L'-R')/((L'+R')/2)` 与阈值比较，即差值相对两侧平均值的比例。
`threshold=0.15` 表示差异达到平均值的 15%：正向达到阈值左转，负向达到阈值右转，其余直行。
左右相等时始终直行，包括阈值为零。阈值允许 `[0,2]`（0%～200%）；
旧的除以 `L+R` 的阈值乘 2 可保留相同转向边界，例如旧 `0.15` 对应新 `0.30`。
正的电压补偿偏向左、负值偏向右；原始电压和周期均值保持原值。
Share 和状态话题单独保存补偿均值、电压差、相对差判据及待生效电压补偿；
Share 的 `Raw diff [%]` 和 `Diff / T [%]` 使用百分数，消息和服务仍使用比例数值。
补偿使任一侧均值变成负数或非有限值时，显示 `voltage_compensation_out_of_range`／`HOLDING`，
继续摆尾并保持上一偏置，暂存整组待生效设置，在后续有效完整周期恢复；不截断负值。
输入应为整流／检波后的包络或幅值电压；当前硬件前端已由用户确认符合此条件。
周期均值表示平均响应幅值，不是物理声强，也不能直接用于原始交流声波电压。
跟踪中调整已确认的 DAC 增益不再停止运动：保持摆尾相位和上一偏置，清空旧窗口，
跳过混合增益的采样组，待新增益稳定并积累完整对齐周期后继续判断。
等待期间状态为 `gain_warmup`，Share 显示 `HOLDING` 和历史周期结果。
校准越界回退／未知增益同样保持摆尾与上一偏置并暂停决策，Share 显示具体原因及
`HOLDING`；数据有效后以 `data_warmup` 重建完整周期并自动恢复判断。
首次进入跟踪而数据尚不可用时，以零偏置等待。时间检查、饱和、Cross 和 Joy 超时等停止规则仍生效。
VIM4 运动节点按 Share 发布纯文本快照，组合入口及 `io.launch` 默认启动 `status_console.py`，
在 VIM4 终端用对齐表格显示三通道瞬时电压、上一完整周期均值、差值及方向。
`status_color:=auto` 自动检测终端颜色，`always` 强制彩色，`never` 关闭；
`enable_status_console:=false` 可关闭显示。独立显示可运行
`roslaunch robotic_fish_sonic status_console.launch color:=auto`。
具体见 [Share 查看系统状态](../robotic_fish/README.md#66-share-查看系统状态)。
三通道映射仍要求 ADC0=头、ADC1=左、ADC2=右，并要求有效校准和已确认增益。

新增接口：

- `srv/SetAcousticSteering`：统一接收 `threshold`、`bias` 和 `lr_voltage_offset_v`，
  三项整组验证、暂存，在下一有效周期一起生效；补偿为有限有符号伏特值，周期内须满足 `-L<=delta<=R`。
- `msg/AcousticSteeringState`：记录周期窗口、方向差值、偏置、参数及模式状态。
- `msg/SonicMotionState`：记录每个控制周期的运动输出、摆幅／相位、输入年龄及 ADC 质量。
- `msg/AcousticSteeringEvent`：记录调参请求接受／拒绝／替换、实际生效周期及关机取消。
- `/robotic_fish/sonic/config`：复用 `RuntimeConfig`，锁存完整控制配置及待生效请求。
- `/robotic_fish/acoustic/steering_state`：启用功能后的运行状态，可随 bag 记录。

目前摆频由 VIM4 运动启动参数指定，窗口自动取倒数；更换摆频后重启并重新积累完整周期。
ADC 年龄、未来时间戳检查均在 VIM4 本机进行，未来容差继续默认 0.05 秒；
`acoustic_future_tolerance_s` 在启动时显式设置，启动日志显示主机名和实际时间限值。
补偿可通过运动 launch 的 `acoustic_lr_voltage_offset_v` 设置初值，运行时调用
`/sonic_teleop_node/set_acoustic_steering` 并填写三项；具体命令见运动包说明。
调用后直接返回当前模式、数据质量、方向、控制偏置、正在使用及待生效参数；
真正生效时 VIM4 运动终端打印请求编号、周期和参数。服务成功表示接受请求，仍等待有效周期。
旧无量纲 `lr_offset` 及其启动参数已移除，新补偿名称同时用于配置、事件和服务响应。
锁存运动配置的 `schema_version` 更新为 3，包含 `difference_reference=channel_mean` 和包含等号的判定规则。
状态、参数事件和服务反馈也标记差值基准，避免与旧 bag 的总和归一化含义混淆。消息及服务定义已改变，
两端需重新编译并 source 两包后重启相关节点，旧 bag 仍可用嵌入式定义离线读取。
不会在运动底层加入通用偏置限速。验证边界见
[周期趋声验证记录](docs/cycle_steering_validation.md)。

## 离线工具

```bash
rosrun robotic_fish_sonic replay_quadrants.py recording.bag --output /tmp/fish-replay
rosrun robotic_fish_sonic calibrate_from_bag.py sweep.bag --output /tmp/fish-calibration
```

输出目录必须不存在。回放支持 `--window-s`、`--model` 和
`--calibration`；更换校准只修改内存中的解码值。校准工具只有显式提供
`--common-input` 才生成候选表，该选项要求实验确实采用共同电输入，
不能用于将自由游动数据归一化。工具不改写 bag 或部署校准表。

## 测试与限制

编译并 source 后，在仓库根目录执行：

```bash
python3 -m unittest discover -s robots/robotic_fish_sonic/test -v
python3 robots/robotic_fish_sonic/test/ros_smoke.py
python3 robots/robotic_fish_sonic/test/bag_compatibility.py
```

通信测试使用独立本机 master（端口 11427）和合成数据，不启动硬件节点；
该端口须空闲。bag 测试在临时目录生成旧／新类型的合成数据。

增益模式切换和保护复位与 DAC 指令串行执行：服务返回成功时，之前的在途指令已经完成。
切换为 `off` 后不再进行普通增益调整，原始 ADC 过压保护仍可降低增益。
DAC 指令超时保护在获得串口执行锁后检查最新指令时间，默认仍为关闭。
运动节点已提供手柄超时和停止锁存，周期趋声控制尚未验证真实水中趋声效果。

详细 IO 行为见 [IO参考](docs/io_reference.md)。原声学验证记录作为
[历史记录](docs/acoustic_validation_20260926.md)保留，其中路径和测试数量
描述的是原验证时的版本。
