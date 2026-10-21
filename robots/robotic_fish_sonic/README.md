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

`sonic.launch` 的 `enable_adc`、`enable_dac`、`enable_gain_control`、
`enable_acoustic` 可分别关闭模块。仅处理外部 ADC 消息时：

```bash
roslaunch robotic_fish_sonic sonic.launch enable_adc:=false enable_dac:=false
```

关闭增益控制节点会同时移除该节点提供的过压降增益保护。

手动增益实验可以在不同终端依次启动：

```bash
roslaunch robotic_fish_sonic sonic.launch gain_mode:=manual
roslaunch spinal bridge.launch dev:=vim4
roslaunch robotic_fish sonic_teleop.launch
roslaunch robotic_fish record.launch
```

`sonic_teleop.launch` 启动手柄和运动程序。只测增益时改用单独的
`joy_node`。只运行一个手柄发布者和一个 spinal 桥；已启动 `io.launch`
时不要再启动同一硬件的 IO 或桥节点。

## 配置与声学输出

- `config/io.yaml`：ADC、校准表、DAC、增益及保护参数。
- `config/acoustic.yaml`：声学节点私有参数，默认历史窗口 0.5 秒、
  最大间隔 0.2 秒、低电压数值阈值 0.001 V。
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

## 按摆尾周期进行左右趋声控制

`cycle_steering.py` 提供独立于 ROS 的完整周期统计、输入质量检查和
阈值／固定偏置判定；`robotic_fish/sonic_teleop.py` 提供摆尾相位、手柄
模式和唯一舵机目标出口。该功能默认关闭，启用后由圆圈选择跟踪模式；
三角仍选择普通自动摆尾，方块选择手动，Cross 统一停止。开启方式和现场调参见
[运动包说明](../robotic_fish/README.md#65-周期趋声控制)。

每次同方向过零时读取上一完整 `1/f` 周期的校准 ADC 数据，比较左右
电压均值，选取 `-C/0/+C`，保持一周期。该路径直接使用 ADC 数据，
不经过 `quadrant_node` 的固定窗口，不需要方向训练模型。
输入应为整流／检波后的包络或幅值电压；当前硬件前端已由用户确认符合此条件。
周期均值表示平均响应幅值，不是物理声强，也不能直接用于原始交流声波电压。
跟踪中调整已确认的 DAC 增益不再停止运动：保持摆尾相位和上一偏置，清空旧窗口，
跳过混合增益的采样组，待新增益稳定并积累完整对齐周期后继续判断。
等待期间状态为 `gain_warmup`，Share 显示 `HOLDING` 和历史周期结果。
校准越界回退／未知增益同样保持摆尾与上一偏置并暂停决策，Share 显示具体原因及
`HOLDING`；数据有效后以 `data_warmup` 重建完整周期并自动恢复判断。
首次进入跟踪而数据尚不可用时，以零偏置等待。时间检查、饱和、Cross 和 Joy 超时等停止规则仍生效。
运动节点按 Share 发布纯文本快照，`io.launch` 默认启动 `status_console.py`，
在 VIM4 终端用对齐表格显示三通道瞬时电压、上一完整周期均值、差值及方向。
`status_color:=auto` 自动检测终端颜色，`always` 强制彩色，`never` 关闭；
`enable_status_console:=false` 可关闭显示。独立显示可运行
`roslaunch robotic_fish_sonic status_console.launch color:=auto`。
具体见 [Share 查看系统状态](../robotic_fish/README.md#66-share-查看系统状态)。
三通道映射仍要求 ADC0=头、ADC1=左、ADC2=右，并要求有效校准和已确认增益。

新增接口：

- `srv/SetAcousticSteering`：将阈值和转向偏置作为一组验证、暂存，在下一有效周期生效。
- `msg/AcousticSteeringState`：记录周期窗口、方向差值、偏置、参数及模式状态。
- `/robotic_fish/acoustic/steering_state`：启用功能后的运行状态，可随 bag 记录。

目前摆频由启动参数指定，窗口自动取倒数；更换摆频后重启并重新积累完整周期。
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

原有固定窗口算法保持不变，其未来时间戳可靠性检查缺口仍待处理。
新的周期趋声路径单独检查采样年龄和未来时间戳，不依赖该固定窗口的可靠性标记。
运动节点已提供手柄超时和停止锁存，周期趋声控制尚未验证真实水中趋声效果。

详细 IO 行为见 [IO参考](docs/io_reference.md)。原声学验证记录作为
[历史记录](docs/acoustic_validation_20260926.md)保留，其中路径和测试数量
描述的是原验证时的版本。
