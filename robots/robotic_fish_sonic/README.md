# robotic_fish_sonic

机器鱼声学系统的 ROS 1 包，合并原 `robotic_fish_io` 和
`robotic_fish_acoustic`。提供 ADC 采集与校准、DAC 公共增益控制、
原始 ADC 过压保护、无模型通道比较、可选方向模型和历史 bag 工具。
运动及手柄控制仍属于 `robotic_fish`，下位机通信属于 `spinal`。

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

本次合并保持已有算法。此前发现的未来时间戳可靠性检查缺口、
运动节点手柄超时处理、声学参数运行快照和左右响应控制均留待后续改动。
尚未进行真实硬件验证。

详细 IO 行为见 [IO参考](docs/io_reference.md)。原声学验证记录作为
[历史记录](docs/acoustic_validation_20260926.md)保留，其中路径和测试数量
描述的是原验证时的版本。
