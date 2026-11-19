# robotic_fish_sonic

机器鱼声学系统的 ROS 1 包，合并原 `robotic_fish_io` 和
`robotic_fish_acoustic`。提供 ADC 采集与校准、DAC 公共增益控制、
原始 ADC 过压保护、无模型通道比较、可选方向模型和历史 bag 工具。
运动及手柄控制仍属于 `robotic_fish`，下位机通信属于 `spinal`。
DS4 的完整轴／物理按键索引见
[手柄输入映射](../robotic_fish/README.md#ds4-输入索引映射)。

rosbag 的话题分层、`exp20261022` 七包完整审计、时钟／校准质量问题及记录整理方案见
[ROS bag 记录梳理](docs/rosbag_audit/README.md)。

包内变量、带单位参数名及对外兼容边界见
[命名与数据出口约定](docs/naming_conventions.md)。当前 ADC 只发布
`/robotic_fish/adc/samples`，已移除重复的单通道 `adc/sample`；消息字段保持兼容。

## 当前包结构

IO 与声学包已在 2026-10-19 的提交 `8e1d24cd` 中合并。目前机器鱼源码仅有
两个 catkin 包：

| 包 | 职责 |
| --- | --- |
| `robotic_fish_sonic` | ADC/DAC 驱动、校准、增益保护、声学计算、消息／服务和状态显示 |
| `robotic_fish` | 手柄、摆尾运动及整机启动；使用 `robotic_fish_sonic` 的数据和算法 |

`io`、`acoustic`、`sonic` 仍可作为配置、模块或话题中的功能名称，不代表三个
独立包。旧目录若只含 `__pycache__/*.pyc`，属于本机缓存残留，可以清理；
工作空间 `build`、`devel` 及 `.catkin_tools` 中也可能保留旧构建记录，
不能据此判断源码中仍有旧包。历史 bag 兼容测试中的旧类型名需要保留。

日常 VIM4 趋声实验使用 `robotic_fish/sonic_vim4.launch`，PC 使用
`robotic_fish/joy.launch`。固定窗口比较／方向模型使用独立的 `quadrant.launch`；
周期趋声控制直接使用 ADC 数据，整机入口已包含所需采集与控制节点。

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
内嵌定义读取；当前校准复查及历史验证范围见 [VALIDATION.md](VALIDATION.md)。

## 启动

日常实验使用 `roslaunch robotic_fish sonic_vim4.launch`，需要同时录包时加 `record:=true`。
运动数值集中在 `robotic_fish/config/motion.yaml`，传感器数值仍在本包 `config/io.yaml`；
配置路径和模块开关见[整机入口与配置](../robotic_fish/README.md#54-日常入口与参数配置)。

独立调试 ADC、DAC、增益控制和固定窗口声学比较：

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
| `status_console.launch` | 独立显示 Share 快照 |
| `robotic_fish/sonic_vim4.launch` | VIM4 整机入口，可关闭运动/桥、可选录包 |

重复组合 `io.launch` 已移除。原 IO＋桥＋显示用法改为
`roslaunch robotic_fish sonic_vim4.launch enable_motion:=false gain_mode:=off`。
仅需本包传感器时继续使用 `sensor_io.launch`。

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
手动 DAC 初始化和调压由增益节点内的独立定时器执行，不依赖 ADC 是否有数据或是否有效；
ADC 的负值、无效值和超时只报告诊断，不阻止手动调压。DAC 指令仍须执行确认并限制在
配置电压范围内，明确检测到原始 ADC 超压时仍优先降压；保护期间不接受调压按键，
有效新 ADC 确认恢复后，须重新按键设定目标，不会自动恢复保护前的高电压。
闭环模式仍要求有效且新鲜的 ADC 数据。
仍由 Circle 进入跟踪、Cross 停止。原 `sonic_teleop.launch` 的组合方式保留；
VIM4 已单独运行传感器和桥时，可仅启动运动：`sonic_teleop.launch enable_joy:=false acoustic_steering:=true`。
每台设备的操作与切换说明见[周期趋声控制](../robotic_fish/README.md#65-周期趋声控制)。
只运行一个手柄发布者、一个运动命令发布者和一个 spinal 桥。
录包可在整机入口设置 `record:=true`，或在 VIM4 另开终端运行 `roslaunch robotic_fish record.launch`，只选一种，
让录制时间与采样、控制时间同源；PC 录包仍需要考虑两端时钟偏差。
录包默认包含连续运动状态、三通道周期结果、锁存控制配置、在线调参事件、诊断和 ROS 日志，
不依赖 Share。详见[录包字段与时序说明](../robotic_fish/README.md#53-控制程序选择)。

## 配置与声学输出

- `config/io.yaml`：ADC、校准表、DAC、增益及保护参数。
- `config/acoustic.yaml`：共享的水听器映射和声学比较参数。默认历史窗口 0.5 秒、
  最大间隔 0.2 秒、低电压数值阈值 0.001 V，以及未来时间戳容差 0.05 秒。
- `config/calibration/`：各电路及供电条件对应的独立通道校准表。
- `config/udev/`：DAC 设备规则；spinal 当前路径为
  `/dev/robotic_fish_spinal`，现场需核实对应规则。

`sonic.launch` 和 `sensor_io.launch` 的 IO 配置由 `config:=...` 选择。
声学配置由 `acoustic_config:=...` 选择，单独运行 `quadrant.launch` 时使用 `config:=...`。
`window_s`、`gap_s`、
`low_signal_v` 的非空启动参数覆盖所选声学 YAML；`model` 通过启动参数选择。
IO 中已有的启动参数覆盖 YAML 的优先级保持不变。

```bash
roslaunch robotic_fish_sonic quadrant.launch window_s:=0.2
rostopic echo /robotic_fish/acoustic/channel_comparison
```

前、左、右水听器的 ADC 通道在 `hydrophone_channels` 中配置。默认值沿用当前接线，
代码和录包中的前向字段名为 `head`。见 [acoustic.yaml](config/acoustic.yaml)。

```yaml
hydrophone_channels:
  FrontPZT: 2
  LeftPZT: 0
  RightPZT: 1
```

`LeftPZT: 0` 表示左侧水听器读取 ADC0。三项必须各使用一个不同的整数通道号（0、1、2）。
修改后重启相关 launch 生效。`sonic_vim4.launch` 和 `sonic_teleop.launch` 也通过
`acoustic_config:=/path/to/acoustic.yaml` 选择文件。独立启动各节点时可指向同一份文件。
采样消息、原始值和校准曲线始终按物理 ADC0/1/2 编号处理，`io.yaml` 中的通道列表采用 `[0, 1, 2]`。
只交换水听器方向、不改变各 ADC 对应电子链路时保留通道校准表；若同时交换模拟前端，应重新验证或标定。
可选闭环 `window_control` 同样使用共享方向映射：窗口均值、峰值及超限组比例
均取所配置的左／右通道原始电压。三通道均参与有效性检查与过压保护。
增益配置的 `window_channel_ids` 记录当前左／右通道号，并同时记录完整映射。
窗口控制仍默认关闭；启用需选择 `closed_loop`、显式设置 `window_control: true`
和经过确认的 `window_high_fraction`。本次只修正通道选择，不改阈值、步长或开启状态。
输出左右差值
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
电压补偿 `delta=lr_voltage_offset_v` 单位为 V、默认零，直接加到左右电压差一次，
以 `d=(L-R+delta)/((L+R)/2)` 与阈值比较，即差值相对两侧平均值的比例。
`threshold=0.15` 表示差异达到平均值的 15%：正向达到阈值左转，负向达到阈值右转，其余直行。
补偿后差值为零时始终直行，包括阈值为零。阈值允许 `[0,2]`（0%～200%）；
旧的除以 `L+R` 的阈值乘 2 可保留相同转向边界，例如旧 `0.15` 对应新 `0.30`。
正的电压补偿偏向左、负值偏向右；原始电压和周期均值保持原值。
舵机映射默认 `left_bias_sign=-1`：判断左侧强时输出负中心偏置，右侧强时输出正中心偏置，
适配当前尾舵需要反向偏转的安装。Share／状态中的 `LEFT`、`RIGHT` 仍表示声学判断方向；
符号参数与电压补偿相互独立。两个运动 launch 和直接运行脚本均采用该默认值，
可在启动时用 `left_bias_sign:=1` 指定初值，也可通过在线服务在下一有效完整周期
切换到 `-1`／`+1`，不重启运动节点或重置摆尾相位；服务中的 `0` 保持当前生效符号。
默认 Option（`buttons[9]`）每次新按下也会翻转符号，长按只触发一次；
按当前实机定义 −1 跟随、+1 远离。基于待生效符号翻转，并保留已有阈值／偏置／补偿。
手动／停止时仅预选，不自动进入跟踪；当前和待生效模式可用 Share 查看。
声源居中时可用多个稳定周期估计 `delta=R-L`。相同数值的补偿效果是旧版左加右减定义的一半；
若需保留旧效果，新值取旧值的两倍；不自动转换配置。
Share 的三通道表只显示原始瞬时电压、原始周期均值、校准文件处理后的周期均值。
`lr_voltage_offset_v` 只在判据计算中使用，不改变上述三行电压；
`AVG [V]` 显示 `(L+R)/2` 的数值代入式；L、R 为文件校准后的上一周期均值。
`Diff before [%]` 展开 `(L-R)/AVG`，
`Diff after/T [%]` 展开 `(L-R+offset)/AVG` 并显示结果／阈值百分比。
分子只增加一次 offset；结果使用控制器已记录的未舍入判据。
另保留电压补偿值、方向与输出偏置及 ADC 质量；Sound mode 显示当前
FOLLOW（−1，跟随）／AVOID（+1，远离），正常跟踪共 18 行。
待生效符号与当前不同时额外显示 Pending mode。
停止原因、历史／无周期标记和待生效补偿仅按需出现。状态话题仍完整保存电压差、
补偿前后相对差及时间信息；消息和服务仍使用比例数值。
诊断中间量 `Lc=L+delta/2`、`Rc=R-delta/2` 的差为 `L-R+delta`，平均值不变。
这些量不写回测得电压；任一中间量为负数或非有限值时，显示 `voltage_compensation_out_of_range`／`HOLDING`，
继续摆尾并保持上一偏置，暂存整组待生效设置，在后续有效完整周期恢复；不截断负值。
输入应为整流／检波后的包络或幅值电压；当前硬件前端已由用户确认符合此条件。
周期均值表示平均响应幅值，不是物理声强，也不能直接用于原始交流声波电压。
跟踪中调整已确认的 DAC 增益不再停止运动：保持摆尾相位和上一偏置，清空旧窗口，
跳过混合增益的采样组，待新增益稳定并积累完整对齐周期后继续判断。
等待期间状态为 `gain_warmup`，Share 显示 `HOLDING` 和历史周期结果。
校准越界回退／未知增益同样保持摆尾与上一偏置并暂停决策，Share 显示具体原因及
`HOLDING`；数据有效后以 `data_warmup` 重建完整周期并自动恢复判断。
首次进入跟踪而数据尚不可用时，以零偏置等待。时间检查、饱和和 Cross 停止规则仍生效。
Joy 断联行为由运动配置 `joy_timeout_action` 决定：项目默认 `continue`，手动保持最后位置，
普通自动和趋声继续运行；趋声仍依赖有效的本地 ADC 数据。设为 `stop` 则在超过
`joy_timeout_s` 后回中并锁存。可通过 `sonic_vim4.launch joy_timeout_action:=stop` 覆盖，
详见 [运动控制说明](../robotic_fish/README.md)。
VIM4 运动节点按 Share 发布纯文本快照，整机入口默认启动 `status_console.py`，
在 VIM4 终端用对齐表格显示三通道瞬时电压、上一完整周期均值、差值及方向。
`status_color:=auto` 自动检测终端颜色，`always` 强制彩色，`never` 关闭；
`enable_status_console:=false` 可关闭显示。独立显示可运行
`roslaunch robotic_fish_sonic status_console.launch color:=auto`。
具体见 [Share 查看系统状态](../robotic_fish/README.md#66-share-查看系统状态)。
周期趋声使用 `acoustic.yaml` 中的方向映射，要求有效校准和已确认增益。
Share 三行电压统一按前／左／右显示，表头标出实际 ADC 通道号。

新增接口：

- `srv/SetAcousticSteering`：接收 `threshold`、`bias`、`lr_voltage_offset_v` 和 `left_bias_sign`，
  四项整组验证、暂存，在下一有效周期一起生效。符号为 `-1`／`+1`，省略或 `0`
  使用当前生效符号；响应给出 `active_left_bias_sign` 和 `pending_left_bias_sign`，
  后者无待生效请求时为 0。补偿为有限有符号伏特值，周期内须满足 `-2L<=delta<=2R`。
- `msg/AcousticSteeringState`：记录周期窗口、方向差值、偏置、参数及模式状态。
- `msg/SonicMotionState`：记录每个控制周期的运动输出、摆幅／相位、输入年龄及 ADC 质量。
- `msg/AcousticSteeringEvent`：记录调参请求接受／拒绝／替换、实际生效周期及关机取消，
  包含解析后的 `left_bias_sign`；拒绝事件保留无效请求符号。
- `/robotic_fish/sonic/config`：复用 `RuntimeConfig`，锁存完整控制配置及待生效请求。
- `/robotic_fish/acoustic/steering_state`：启用功能后的运行状态，可随 bag 记录。

目前摆频由 VIM4 运动启动参数指定，窗口自动取倒数；更换摆频后重启并重新积累完整周期。
ADC 年龄、未来时间戳检查均在 VIM4 本机进行，未来容差继续默认 0.05 秒；
`acoustic_future_tolerance_s` 在启动时显式设置，启动日志显示主机名和实际时间限值。
补偿可通过运动 launch 的 `acoustic_lr_voltage_offset_v` 设置初值，运行时调用
`/sonic_teleop_node/set_acoustic_steering` 填写前三项，并按需指定方向符号；具体命令见运动包说明。
新增符号字段改变服务与事件的 ROS MD5；两端须重新编译 `robotic_fish_sonic robotic_fish`、
source 并重启节点。新版本仍接受省略符号的三字段 YAML 调用，旧二进制客户端需重新生成。
调用后直接返回当前模式、数据质量、方向、控制偏置、正在使用及待生效参数；
真正生效时 VIM4 运动终端打印请求编号、周期和参数。服务成功表示接受请求，仍等待有效周期。
旧无量纲 `lr_offset` 及其启动参数已移除，新补偿名称同时用于配置、事件和服务响应。
锁存运动配置的 `schema_version` 为 5，`settings.channel_layout` 按 ADC0/1/2 顺序记录方向，
`settings.hydrophone_channels` 记录每个方向对应的通道号，无跟踪时也记录映射。
保留版本 4 引入的 `difference_reference=channel_mean`、
`voltage_offset_reference=left_minus_right` 和包含等号的判定规则；旧版本 3 的同名补偿字段是单侧调整量。
状态、参数事件和服务反馈也标记差值基准，避免与旧 bag 的总和归一化含义混淆。消息及服务定义已改变，
两端需重新编译并 source 两包后重启相关节点，旧 bag 仍可用嵌入式定义离线读取。
不会在运动底层加入通用偏置限速。验证边界见
[周期趋声验证记录](docs/cycle_steering_validation.md)。

## 离线工具

```bash
rosrun robotic_fish_sonic replay_quadrants.py recording.bag --output /tmp/fish-replay --channel-layout left_right_head
# 旧接线（ADC0=前、ADC1=左、ADC2=右）的历史 bag：
rosrun robotic_fish_sonic replay_quadrants.py old.bag --output /tmp/fish-replay-old --channel-layout head_left_right
rosrun robotic_fish_sonic calibrate_from_bag.py sweep.bag --output /tmp/fish-calibration
```

输出目录必须不存在。回放必须显式指定 `--channel-layout`，名称依次表示 ADC0/1/2 的方向；
不能根据 bag 日期、消息 schema 或模型版本猜测实际接线。每行结果及摘要均保存选用的映射。
运动节点通过 `/robotic_fish/sonic/config` 记录映射；独立声学节点也通过既有
`/robotic_fish/gain_control/config` 发布 `device=acoustic_quadrants` 的锁存配置，已纳入录包。
模型特征顺序保持前向比例、左侧比例、左右差、前侧差，不随 ADC 重排；真实模型仍需实测验证。
回放支持 `--window-s`、`--model` 和
`--calibration`；更换校准只修改内存中的解码值。校准工具只有显式提供
`--common-input` 才生成候选表，该选项要求实验确实采用共同电输入，
不能用于将自由游动数据归一化。工具不改写 bag 或部署校准表。
生成前检查全部原始 ADC 记录，包括 DAC 反馈无效的组；存在非有限值、
饱和码或达到会话 ADC 阈值时拒绝生成。时间留出覆盖率须至少 99%，
且校准后绝对差异 P95 不得变差。通过这些检查仍需要独立复测后才能部署。

新电路的公共输入校准流程见[校准实验说明](docs/calibration_procedure.md)。
2026-11-14 的 5 mVpp 实验使用 `--dac-max-v 1.30` 筛选 DAC 平台。该参数只用于离线筛选。
离线 ADC 阈值优先采用 bag 中记录的
`safety_limit_v`；无记录时采用显式 `--adc-safety-limit-v` 或本包 IO 配置，
与线上在达到阈值时即触发保护的判定一致。当前默认触发值为 4.00 V，
恢复值 3.30 V 仍单独保留用于滞回。

当前默认使用 2026-11-14 **电池供电**校准表。0–1.30 V 共扫描 131 档，
其中 121 个正电压平台用于拟合，加上训练数据实测上下边界，每通道共 123 个节点。
边界只取自训练样本，不使用独立复测扩大范围，也不将边界内读数统一压到平台中位数。
独立复测的 15 个支持平台共 3,507 组窗口样本，3,502 组成功校准、5 组回退，
覆盖率 99.857%；配对样本的绝对差异 P95 从旧表 4.96 mV 降至 3.48 mV。
低信号 0.10/0.15 V 档相对差异 P95 仍约 11%/14%，非正读数仍回退；不能视为全量程精密校准。
完整边界、夹紧统计、原始数据与重建命令见[电池校准报告](docs/calibration_20261114_battery.md)。
11 月 12 日的[稳压电源校准表](docs/calibration_20261112_measured_support.md)保持原样；切回该供电条件应选择并验证对应配置。
初次 0–1.24 V 扫描记录保留在[原始校准报告](docs/calibration_20261112_new_circuit.md)。

2026-11-19 对更换后的 LNA 以 40 kHz、5 mVpp 公共电输入进行校准。
前、左、右水听器在台架校准时由信号源代替，结果描述三条电子链路的相对响应。

| 校准表 | DAC 扫描范围 | ADC0／ADC1／ADC2 上界 | 独立复测 |
| --- | --- | --- | --- |
| [初次扫描](config/calibration/adc_independent_transfer_20261119_lna_40khz_5mvpp.json) | 0–1.03 V | 2.192750／2.180125／2.176500 V | 1,924 组覆盖率 100%，通道差 P95 从 23.13 mV 降到 2.38 mV |
| [扩展扫描](config/calibration/adc_independent_transfer_20261119_lna_40khz_5mvpp_extended.json) | 0–1.23 V | 3.821375／3.794875／3.795750 V | 2,971 组覆盖率 100%，通道差 P95 从 28.50 mV 降到 3.72 mV |

扩展表合并 63 个训练平台，4 个重叠档的最大中位电压漂移约为 0.272%。
文件记录了各通道实测边界、原始 bag 的路径和校验和，以及独立复测结果。
在新 LNA 上使用扩展表时，将 `io.yaml` 的 `adc.calibration.file` 设为
`calibration/adc_independent_transfer_20261119_lna_40khz_5mvpp_extended.json` 后重启 ADC 节点。

本机 DAC 当前使用已确认的 CH340 稳定路径
`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`；原物理端口规则下的
`/dev/robotic_fish_dac` 别名在当前接线中不存在。

单个扫描 bag 可用 `calibrate_from_bag.py --include-measured-endpoints` 生成实测支持边界。
原始曲线仍为分段线性插值；边界节点的参考值由最近两个中位数节点线性估计，
不是新增的精密参考电压测量。未使用该选项时保留原端点处理。
11 月 12 日多 bag 合并可通过 `rebuild_new_circuit_calibration.py` 重建；电池版使用随实验数据归档的 `rebuild.py`，命令见各自报告。

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
