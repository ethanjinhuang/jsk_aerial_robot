# robotic_fish_sonic 合并验证

## 2026-11-12 包内命名整理与 ADC 重复发布移除

本轮仅修改 `robots/robotic_fish_sonic/`，未修改 `robotic_fish`、`spinal`
或其他模块。规则与旧名／新名对照见 [命名与数据出口](docs/naming_conventions.md)。

- ADC、DAC、增益节点及采集／校准／声学算法的内部物理量名称明确含义、单位和时钟。
  增益请求值与 DAC 确认值分开命名，固定窗口和周期判据的分母含义分开命名。
- 新的私有参数别名优先于旧名，仍支持当前 launch/YAML 和历史 `agc` 参数；
  显式 `false`、零值和空字符串不会被错误地忽略。
- 移除单通道 `adc/sample` 发布器和对应配置，仅发布完整 `adc/samples`。
  旧 `publish_array=false` 给出警告并仍发布批数据，防止唯一数据出口被意外关闭。
- 全部 10 个消息和 2 个服务定义、校准 JSON 文件逐字节未变；
  周期控制器公开参数、数据类字段、适配字典与配置 JSON 的旧键保持兼容。
  配置 `topics` 中删除已不存在的单通道 `sample` 条目。
- 没有实施跨包的运动消息重构、周期决策新话题、配置话题合并或调参事件开关。

| 验证 | 本轮结果 |
| --- | --- |
| 修改前声学包基线 | 167 项通过 |
| 修改后声学包单元测试 | 173 项通过，含 6 项新接口／发布回归 |
| 未修改的运动包测试 | 65 项通过 |
| 独立 CMake 构建、消息生成、安装 | 通过，输出全部位于 `/tmp` |
| CMake `run_tests` XML | 12 份 XML，173 项，0 errors、0 failures、0 skipped |
| 安装环境单元测试 | 173 项通过；核心算法与参数模块实际从 install 路径导入 |
| 新旧 bag 兼容集成测试 | 4 项通过，含旧包 12／14 字段、新包 14 字段及候选拒绝路径 |
| ADC 实际 ROS + `record -a` 回读 | 源码与安装可执行文件均通过：无单通道话题、完整批数据、校准、DAC 标记、一秒均值、晚订阅配置 |
| 声学隔离 ROS 通信 | 无模型比较、质量回退、未来帧拒绝及恢复通过 |
| 未修改运动节点的隔离 ROS 通信 | 源码／安装声学环境均通过，含 1.5／2 Hz、两种舵机符号、保持／停止／恢复、Share 和真实录包回读 |
| 原始实验 bag 的算法差分 | 7 包共 27,233 组，适配结果、固定窗口输出与周期控制器结果逐组相同 |
| 静态检查与范围 | Python 3.8 语法、文档链接、差异格式与修改目录检查通过 |

6 项新增回归覆盖：新名优先及显式零／假值、ADC 新旧参数与真实配置快照、
DAC 新旧参数快照、gain 旧 launch 参数和新别名、只发布完整 ADC 组、
中途转换失败不发布残缺组。构造测试使用实际生成消息与真实节点代码，
mock ROS 通信和硬件打开操作；通信测试另使用真实 localhost ROS 传输。

算法差分以修改前源码副本为基线，对同一输入与同一时间映射分别运行两版。
周期控制器使用固定 1.5 Hz 和相同的离线接收时间映射；该检查证明重命名前后
计算行为一致，不声称重建了现场的 Joy 模式切换或所有在线周期决策。
ADC 通信测试仅把 I²C 打开操作替换为模拟总线；不会访问 `/dev/i2c-*`。
未启动真实 DAC、spinal 桥或执行器，也未部署到现场运行节点。

本轮工作目录为 `/tmp/fish-sonic-naming-5qis6pbs`，包括 `build`、`devel`、
`install`、修改前 `baseline`、临时测试依赖及差分复查脚本 `compare_bags.py`。
日志为同目录下的 `cmake.log`、`build.log`、`cmake-tests.log`、`install.log`、
`motion-tests.log`、`bag-compatibility.log`、`bag-differential.log`、
`adc-ros.log`、`acoustic-ros.log`、`cycle-ros.log`、
`installed-adc-ros.log` 和 `installed-cycle-ros.log`。
单元测试日志另位于 `/tmp/fish-sonic-naming-tests.log`。
测试依赖仅安装于该临时目录；CMake 使用的临时 nose 启动器补齐
Python 3.10 的 `collections.Callable` 兼容，不修改系统 Python 或项目算法。

可复查的包内入口见 [命名文档中的验证命令](docs/naming_conventions.md#6-验证入口)。
主工作空间的 build/devel/install 未更新；现场使用前需重新构建声学包并重启相关声学节点。

## 2026-11-12 速度限幅与 DAC 请求校验修复

按用户确认的范围修复下方检查中的问题 4、5。共享串口桥暂缓，旧控制器的
Joy 失联保护留待专项处理，通用舵机桥不属于当前 fish_sonic 修复范围。

- `robotic_fish/scripts/teleop.py` 与 `spring_teleop.py` 对缩放后的速度输出
  做正负限幅。非有限输入或缩放溢出时发送零速，后续有效命令可正常执行。
- `dac_node.py` 在进入硬件操作前完成电压及通道校验。非法请求返回失败，
  保留已确认电压、串口连接、原锁存状态和命令看门狗时间；请求失败仍计入错误统计。
  实际通信失败、写超时或回显错误仍发布无效状态并关闭连接。
- 新增 7 项回归测试，覆盖两个旧速度入口的正负边界、自定义倍率／限值、
  NaN/Inf／缩放溢出及恢复，以及 DAC 参数拒绝后的状态保留、后续合法请求、
  断连／超时／回显失败。新增测试在修复前已复现原问题。
- 更新原有 DAC 记录测试：成功和硬件失败发布执行结果；参数拒绝仅通过服务
  返回原因，不覆盖硬件状态话题。

两个机器鱼包的 CMake `run_tests` 结果为声学包 167 项、运动包 65 项，
合计 232 项，0 errors、0 failures、0 skipped。独立构建／安装空间复用
`/tmp/fish-project-audit-build-NjZsMW`；日志位于
`/tmp/fish-fixes-cmake-tests.log` 和 `/tmp/fish-fixes-install.log`。
本次未启动硬件节点或更改现场运行配置。

## 2026-11-12 仓库检查：机器鱼链路与共享组件

本轮对仓库做静态扫描，深入检查机器鱼的采集、校准、增益保护、运动与录包链路，
并检查它们使用的共享串口桥及通用舵机桥。以下记录检查时复现的问题；
问题 4、5 已随后修复，验证与当前处理范围见上节。
P1 表示应优先处理的启动或控制异常，P2 表示需要修正的功能问题。

### 已确认问题

1. **P1：共享串口桥禁止调用方覆盖串口，阻止其他机器人启动。**
   `aerial_robot_nerve/spinal/launch/bridge.launch:5–6` 用 `value` 声明
   `serial_port`，固定为 `/dev/robotic_fish_spinal`。其他机器人的传感器入口
   通过 include 传入 `/dev/flight_controller` 时，ROS 报
   `cannot override arg 'serial_port', which has already been set`。
   使用真实 XmlLoader、`real_machine:=true` 逐个验证，dragon、gimbalrotor、
   hydrus、hydrus_xi、mini_quadrotor 的 14 个传感器配置全部在这里失败。
   建议共享入口使用可覆盖的通用默认值，机器鱼专用串口由机器鱼入口传入，
   并补充跨机器人启动参数验证。

2. **P1：旧控制入口没有有效 Joy 超时停止，短消息也不触发停止。**
   `robotic_fish/scripts/advance_teleop.py:212–225` 的控制循环只处理自动模式，
   没有 Joy 接收时间检查；巡航后不再收到 Joy，调用 500 次控制循环也没有
   新的停止指令。`teleop.py`、`spring_teleop.py`、`advance_teleop.py`
   收到空 Joy 均抛出 `IndexError`，没有发布停止命令。
   这确认的是上位机节点缺少停止动作；真实电机最终行为仍取决于下位机及驱动配置。
   当前 `sonic_teleop.py` 的超时／停止测试通过，但这些旧入口没有共享其保护。
   建议仍需使用的旧控制器加入基于单调时钟的有效输入超时、输入校验和停止锁存；
   不再使用的入口应明确归档。旧版速度／多执行器控制不能直接替换为单舵机位置控制。

3. **P1：通用舵机桥的无名称力矩消息校验字段错误。**
   `aerial_robot_model/src/servo_bridge/servo_bridge.cpp:455–466` 在力矩回调中
   检查、遍历 `position`，却读取 `effort.at(i)`。一个关节时，合法的
   `name=[]、position=[]、effort=[1]` 被拒绝；
   `name=[]、position=[0]、effort=[]` 抛出 `std::out_of_range`。
   节点主函数没有捕获此异常。已抽取原始回调函数体，用 C++ 桩编译运行复现；
   没有连接真实 ROS 舵机桥或硬件。
   建议此分支按 `effort` 校验并遍历，先验证整条消息再更新任何关节。

4. **P2：旧版速度回调对缩放前的值做限幅，实际输出可超过配置上限。**
   `robotic_fish/scripts/teleop.py:264–269` 和
   `robotic_fish/scripts/spring_teleop.py:164–169` 计算
   `val = vel * vel_rate`，随后却比较 `vel` 与 `max_val`。
   直接调用原始回调已确认：`vel_rate=250、max_val=250、linear.x=2`
   输出命令为 500；负向输入同样存在问题。
   建议对缩放后的 `val` 做对称限幅，并拒绝非有限输入。

5. **P2：DAC 参数错误会清除已确认状态并关闭串口。**
   `robotic_fish_sonic/scripts/dac_node.py:109–122` 将参数验证失败和硬件写入
   失败放进同一个异常处理。已确认当前电压为 1.23 V 时，向非配置通道发请求，
   在没有执行硬件写入的情况下也会把 `applied_voltage` 清空、发布反馈无效并关闭串口。
   下游因而失去已确认的增益信息；周期趋声会进入未知增益等待。
   建议在硬件异常处理之前完成参数校验，仅实际通信／执行失败才使 DAC 状态失效。

### 本轮验证结果

| 检查 | 结果 |
| --- | --- |
| 仓库包清单 | 17 个 catkin 包，清单可解析；机器鱼仅有两个有效源码包 |
| 跟踪文件语法 | Python 81、XML/launch/URDF/xacro 150、YAML 152、JSON 6，共 389 个，解析无错误 |
| 机器鱼现有单元测试 | 声学包 163 项、运动包 62 项通过 |
| 独立 CMake 构建、安装、run_tests | 两个机器鱼包通过；测试 XML 合计 225 项，0 errors、0 failures、0 skipped |
| bag 兼容／候选拒绝集成测试 | 4 项通过 |
| 隔离声学 ROS 通信 | 校准回退、无模型比较、未来帧拒绝及恢复通过 |
| 安装后运动 ROS 通信 | 暂停 ROS 时钟、摆尾、停止锁存、输入异常和 Joy 超时通过 |
| 安装后周期趋声 ROS 通信 | 1.5/2 Hz、两种转向符号、增益／质量等待、在线调参、Share 及停止恢复通过 |
| 实际 record.launch 录包回读 | 连续运动／相位、周期结果、锁存配置、调参事件、Share 和 ROS 日志通过 |
| 原始实验 bag 重建默认校准 | 重建 JSON 内容与默认表相同；独立复测 3,562 组、18 个平台，0 回退 |
| 跨机器人传感器启动配置 | 14 个确认失败，原因见问题 1 |

默认校准复测的绝对差异 P95 为 2.346 mV，相对差异 P95 为 0.488%；
这些结果沿用原实验条件，不能解释为新的硬件测量或绝对电压精度认证。

另扫描了 221 处本地 ROS 资源引用：MuJoCo 模型目录在本机未提供，旧 hydrus
T265 配置引用的 `T265ModuleList.yaml` 也不存在。这些可选／历史场景需要补齐
资源后再验证；动态拼接的资源路径未据此判定完整可用。

构建与安装目录为 `/tmp/fish-project-audit-build-NjZsMW`；日志前缀为
`/tmp/fish-project-audit-`，包括 `static.log`、`cmake-tests.log`、
`bag-tests.log`、`acoustic-ros.log`、`motion-ros.log`、`cycle-ros.log`、
`cross-launch.log`、`python-repro.log` 和 `torque-repro.log`。
临时重建表位于独立构建目录的 `calibration-rebuilt/`，没有替换默认配置。

本轮没有执行飞行、舵机／电机动作、ADC/DAC 硬件采集或固件刷写；
未重新编译整个飞行 C++ 栈与 STM32 固件。机器鱼安装验证仍使用现有工作空间
提供的 spinal 等依赖，不能据此声称所有包均可在全新系统独立部署。
工作空间旧包的 build/devel/.catkin_tools 残留未修改。

## 2026-11-12 新电路校准复查

本节记录当前校准改动的代码审查与离线验证。校准采集和独立复测数据见
[初次扫描报告](docs/calibration_20261112_new_circuit.md)及
[1.30 V 与实测边界报告](docs/calibration_20261112_measured_support.md)。
下文 10 月的测试数量和配置属于历史版本。

当前默认表为 `adc_independent_transfer_20261112_060020_new_circuit_psu_130_observed_support`。
每通道有 131 个平台中位数节点和两个实测支持边界节点；边界参考值是局部线性
估计，明确标记在 JSON 中。历史 125 节点表保留，未通过覆盖率检查的中位数端点
候选没有部署。正式运行仍采用 DAC 0–5 V、ADC 4.00 V 触发和 3.30 V 恢复阈值。
1.30 V 是本次 5 mVpp 实验的 DAC 上限。

审查后补齐了以下路径：

- 单 bag 与多 bag 工具共用原始数据读取与审查。先检查全部原始 ADC，再过滤
  DAC 反馈无效组；非有限电压、饱和码及会话超限不能被反馈过滤掩盖。
- 单 bag 生成候选要求时间留出覆盖率至少 99%，且绝对差异 P95 不变差；
  失败时不创建候选目录。仅审计模式仍能输出异常数据摘要。
- 多 bag 重建检查完整的训练、重叠及独立复测平台计划。独立验证仍要求每平台
  覆盖率至少 95%、整体至少 99%，且每平台绝对差异 P95 改善。
- 安装后的离线工具通过 ROS 包资源路径查找 `config/io.yaml`，支持无配置快照
  的旧 bag；显式与记录阈值冲突仍拒绝生成。

| 检查 | 结果 |
| --- | --- |
| 全包单元测试 | 163 项通过；源码环境与独立安装环境均通过 |
| bag 集成测试 | 4 项通过：旧包 12/14 字段、新包 14 字段、隐藏超限和留出失败拒绝生成 |
| 原始实验 bag 重建 | 与当前默认校准 JSON 完全一致 |
| 独立复测 | 18 个平台、3,562 组稳定样本，0 回退；绝对差异 P95 为 2.346 mV，相对差异 P95 为 0.488% |
| 独立 CMake 构建、安装 | 通过；产物位于 `/tmp/fish-calibration-final-review` |
| 安装后的 CLI 与资源定位 | 两个工具入口可用；旧 bag 正确回退到安装包配置，新校准表可加载 |
| 安装后的 launch 参数展开 | 选择新表及 CH340 by-id；DAC 上限 5.0 V，ADC 保护 4.00 V |
| 隔离 ROS 通信 | 合成消息更新、校准回退、无模型比较、未来帧拒绝及恢复通过 |
| Git 差异格式 | `git diff --check` 通过 |

可重复执行的入口：

```bash
python3 -m unittest discover -s robots/robotic_fish_sonic/test -v
python3 robots/robotic_fish_sonic/test/bag_compatibility.py
python3 robots/robotic_fish_sonic/test/ros_smoke.py
```

需先 source 对应的构建或安装环境。ROS 通信脚本使用隔离 master 11427，
不启动 ADC、DAC、spinal 或运动硬件节点；本次复查没有再次操作硬件。
完整实验重建命令见实测边界报告。

本机验证日志位于 `/tmp/fish_final_review_*.log`，参数展开为
`/tmp/fish_final_review_launch.yaml`，重建结果位于 `/tmp/fish_final_review_rebuilt`。
这些临时路径不是长期数据归档；原始 bag 的位置及 SHA-256 已嵌入校准 JSON。
当前验证覆盖同一 5 mVpp、稳压供电条件；频率、ADC VDD 仍未提供，未验证弱输入
配合更高增益、温度变化或模拟链路削波，不能把相对通道一致性当成绝对电压精度。

## 历史合并验证

2026-10-20 新增的完整摆尾周期声学控制、在线调参和隔离通信验证见
[周期趋声验证记录](docs/cycle_steering_validation.md)。下文保留合并时的验证范围。

## 2026-10-21 审查修复

新增回归先在修复前复现了模式切换提前返回、DAC 看门狗覆盖新指令，以及
固定窗口接受未来时间戳的问题，再验证修复后的行为：

- 增益模式切换与保护复位使用与 ADC／DAC 执行相同的锁顺序。
  服务返回成功前等待在途指令完成；`off` 仍保留 ADC 过压降增益保护。
- DAC 看门狗在锁内读取最新指令时间，真实超时只执行一次；写入失败仍可重试。
- 固定窗口拒绝未来采样组，保留历史结果及原时间戳，并要求重新预热。
  ROS 时间回退后的心跳也会使窗口失效；默认允许 0.05 秒的小幅时钟偏差。

验证完成：声学包 138 项、运动包 42 项单元测试通过，新增的 11 项覆盖上述异常
与恢复路径。声学包独立构建、CMake 测试及安装通过，138 项结果无失败或跳过。
三套隔离 ROS 通信测试通过，包含真实消息的未来帧拒绝与重新预热、手柄控制、
1.5／2 Hz 周期跟踪和三字段在线调参。3 种新旧 bag 定义的兼容测试通过。
另将用户记录的 `_2026-10-21-15-22-24.bag` 中全部 2970 组 ADC 数据与修复前
固定窗口输出逐组对照，结果一致。安装后的 launch 正确加载 0.05 秒容差。

构建、CMake 测试、安装及参数展开记录位于
`/tmp/robotic-fish-sonic-merge-check/audit-fixes-*.log` 和
`audit-fixes-quadrant-params.yaml`；周期通信日志为 `/tmp/fish-cycle-ros-ybbdsaqc`，
运动通信日志为 `/tmp/fish-motion-ros-5hhis4cw`。
这些修复不改变周期控制判据或消息／服务字段，未运行硬件节点或修改现场配置。

## 范围

合并 `robotic_fish_io` 与 `robotic_fish_acoustic`，统一构建、Python 包、
消息类型和资源定位；保留话题／服务地址、字段、校准表和运行算法。
新增 `sonic.launch` 组合入口以及声学参数 YAML。

旧包基线：IO 75 项、声学 16 项单元测试通过；三个机器鱼相关包独立构建、
安装通过，隔离 ROS 通信测试通过。原记录见
[声学历史验证](docs/acoustic_validation_20260926.md)。

## 本次结果

验证日期：2026-10-19。环境为本机 ROS 1（前缀 `/opt/ros/one`）、
Python 3.10。构建、安装、测试依赖和合成数据放在 `/tmp`，未修改主工作空间
的 build/devel/install。运行时检查使用新安装目录，不加载旧包的 Python 路径。

| 检查 | 结果 |
| --- | --- |
| 迁移后的原有单元测试 | 91 项通过 |
| CMake `run_tests` 的 XML 核验 | 91 项，0 errors，0 failures |
| 新包独立配置、构建和安装 | 通过 |
| 更新依赖后的 `robotic_fish` 独立构建、安装 | 通过 |
| 安装后的导入、消息生成及校准资源定位 | 通过；无旧包导入依赖 |
| 7 种消息、1 种服务的字段与序列化 MD5 | 与迁移前一致；类型所属包更新 |
| 原有隔离 ROS 通信测试 | 通过，使用新安装的消息和 Python 库 |
| 新旧 bag 兼容测试 | 3 项通过；安装后的离线入口重复验证通过 |
| 原算法及入口核对 | 核心文件仅命名空间变化；回放移除跨包源码路径 |
| 三份校准 JSON | 与迁移前逐字节一致 |
| 旧包源码和依赖引用 | 旧源码包移除，运行依赖全部迁移 |

bag 测试覆盖旧包 12 字段、旧包 14 字段及新包 14 字段的内嵌定义，
每种生成 320 组三通道数据。检查无模型比较、共同电输入候选校准、
重新校准回放及原 bag 哈希不变。它们是合成兼容性测试，不是真实实验数据。

额外使用合并前保存的 12／14 字段合成 bag 作对照：共 640 组声学 JSONL
结果逐组完全一致；校准审计的 summary、配置快照和平台 CSV 逐字节一致。
回放摘要中的计算耗时不作为等价比较项。

launch 检查只解析配置，不启动节点。10 组场景全部通过：默认入口、显式参数
覆盖、关闭声学、仅声学、关闭增益控制、全部关闭、带串口桥的 IO 入口、
运动包引用、自定义声学 YAML、命令行覆盖 YAML。新节点均解析到安装后的
可执行文件。默认入口不包含手柄、spinal 桥、运动或录包节点。

CMake 逐文件运行测试时，发现两个旧测试对相邻测试文件的导入依赖工作目录；
已在这两个测试中显式加入测试目录，随后全部通过。
本机临时使用的 PyPI nose 1.3.7 在 Python 3.10 下需要
`collections.Callable` 兼容别名，兼容层只存在 `/tmp` 的测试环境；
直接 `unittest` 的 91 项测试不依赖该兼容层。

CI 脚本已加入新包单元测试、ROS 通信和 bag 兼容测试；未声称远端 CI 已执行。
本机详细日志位于 `/tmp/robotic-fish-sonic-merge-check`。

## 已知限制

- 未启动 ADC、DAC、spinal 串口桥或舵机硬件节点。
- 合成 bag 兼容测试不代替真实实验 bag 和目标机验证。
- 合并时保留的未来时间戳检查缺口已于 2026-10-21 修复，见上文。
- 手柄超时、左右转向闭环、声学配置运行快照未包含在本次迁移中。
