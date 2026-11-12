# fish_sonic 命名与数据出口

本次修改仅位于 `robots/robotic_fish_sonic/`。内部名称采用明确的对象、含义和单位；
跨包使用的 ROS 消息、服务、话题及 Python 接口保留兼容名称。
没有实施此前讨论的运动消息精简、周期决策新话题、配置话题合并或调参事件开关，
因为这些发布逻辑位于不允许修改的 `robotic_fish` 包。

## 1. 运行行为

- ADC 唯一的数据话题为 `/robotic_fish/adc/samples`，每次发布一组三通道数据。
- 删除 `/robotic_fish/adc/sample` 发布器及 `adc/sample_topic` 配置，不保留调试开关。
- `AdcSample.msg` 继续作为批数据的元素类型，14 个字段及其含义不变。
- 每组转换完成后更新一秒均值、应用校准，再发布完整数据组；中途 I²C／转换失败不发布残缺组。
- `adc/publish_array` 不再是输出开关；旧值 `false` 会给出警告并仍发布批数据。
  若要停止采集，请使用已有的 `enable_adc:=false`。新配置不再列出该开关。
- 继续支持系统持续运行、每项试验使用全量 rosbag 录制的流程。
  其他话题、校准／保护阈值、控制判据和服务行为保持兼容。

## 2. 内部命名规则

| 类别 | 规则 | 示例 |
| --- | --- | --- |
| 变量、属性、函数 | `snake_case`，对象与含义明确 | `confirmed_dac_voltage_v`、`_on_adc_samples()` |
| 类型 | `PascalCase` | `AdcNode`、`GainControlNode` |
| 常量／质量位 | `UPPER_SNAKE_CASE` | `CALIBRATION_FALLBACK` |
| 电压 | `_v` | `raw_voltage_v`、`target_voltage_v` |
| 时长 | `_s`；高精度驱动计时用 `_ns` | `conversion_timeout_s` |
| 单调时钟时间点 | `_monotonic_s` 或 `_monotonic_ns` | `last_command_monotonic_ns` |
| ROS 时间点 | `_stamp` | `sample_stamp`、`received_stamp` |
| 采样组频率 | `_hz` | `sample_group_rate_hz` |
| ADC 硬件转换率 | `_sps` | `conversion_rate_sps` |
| 无量纲比例 | `_ratio` | `turn_decision_ratio` |
| 转换序号 | `_sequence` | `sample_sequence` |
| 接口对象 | `_publisher`、`_subscriber`、`_client` | `samples_publisher`、`set_dac_voltage_client` |

ADC 的硬件转换率与三通道发布频率分开命名；单调时钟与 ROS 时间分开命名；
DAC 的请求值与确认值分开命名。`adc`、`dac`、`i2c`、`ros` 等通用缩写保留。
函数内临时循环索引可以使用 `i` 等简短名称，物理量不使用含糊的 `v`／`t`／`g`
作为其主要变量名。历史格式适配处允许沿用协议字段名。

## 3. 本次主要内部名称

| 位置／原名称 | 当前内部名称 |
| --- | --- |
| ADC `bus_number`、`address`、`channels` | `i2c_bus_id`、`i2c_address`、`channel_ids` |
| ADC `data_rate`、`channel_rate` | `conversion_rate_sps`、`sample_group_rate_hz` |
| ADC `timeout`、`poll_interval` | `conversion_timeout_s`、`ready_poll_interval_s` |
| ADC `gain_control_state` | `dac_voltage_state`（有效性／电压的原子快照） |
| ADC `raw_means` | `raw_voltage_means` |
| DAC `port`、`baud`、`default_channel` | `serial_port`、`serial_baud_rate`、`dac_channel_id` |
| DAC `timeout`、`safe_voltage` | `serial_timeout_s`、`safe_voltage_v` |
| DAC `applied_voltage` | `applied_voltage_v` |
| 增益节点 `current_dac_voltage` | `confirmed_dac_voltage_v` |
| 增益节点 `commanded_dac_voltage` | `requested_dac_voltage_v` |
| 增益节点 `last_adc_monotonic_s` | `last_valid_adc_received_monotonic_s` |
| 增益节点 `latest_batch_valid` | `latest_sample_group_valid` |
| 周期计算 `head`／`left`／`right` | `head_voltage_mean_v`／`left_voltage_mean_v`／`right_voltage_mean_v` |
| 周期计算 `difference` | `left_right_difference_ratio`，以左右平均值归一化 |
| 周期计算 `compensated` | `turn_decision_ratio`，加入电压差补偿后的实际判据 |
| 固定窗口 `lr_difference` | `left_right_sum_difference_ratio`，以左右之和归一化 |
| 驱动 `command`／`echo` 局部变量 | `command_bytes`／`echo_bytes` |

节点回调统一使用 `_on_...`。驱动转换起止、增益接收和控制窗口等内部时间量
显式区分时钟和单位。所有计算仍使用原来的输入、顺序、阈值和边界条件。

## 4. 新参数名及旧参数兼容

`get_compatible_param()` 按“新名称、旧名称、默认值”查找，使用是否显式设置
判断，`false`、0 和空字符串都不会被当作缺省值。新旧名称同时设置时新名称优先。

为保持未修改的 launch 参数覆盖行为，现有 `config/io.yaml` 中旧键继续有效，
没有把新旧默认值同时写入 YAML。现场无需为这次内部重命名同步修改其他包。

以下均为节点私有参数；如 ADC 的完整新参数为 `~adc/sample_group_rate_hz`。

| ADC 旧键 → 新键 | DAC 旧键 → 新键 |
| --- | --- |
| `bus` → `i2c_bus_id` | `port` → `serial_port` |
| `address` → `i2c_address` | `baud` → `serial_baud_rate` |
| `channels` → `channel_ids` | `channel` → `channel_id` |
| `data_rate` → `conversion_rate_sps` | `timeout` → `serial_timeout_s` |
| `channel_rate` → `sample_group_rate_hz` | `safe_voltage` → `safe_voltage_v` |
| `conversion_timeout` → `conversion_timeout_s` | `command_timeout` → `command_timeout_s` |
| `ready_poll_interval` → `ready_poll_interval_s` | `reconnect_interval` → `reconnect_interval_s` |
| `reconnect_interval` → `reconnect_interval_s` | |
| `frame_id` → `device_id` | |
| `gain_control_topic` → `dac_state_topic` | |

增益参数继续兼容 `~gain_control/<旧键>` 和历史 `~agc/<旧键>`。
新键使用 `~gain_control/` 前缀；未列出的参数名称保持不变。

| 旧键 | 新键 |
| --- | --- |
| `start_voltage` | `start_voltage_v` |
| `dac_channel` | `dac_channel_id` |
| `service_timeout`、`adc_timeout` | `service_timeout_s`、`adc_timeout_s` |
| `manual_voltage`、`manual_step_v` | `manual_target_voltage_v`、`manual_voltage_step_v` |
| `target_min_v`、`target_max_v` | `target_min_voltage_v`、`target_max_voltage_v` |
| `step_v`、`max_normal_step_v` | `voltage_step_v`、`max_normal_voltage_step_v` |
| `consecutive_samples` | `consecutive_sample_group_count` |
| `dac_min_v`、`dac_max_v` | `dac_min_voltage_v`、`dac_max_voltage_v` |
| `adc_safety_limit_v`、`adc_safety_recovery_v` | `adc_safety_trigger_voltage_v`、`adc_safety_recovery_voltage_v` |
| `safety_step_v`、`manual_ramp_step_v` | `safety_voltage_step_v`、`manual_ramp_voltage_step_v` |
| `window_control` | `window_control_enabled` |
| `window_mean_min_v`、`window_peak_upper_v` | `window_mean_min_voltage_v`、`window_peak_max_voltage_v` |
| `window_high_fraction` | `window_high_sample_ratio` |

## 5. 保留兼容名称的边界

| 边界 | 处理 |
| --- | --- |
| 全部 `.msg`／`.srv` | 文件、字段和 MD5 不变，包括 `volt_raw`、`status_cali` 等历史名称 |
| 原有 ROS 话题与服务地址 | 除明确移除的单通道 ADC 话题外，地址不变 |
| 录入配置 JSON 的 `settings` | 保留已有键与结构，写入重命名后的真实有效值，不增加双份字段 |
| ADC 配置 JSON 的 `topics` | 移除已不存在的 `sample` 条目；保留 `samples` 和 `dac_state` |
| `SteeringSettings`／`CycleDecision` | 构造参数、字段及 `vars(settings)` 结果不变，供外部运动节点直接使用 |
| `CycleSteering`、`adapt()`、驱动与增益控制器公开接口 | 公开参数、结果属性／字典键保持兼容，内部计算变量规范化 |
| 校准表与模型 JSON | 原文件与 schema 不变 |
| 历史 rosbag | 不重写、不移动，按其嵌入式消息定义读取 |

因此，本轮不会让旧运动节点突然需要读取 `raw_voltage_v` 等新 ROS 字段；
这些新名称首先用于包内计算。将来若允许跨模块接口迁移，再单独调整消息字段。

## 6. 验证入口

先构建并 source 对应 devel 或 install 环境，再执行：

```bash
python3 -m unittest discover -s robots/robotic_fish_sonic/test -p 'test_*.py'
python3 robots/robotic_fish_sonic/test/bag_compatibility.py
python3 robots/robotic_fish_sonic/test/adc_ros_smoke.py
python3 robots/robotic_fish_sonic/test/ros_smoke.py
```

`adc_ros_smoke.py` 使用独立 localhost ROS master、模拟 I²C 和真实全量录包回读，
覆盖批数据唯一发布、校准、DAC 标记、一秒均值以及晚开始录包的锁存配置。
它不启动真实 DAC、spinal 桥或运动执行器，需使用空闲的测试端口 11430。
测试结果与限制另见 [VALIDATION.md](../VALIDATION.md)。
