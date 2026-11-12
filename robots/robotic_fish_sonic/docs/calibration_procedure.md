# 新电路三通道相对一致性校准

## 实验范围与安全阈值

当前信号发生器最小输出为 5 mVpp。在这个输入幅值和本次电路配置下，
DAC 增益控制电压扫描上限设为 **1.30 V**。该上限是实验条件，不是 DAC
硬件的上限，也不是正式运行的通用增益上限。实际声信号更弱时可以使用更高增益，
正式配置中的 `gain_control.dac_max_v` 保持 5.0 V。

ADC 软件过压触发阈值与 DAC 电压上限是两个独立参数。当前线上默认 ADC
触发阈值为 **4.00 V**；离线工具优先读取 bag 中 gain_control 配置的
`settings.safety_limit_v`，达到或超过该值的平台整段剔除，包括稳定等待期间的过压。
不再用固定的 3.30 V 作为校准上限。**3.30 V 仍是线上过压恢复阈值**，
保留滞回，避免在触发边界附近反复切换。

这个阈值统一的是软件判定策略，不代表已经验证新电路在 4.00 V 以下都线性、
或 ADC 引脚的电气耐压。示波器发现提前削波的平台仍应排除，缩小实验扫描范围。

本轮后续补测按操作者要求仅在校准会话取消 4.00 V 提前触发，使用
4.095875 V 数字饱和判定，才完成到 1.30 V 的采集。这不是引脚安全电压认证。
正常运行的 4.00 V 默认值不变。只在明确选择这种校准会话时使用：

```bash
roslaunch robotic_fish_sonic sensor_io.launch gain_mode:=off adc_safety_limit_v:=4.095875
```

默认 4.00 V 会话中，当前电路在 1.30 V 会触发保护，应按摸底结果缩小范围；
不能直接沿用 1.30 V 目标又忽略该会话实际记录的阈值。

## 公共输入与采集

本方法校准三通道相对响应，不提供外部精密基准下的绝对电压或 LNA 增益。

1. 将相同的电气输入送入三个通道的等效输入点。固定输入频率、5 mVpp 幅值、
   偏置、供电、接地和屏蔽。记录电路版本、仪器幅值的负载设定和实测输入幅值。
2. 采用实际运行的 ADC 配置：ADS1115 128 SPS、三通道完整样本组约 20 Hz。
3. 停止手柄增益控制和闭环调节，使用 `gain_mode:=off`，保留软件过压保护。
   校准工具只拟合 `volt_raw`，旧 `volt_cali` 仅用于前后比较，不影响新曲线拟合。
4. 使用 `sensor_io.launch` 启动传感器，另开终端先开始录包：

   ```bash
   roslaunch robotic_fish_sonic sensor_io.launch gain_mode:=off
   ```

   ```bash
   roslaunch robotic_fish record.launch bag_prefix:=/home/khadas/new_circuit_sweep
   ```

5. 先确认 DAC 为 0 V，并通过示波器检查输出。摸底时使用较少平台
   （0、0.2、0.4、0.6、0.8、1.0、1.1、1.2、1.25、1.3 V），单独录制。
6. 正式数据另录 bag，从 0.00 V 到 1.30 V，步长 0.01 V，每档至少 10 秒。
   共 131 档，约 21 分 50 秒。每次通过 DAC 服务设置并确认成功：

   ```bash
   rosservice call /robotic_fish/dac/set_voltage "channel: 1
   voltage: 0.0"
   ```

   这里的示例只设置 0 V；后续目标必须由实验操作者或专用采集程序限制在 0–1.30 V。
   `--dac-max-v` 是离线筛选参数，不会限制硬件服务输出，也不会自动执行扫描。
7. 每档检查 ADC 原始值和输出波形，发现削波或过压立即停止扫描。
   上升扫描不要混入下降或反复调节；安全降压发生后应结束该轮实验。
8. 录制结束前将 DAC 恢复到 0 V，然后 Ctrl+C 正常关闭 bag。
   下降扫描和独立复测分别录制，以免和上升拟合数据混淆。

默认稳定窗口剔除平台开始后 1.00 秒和结束前 0.25 秒。每个平台至少保留
16 组稳定样本。工具在首次确认 DAC 降低后排除后续平台。

录包必须至少包含 `/robotic_fish/adc/samples` 和
`/robotic_fish/gain_control/config`；默认 `robotic_fish/record.launch` 已包含。
后者保存实际线上安全阈值，供离线工具核对。
原始 ADC 检查先于 DAC 反馈有效性过滤；反馈无效的组也不能掩盖过压、
饱和码或非有限原始电压。仅审计时会记录这些问题；生成候选表时会拒绝整轮
存在上述问题的实验，应将摸底与正式有效扫描分别录包。

## 生成候选校准

确认实验确实采用三通道公共输入后执行：

```bash
rosrun robotic_fish_sonic calibrate_from_bag.py \
  /home/khadas/new_circuit_sweep_TIMESTAMP.bag \
  --output /tmp/new_circuit_calibration \
  --common-input \
  --dac-max-v 1.30
```

输出目录必须不存在。输出包含 `summary.json`、`recorded_config.json`、
`plateaus.csv` 和候选 `candidate_calibration.json`。
候选生成要求时间留出验证至少 99% 的样本可应用曲线，且校准后绝对差异 P95
不高于校准前；失败时退出且不创建输出目录。去掉 `--common-input` 可以单独
生成审计报告，查看 `raw_observation_audit` 和平台排除原因，不生成候选表。

工具对每个稳定平台计算三通道中位数，再以三个中位数的中位值作为公共参考。
每个通道形成独立分段线性曲线，运行时查询不依赖 DAC。

`summary.json` 明确记录 `adc_safety_limit_v`、`safety_limit_source` 和
`experimental_dac_max_v`。默认记录的 ADC 阈值是 4.00 V；如果采集时覆盖为
3.50 V，则离线自动采用 3.50 V。旧 bag 没有控制配置时，可明确提供：

```bash
rosrun robotic_fish_sonic calibrate_from_bag.py old_sweep.bag \
  --output /tmp/old_sweep_calibration --common-input \
  --dac-max-v 1.30 --adc-safety-limit-v 4.00
```

没有记录也没有显式参数时，回退读取包内 `config/io.yaml`，来源在摘要中标注；
源码和安装后的工具均支持该定位方式。
该回退不能证明历史实验实际使用该阈值。显式阈值与 bag 配置冲突、或同一个 bag
包含多个触发阈值时，工具拒绝生成，应先拆分实验数据。

## 独立验证与部署

前半稳定样本拟合、后半稳定样本验证只是同一轮实验的时间留出验证。
必须另录一轮独立数据，使用中间 DAC 平台，并在不同输入幅值或增益条件下复测。
比较原始与校准后三通道差值、中位误差、P95 差异、回退比例和重复测量漂移。
接近噪声底时优先比较绝对误差。

DAC 到 1.30 V 是否足够取决于 ADC 输出覆盖范围。超出已观测曲线的正式数据
仍整组回退原始值，不做外推。如果需要固定 DAC、改变信号源幅值补充采样，
当前按 DAC 平台分段的工具不能直接生成多平台曲线，应使用对应的幅值分段工具。

新候选文件不会自动部署。验证通过后，使用新的唯一名称保存到
`config/calibration/`，再更新 `config/io.yaml` 的 `adc.calibration.file`。
同时保留原始 bag、实验条件、摘要和验证结果；旧校准保留用于追溯。

## 已观测边界电压的纳入方式

若末档稳定采样的波动超过“中位数节点 + 2.5 mV”，可以显式使用
`--include-measured-endpoints`。工具在 131 个平台中位数节点之外，
按首、末稳定训练平台的实际最小/最大输入添加边界节点，因此本轮每通道共 133 节点。
每个边界节点用相邻两个中位数节点的斜率估计参考值，并标记 `reference_estimated`。
运行时对这些已存储节点做线性插值，保留边界附近读数的变化。

边界只从训练样本建立。验证样本不用于扩大支持范围；实际输入若超出支持范围及
原有 2.5 mV 端点容差，仍然整组回退。不能单凭低于 ADC 满量程就纳入校准。
重新验证时同时报告每个平台的有效覆盖率、回退数和误差，避免只挑选可校准样本。
本轮完整重建及独立复测命令见[实测边界补充报告](calibration_20261112_measured_support.md)。
