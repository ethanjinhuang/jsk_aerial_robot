# 2026-11-12：补全 1.30 V 与实测支持边界

稳压电源版本校准 ID：`adc_independent_transfer_20261112_060020_new_circuit_psu_130_observed_support`。
当前默认已切换为[2026-11-14 电池供电校准](calibration_20261114_battery.md)，本表保持原样用于追溯和供电条件切换。
原 125 节点表和仅补节点但覆盖率不足的候选均保留，未覆盖原始数据。

## 数据与阈值

实验仍为操作者确认的三通道 5 mVpp 公共电气输入、稳压电源供电。
信号频率及 ADS1115 芯片 VDD 未提供，保持未知记录；没有通过软件确认模拟链路是否削波。

初次扫描使用 4.00 V 触发阈值，完成 0–1.24 V。
后续按操作者要求，仅在补测会话将提前触发点改为数字满量程饱和判定
4.095875 V，DAC 始终限制在 0–1.30 V。正式运行配置仍为 ADC 4.00 V、DAC 5.0 V。
这些阈值不代表 ADC 引脚耐压或模拟链路线性认证。

数据目录：

```text
/home/khadas/calibration_runs/new_circuit_20261112T050340Z_verified/
  ascending_sweep.bag              # 0–1.24 V，125 档，每档10秒
  independent_repeat.bag           # 13个独立平台

/home/khadas/calibration_runs/new_circuit_20261112T060020Z_supplement/
  supplement_120_130.bag           # 1.20–1.30 V，11档，每档10秒
  independent_high_repeat.bag      # 1.25/1.27/1.29/1.30 V独立复测
  rejected_extended_candidate/    # 仅使用中位数端点、覆盖不足的旧候选
  measured_support_candidate/     # 本次通过验证的候选与摘要

/home/khadas/calibration_runs/new_circuit_20261112T060020Z_endpoint/
  endpoint_training.bag            # 1.30 V延长训练60秒
  endpoint_independent.bag         # 1.30 V重新归零后的独立复测30秒
```

1.20–1.24 V 重叠区间各通道平台中位数最大相对漂移约 0.1274%，
在预设 2% 合并检查界限内。保留原始 0–1.24 V 节点，添加补测 1.25–1.29 V
及延长训练的 1.30 V 节点，共 131 个平台中位数节点。

## 为什么增加边界节点

末档中位数不是实际观测范围的上限。仅使用中位数端点时，两轮 1.30 V 独立复测
分别有 82/176 和 70/575 组超出曲线端点容差；该候选未部署。

新的边界只从首、末稳定训练平台提取，不使用独立复测决定范围：

| 通道 | 末档中位数 (V) | 训练实测上界 (V) | 训练实测下界 (V) |
| --- | ---: | ---: | ---: |
| ADC0 | 4.068375 | 4.089625 | 0.009250 |
| ADC1 | 4.055250 | 4.079500 | 0.008625 |
| ADC2 | 4.062375 | 4.088250 | 0.008875 |

每通道添加上下两个边界节点，节点原始电压来自上述实测值；参考值采用相邻两个
平台中位数节点的局部线性模型估计。因此每通道总计 **133 个节点**。
边界节点在 JSON 中标记 `node_kind=measured_support_boundary` 和
`reference_estimated=true`，`measured_support` 保存中位数与实测范围。

这是明确的边界模型估计，不是新增精密参考测量。运行时无需更改校准加载器：
仍在存储节点之间线性插值，不把边界内的不同输入统一压成一个平台中位数。
超出实测支持范围后，仍只允许原有 2.5 mV 端点夹紧容差；更远的输入整组回退。
4.095875 V 没有被当成校准表的输入上界。

## 独立验证

三份独立复测共 18 个稳定平台、3,562 组稳定样本。边界建立时没有使用这些样本。
所有样本可应用新曲线，回退数为 0；每个平台绝对差异 P95 都低于校准前。

| 三通道差异指标 | 原始 | 校准后 |
| --- | ---: | ---: |
| 绝对差异中位数 | 13.375 mV | 0.389 mV |
| 绝对差异 P95 | 60.875 mV | 2.346 mV |
| 相对差异中位数 | 3.313% | 0.0377% |
| 相对差异 P95 | 4.902% | 0.488% |

1.30 V 两轮独立复测分别为 176 和 575 组，均无回退。
校准后相对差异 P95 分别为 0.0729% 和 0.0376%。
不同阶段样本集合与初次报告不同，整体百分位不能直接作为前后版本的公平对比；
这里报告的是当前候选在全部保留独立数据上的结果。

检查门槛：每个平台覆盖率至少95%、整体覆盖率至少99%、各平台绝对差异P95改善。
本轮全部通过。边界斜率只在当前实验数据支持的区间内验证，不能据此推广到
未采集的幅值、频率、温度、供电或更弱声信号配合更高 DAC 增益的条件。

## 可复现重建

编译并 source 后运行，输出目录必须不存在：

```bash
rosrun robotic_fish_sonic rebuild_new_circuit_calibration.py \
  /home/khadas/calibration_runs/new_circuit_20261112T050340Z_verified \
  /home/khadas/calibration_runs/new_circuit_20261112T060020Z_supplement \
  --endpoint /home/khadas/calibration_runs/new_circuit_20261112T060020Z_endpoint \
  --include-measured-endpoints \
  --output /tmp/fish_calibration_rebuilt
```

工具对每份 bag 分别核对其记录阈值、全量原始过压事件和饱和码，检查重叠漂移后合并，
并核对原扫描 125 档、补测 11 档及独立复测的平台计划，防止缺档或错档仍被合并。
再用三份独立数据计算覆盖率与误差。检查不通过会拒绝生成可部署候选；
`--save-rejected` 仅用于带失败标记的结果归档。
源 bag 路径、SHA-256、各阶段阈值和逐平台结果均嵌入新校准文件。
