# 单舵机运动调整验证记录

验证日期：2026-10-20。

本次将 `sonic_teleop.py` 的轨迹计算和手柄状态机提取到
`src/robotic_fish/sonic_motion.py`，增加中心偏置限速、停止锁存、
有效 Joy 接收超时和显式恢复。声学输入尚未接入。

## 已完成的检查

| 检查 | 结果 |
| --- | --- |
| 运动与 ROS 适配层单元测试 | 24 项通过，0 错误、0 失败 |
| `robotic_fish_sonic` 回归测试 | 91 项通过 |
| 原版正弦轨迹对照 | 392 个样本一致，浮点误差小于 `1e-10` |
| 干净构建、测试和安装 | 在 `/tmp` 下独立构建／安装空间完成 |
| 安装后的 Python 模块与节点 | 可导入；与源文件内容一致 |
| launch 配置加载 | 默认参数和覆盖参数两种情况通过 |
| 安装后的节点通信测试 | 独立 ROS master、合成 Joy 和测试话题下通过 |
| 其他运动程序 | `teleop.py`、`advance_teleop.py`、`spring_teleop.py` 与修改前一致 |

通信测试未启动桥、ADC、DAC 或执行器。测试 master 使用本机端口 11428；
输入、输出分别重映射到 `/motion_test/joy` 和 `/motion_test/targets`。
启用 `/use_sim_time` 且不发布 `/clock`，确认墙上时间控制循环仍正常运行：

- 自动摆尾覆盖正负峰值，输出间隔符合约 50 Hz，目标处于软件范围内。
- 停止后持续回中；保持自动键不会恢复。
- 释放按键并回中后，新按手动键可恢复；中心偏置按设置速率变化。
- 短 Joy 消息和有效消息中断均停止运动；连接恢复不会自行恢复运动。
- 显式释放并重新按自动键可恢复；节点正常退出。

独立构建目录为 `/tmp/robotic-fish-motion-check`，其中保留构建、测试、
安装和 launch 检查日志。通信测试日志位于 `/tmp/fish-motion-ros-wj61x6ml`。
这些临时目录仅是本次本机验证证据，不是仓库运行依赖。
本机旧版 nose 在 Python 3.10 下的兼容处理也仅位于 `/tmp`。

## 复现

先在 ROS 工作空间编译 `robotic_fish` 并 source 对应的 setup 文件，
再从仓库根目录执行：

```bash
python3 -m unittest discover -s robots/robotic_fish/test -v
python3 robots/robotic_fish/test/sonic_ros_smoke.py
```

安装空间验证可为通信测试增加 `--node-script`，指向安装后的
`lib/robotic_fish/sonic_teleop.py`。通信测试使用的端口需空闲。

## 尚待现场确认

本次未连接真实舵机、手柄或机器鱼，未执行水中实验，也未运行远端 CI。
软件限位、左右转向符号、摆频／摆幅和偏置限速仍需现场确认。
Joy 看门狗验证消息接收，不验证无线链路状态；真实手柄断连时驱动是否
继续重复旧消息，以及串口失联时执行器的行为，需要单独实测。
声信号到中心偏置的映射属于后续阶段。
