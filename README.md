# 智能车 STC32G12K128 协作代码说明

本项目是开放类智能车赛道的协作代码，目标是先稳定完成基础循迹和路口处理，再逐步调试速度、死路、起点/终点停车等高级逻辑。

当前硬件和工程环境：

- MCU：`STC32G12K128`
- 编译环境：`Keil C251`
- 主控工程：`PWM.uvproj`
- 主逻辑入口：`main.c`
- 当前核心控制：Timer2 中断内每约 1ms 执行一次巡线状态机

## 协作原则

- 这是实车迭代代码，不要一次性大改多个问题。
- 改控制逻辑前，先明确本轮只验证一个目标，例如十字、T 字、全白找线或普通 PID。
- 改完必须记录实车现象：通过次数、失败现象、下一步，记录到 `迭代计划.md`。
- 不要改传感器 bit 位序，除非同步修改 README、注释和所有判断逻辑。
- 不要删除状态机注释。后续协作者需要靠这些注释判断为什么这么写。
- 成绩规则优先避免冲出赛道，再优化速度。

## 传感器 bitmask

Timer2 每次有效控制周期会读取 5 路灰度/循迹传感器，并压成 `line_mask`。

| bit | 变量 | 含义 |
| --- | --- | --- |
| `0x10` | `zuo2` | 左外 |
| `0x08` | `zuo1` | 左内 |
| `0x04` | `zhong` | 中线 |
| `0x02` | `you1` | 右内 |
| `0x01` | `you2` | 右外 |

辅助判断：

```c
has_center = (line_mask & 0x04) != 0;
has_left   = (line_mask & 0x18) != 0;
has_right  = (line_mask & 0x03) != 0;
is_white   = (line_mask == 0);
is_line    = line_mask 是单个传感器位;
is_junction = has_center && has_left && has_right;
```

## 当前状态机

当前状态变量是 `junction_state`，默认状态是 `LINE`。

| 状态 | 业务含义 | 动作 | 退出条件 |
| --- | --- | --- | --- |
| `LINE` | 正常巡线 | 调用 `RunLinePid(...)` | 全白进 `SEARCH`；十字进 `JUNCTION`；多个非十字进 `TURN` |
| `JUNCTION` | 十字路口 | `PWM_Run(line_base_speed, line_base_speed)` 直行 | 离开十字组合后进 `LOCKOUT` |
| `TURN` | T 字、直角、普通转弯统一处理 | 按 `last_turn_dir` 原地旋转 | 重新看到单传感器后回 `LINE` |
| `SEARCH` | 五路全白，可能是丢线、死路、转弯后找线 | 按 `last_turn_dir` 原地旋转找线 | 看到线后按当前 mask 进入对应状态 |
| `LOCKOUT` | 十字通过后的冷却 | 复用 PID 巡线，但不重新触发十字 | 连续 `LOCKOUT_LINE_TICKS` 次看到单传感器后回 `LINE` |

`last_turn_dir` 是方向记忆：

- `TURN_LEFT = 0`
- `TURN_RIGHT = 1`
- 默认值：`TURN_RIGHT`
- 全白找线、转弯动作都会复用这个方向
- 多传感器非十字时，只有单侧明确有线才更新方向；左右都有但中线没有时沿用上次方向

## mask 分类

当前逻辑下 32 种传感器组合会被这样分类：

| mask | 传感器组合 | 状态 |
| --- | --- | --- |
| `0x00` | 无 | `SEARCH` |
| `0x01` | `you2` | `LINE` |
| `0x02` | `you1` | `LINE` |
| `0x03` | `you1 + you2` | `TURN`，方向更新为右 |
| `0x04` | `zhong` | `LINE` |
| `0x05` | `zhong + you2` | `TURN`，方向更新为右 |
| `0x06` | `zhong + you1` | `TURN`，方向更新为右 |
| `0x07` | `zhong + you1 + you2` | `TURN`，方向更新为右 |
| `0x08` | `zuo1` | `LINE` |
| `0x09` | `zuo1 + you2` | `TURN`，方向沿用上次 |
| `0x0A` | `zuo1 + you1` | `TURN`，方向沿用上次 |
| `0x0B` | `zuo1 + you1 + you2` | `TURN`，方向沿用上次 |
| `0x0C` | `zuo1 + zhong` | `TURN`，方向更新为左 |
| `0x0D` | `zuo1 + zhong + you2` | `JUNCTION` |
| `0x0E` | `zuo1 + zhong + you1` | `JUNCTION` |
| `0x0F` | `zuo1 + zhong + you1 + you2` | `JUNCTION` |
| `0x10` | `zuo2` | `LINE` |
| `0x11` | `zuo2 + you2` | `TURN`，方向沿用上次 |
| `0x12` | `zuo2 + you1` | `TURN`，方向沿用上次 |
| `0x13` | `zuo2 + you1 + you2` | `TURN`，方向沿用上次 |
| `0x14` | `zuo2 + zhong` | `TURN`，方向更新为左 |
| `0x15` | `zuo2 + zhong + you2` | `JUNCTION` |
| `0x16` | `zuo2 + zhong + you1` | `JUNCTION` |
| `0x17` | `zuo2 + zhong + you1 + you2` | `JUNCTION` |
| `0x18` | `zuo2 + zuo1` | `TURN`，方向更新为左 |
| `0x19` | `zuo2 + zuo1 + you2` | `TURN`，方向沿用上次 |
| `0x1A` | `zuo2 + zuo1 + you1` | `TURN`，方向沿用上次 |
| `0x1B` | `zuo2 + zuo1 + you1 + you2` | `TURN`，方向沿用上次 |
| `0x1C` | `zuo2 + zuo1 + zhong` | `TURN`，方向更新为左 |
| `0x1D` | `zuo2 + zuo1 + zhong + you2` | `JUNCTION` |
| `0x1E` | `zuo2 + zuo1 + zhong + you1` | `JUNCTION` |
| `0x1F` | 全部识别 | `JUNCTION` |

## PID 巡线逻辑

`RunLinePid(line_mask, line_sum, line_count)` 是普通巡线函数。

- 左侧传感器权重为负：`zuo2=-20`，`zuo1=-10`
- 中线权重为 0：`zhong=0`
- 右侧传感器权重为正：`you1=10`，`you2=20`
- `line_value = line_sum / line_count`
- `line_error = -line_value`
- `pid_output` 使用整数 PD：`line_pid_kp` 和 `line_pid_kd`

效果：

- 黑线偏右：左轮更快，右轮更慢，车向右修
- 黑线偏左：左轮更慢，右轮更快，车向左修
- PWM 最终限制在 `-100 ~ 100`

## 关键参数

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| `line_base_speed` | `80` | 普通巡线基础速度 |
| `line_pid_kp` | `20` | 位置误差比例项 |
| `line_pid_kd` | `13` | 误差变化微分项 |
| `line_pid_limit` | `25` | PID 输出限幅 |
| `TURN_SPEED` | `85` | 强制转向原地旋转速度 |
| `SEARCH_SPEED` | `85` | 全白找线原地旋转速度 |
| `LOCKOUT_LINE_TICKS` | `20` | 十字冷却退出所需连续单线次数 |

## 已知风险

- `is_junction` 判定较宽，只要中线、左侧、右侧同时识别就会当十字。T 字、黑方块边缘、弯道过渡可能误判。
- `SEARCH` 优先级最高，全白会直接覆盖当前状态。如果十字区域内短暂全白，可能绕开十字冷却。
- `TURN` 看到一次单传感器就会交回 `LINE`，没有连续确认，噪声可能导致提前退出。
- `junction_tick` 当前只计数，不参与最大直行时间保护。

这些风险不是 bug 结论，而是下一轮实车验证时应重点观察的点。

## 实车测试建议

建议按顺序验证：

1. 单传感器巡线：直线、缓弯、绕圈。
2. 左右普通转弯：确认 `last_turn_dir` 更新方向正确。
3. 全白找线：确认默认右转是否符合赛道实际。
4. 十字路口：确认只直行，不反复触发。
5. T 字和直角：确认不会被误判为十字。
6. 完整赛道：记录冲出赛道次数和失败位置。

每轮测试后把记录写入 `迭代计划.md` 的表格，不要只凭印象继续改参数。

## 修改建议

如果后续继续迭代，优先按这个顺序小步改：

1. 给 `TURN` 增加连续确认或最短转向时间。
2. 加硬 `LOCKOUT`，避免全白短暂覆盖十字冷却。
3. 收紧 `JUNCTION` 判定，减少 T 字和黑方块误判。
4. 再单独设计死路掉头和终点停车。

