# 智能车 STC32G12K128 协作代码说明

本项目是开放类智能车赛道的协作代码。当前版本目标不是一次性做完所有复杂赛道元素，而是先用一个清晰、可协作、可实车验证的状态机处理基础巡线、十字直行、转向和全白找线。

## 项目信息

- MCU：`STC32G12K128`
- 编译环境：`Keil C251`
- 工程文件：`PWM.uvproj`
- 主逻辑文件：`main.c`
- 当前核心逻辑：Timer2 中断中约每 1ms 执行一次巡线状态机
- Gitee：`https://gitee.com/chenhaotwooooo/smart-car_-stc32-g12-k128`
- GitHub：`https://github.com/zhang-chenhao-creator/stc32g-smart-car`

## 协作原则

- 这是实车迭代代码。每次只改一个主要问题，不要同时改 PID、路口判定、速度和状态机。
- 改控制逻辑前，先明确本轮验证目标，例如普通巡线、十字、T 字、全白找线。
- 改完后要记录实车结果：测试元素、通过次数、失败现象、下一步，写入 `迭代计划.md`。
- 不要改传感器 bit 位序，除非同步修改 `main.c` 注释、README 和所有判定逻辑。
- 不要删除状态机注释。后续协作者需要靠注释理解为什么这样判断。
- 比赛成绩按时间和冲出赛道次数综合计算，当前优先级是少冲出赛道，再优化速度。

## Timer2 运行节奏

Timer2 配置为 100us 中断一次，但代码中用 `line_pid_tick` 做节流：

```c
line_pid_tick++;
if(line_pid_tick < 10)
{
    return;
}
line_pid_tick = 0;
```

因此巡线状态机约每 1ms 真正执行一次。README 中所有 `tick` 都按约 1ms 理解。

## 传感器 bitmask

Timer2 每个有效周期会读取 5 路传感器，并压成 `line_mask`。

| bit | 变量 | 位置 | PID 权重 |
| --- | --- | --- | --- |
| `0x10` | `zuo2` | 左外 | `-20` |
| `0x08` | `zuo1` | 左内 | `-10` |
| `0x04` | `zhong` | 中线 | `0` |
| `0x02` | `you1` | 右内 | `10` |
| `0x01` | `you2` | 右外 | `20` |

基础判定：

```c
has_center  = (line_mask & 0x04) != 0;
has_left    = (line_mask & 0x18) != 0;
has_right   = (line_mask & 0x03) != 0;
is_white    = (line_mask == 0);
is_line     = line_mask 是单个传感器位;
is_junction = has_center && has_left && has_right;
```

## 状态机总览

状态变量是 `junction_state`，默认状态是 `LINE`。状态机分两步执行：先根据传感器组合更新状态，再根据状态输出一次 PWM。

| 状态 | 业务含义 | 触发条件 | 动作 | 退出条件 |
| --- | --- | --- | --- | --- |
| `LINE` | 普通单线巡线 | 只有一个传感器识别黑线 | `RunLinePid(...)` | 全白进 `SEARCH`；十字进 `JUNCTION`；多个非十字进 `TURN` |
| `JUNCTION` | 十字路口 | 中线、左侧、右侧同时识别 | `PWM_Run(line_base_speed, line_base_speed)` 直行 | 离开十字组合后进 `LOCKOUT` |
| `TURN` | 转向处理 | 多个传感器识别但不是十字 | 按 `last_turn_dir` 原地旋转 | 重新看到单传感器后回 `LINE` |
| `SEARCH` | 全白找线 | `line_mask == 0` | 按 `last_turn_dir` 原地旋转 | 看到线后重新按当前 mask 判断 |
| `LOCKOUT` | 十字冷却 | 从 `JUNCTION` 离开后进入 | 复用 PID，但不触发新十字 | 连续 `LOCKOUT_LINE_TICKS` 次看到单传感器后回 `LINE` |

## 状态转移优先级

当前代码的状态转移优先级是固定的：

1. 如果当前是全白，直接进入 `SEARCH`。
2. 如果当前已经在 `JUNCTION`，继续直行；离开十字组合后进入 `LOCKOUT`。
3. 如果当前已经在 `LOCKOUT`，忽略新的十字触发；只有连续稳定看到单传感器才回 `LINE`。
4. 如果普通状态下检测到十字，进入 `JUNCTION`。
5. 如果普通状态下只有一个传感器识别，进入 `LINE`。
6. 其余多个传感器组合进入 `TURN`。

这个优先级的含义是：全白最强，十字需要冷却，普通转向最后处理。

## 方向记忆

`last_turn_dir` 记录最近一次明确转向方向：

- `TURN_LEFT = 0`
- `TURN_RIGHT = 1`
- 默认值是 `TURN_RIGHT`

方向更新规则：

- 只有左侧有黑线：更新为左转。
- 只有右侧有黑线：更新为右转。
- 左右都有但不是十字：不更新方向，沿用上一次方向。
- `SEARCH` 和 `TURN` 都复用 `last_turn_dir`。

## PID 巡线逻辑

`RunLinePid(line_mask, line_sum, line_count)` 用于 `LINE` 和 `LOCKOUT`。

计算过程：

```c
line_value = line_sum / line_count;
line_error = -line_value;
pid_output = (kp * error + kd * error_delta) / LINE_PID_SCALE;
left_pwm   = line_base_speed - pid_output;
right_pwm  = line_base_speed + pid_output;
```

效果：

- 黑线偏右：`line_value > 0`，车向右修。
- 黑线偏左：`line_value < 0`，车向左修。
- PWM 最终限制在 `-100 ~ 100`。
- `line_count == 0` 时不计算 PID，避免除零。

## 32 种 mask 分类

| mask | 传感器组合 | 当前分类 |
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

## 关键参数

| 参数 | 当前值 | 作用 |
| --- | --- | --- |
| `line_base_speed` | `80` | 普通巡线基础速度 |
| `line_pid_kp` | `20` | PID 比例项 |
| `line_pid_kd` | `13` | PID 微分项 |
| `line_pid_limit` | `25` | PID 输出限幅 |
| `TURN_SPEED` | `85` | TURN 原地旋转速度 |
| `SEARCH_SPEED` | `85` | SEARCH 原地找线速度 |
| `LOCKOUT_LINE_TICKS` | `20` | LOCKOUT 退出需要的连续单线次数，约 20ms |

## 当前已知风险

- `is_junction` 判定较宽，只要中线、左侧、右侧同时识别就当十字。T 字、黑方块边缘、弯道过渡可能误判。
- `SEARCH` 优先级最高，全白会覆盖当前状态。如果十字区域内短暂全白，可能绕开十字冷却。
- `TURN` 没有方向锁定和连续确认，看到一次单传感器就会回 `LINE`，转向中可能提前交给 PID。
- `junction_tick` 当前只计数，不参与超时保护。

这些风险是下一轮实车验证重点，不要在没有实车现象的情况下同时修改多个点。

## 实车测试顺序

建议按这个顺序验证：

1. 普通单线：直线、缓弯、绕圈，确认 PID 不明显抖动。
2. 全白找线：确认默认右转方向是否符合赛道和车体安装。
3. 普通左右转：确认 `last_turn_dir` 更新方向正确。
4. 十字路口：确认直行，不误转，不在同一区域重复触发。
5. T 字和直角：确认不会被误判为十字。
6. 完整赛道：记录冲出赛道次数和失败位置。

## 后续修改建议

如果继续迭代，建议按顺序小步修改：

1. 给 `TURN` 加方向锁定、最短转向时间或连续找回线确认。
2. 加硬 `LOCKOUT`，避免短暂全白绕开十字冷却。
3. 收紧 `JUNCTION` 判定，减少 T 字和黑方块误判。
4. 单独设计死路掉头。
5. 最后做起点/终点黑方块识别和停车。

