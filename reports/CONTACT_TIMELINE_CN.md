# CONTACT—DOCK 时间差归因（原始 P0/P1 只读复核）

首次接触到锁定的 **2.184 s** 主要来自**第一次接触未满足速度/姿态捕获条件后弹跳、脱离，直到第二次低速接触**。不是长期持续接触而未锁定，也不是反复清零捕获驻留计时。未修改物理、控制器、状态机或捕获阈值；本分析直接复用 [原始 MuJoCo CSV](../results/physics_validation/contact_capture_release/timeseries.csv)、[events.json](../results/physics_validation/contact_capture_release/events.json) 和配置。

## 时间与状态序列

| FSM 控制采样时刻 / s | 状态变化 | 对应输出 CSV 时刻 / s |
|---|---|---|
| 0.000 | 初始化 → FREE | 0.002 |
| 1.000 | FREE → APPROACH | 1.002 |
| 2.440 | APPROACH → ALIGN | 2.442 |
| 4.870 | ALIGN → CAPTURE_PENDING | 4.872 |
| 4.906 | CAPTURE_PENDING → CONNECTED | 4.908 |
| 6.006 | CONNECTED → RELEASE_PREP | 6.008 |
| 6.110 | RELEASE_PREP → SEPARATING | 6.112 |
| 6.740 | SEPARATING → RECOVERY | 6.742 |
| 7.746 | RECOVERY → FREE | 7.748 |

独立事件时间：CONTACT **2.722 s**，CAPTURE_READY **4.870 s**，DOCK **4.906 s**，UNDOCK **6.110 s**。CSV 每次物理步执行完才记录；控制切换在本步开始，故其对应 CSV 行晚 2 ms。重建每个控制判据时使用前一行的物理状态，不能把切换输出行状态作为判据输入。共重建 6000 个物理样本、2400 个控制采样；捕获状态/驻留重放与实际 FSM **0 处不一致**。

- 第一次接触对象存在于 **2.722–2.760 s**（首个无接触样本 **2.762 s**），半开时间段为 [2.722, 2.762)，共 40 ms。两个接触点是同一次接触的接触流形，并非两次弹跳。
- 正法向接触力只持续至 **2.738 s**，2.740 s 起为零，尽管接触对象暂时还在碰撞检测集合中。初次法向力为 **31.536357 N**。
- **2.762–4.870 s 完全无连接器接触，共 2.108 s**。连接点距离最大为 **107.606 mm**，发生于 **3.552 s**。径向相对速度从负（靠拢）变正（分离），随后重新变负，明确显示一次脱离—回接。
- **4.542 s** 起所有运动学判据均满足，但接触判据仍不满足；因此必须等待 4.870 s 的实际再接触。
- **4.870 s** 二次接触立即进入 CAPTURE_PENDING / CAPTURE_READY；整个过程中 CAPTURE_PENDING 仅进入 **1 次**、捕获前退出 **0 次**、驻留重置 **0 次**。
- 从 CAPTURE_READY 到 DOCK 共 **36 ms**。配置驻留为 30 ms，实际控制步交替为 4/6 ms。4.900 s 的累加驻留为 `0.029999999999996696`，因现有 C++ 使用直接 `>= 0.03` 比较，未在该采样锁定，下一 6 ms 采样于 4.906 s 锁定。该浮点比较延迟只解释 6 ms，不能解释 2.184 s 的主体。

因此 2.184 s = **2.148 s 等待二次合格接触 + 0.036 s 驻留**。未通过放宽物理阈值改变结果。未来如修正驻留计时的浮点边界，应单独回归并更新事件时间，不应改捕获速度或姿态门槛。

## 判据与控制参考证据

| 量 | 首次接触 2.722 s | 二次接触 4.870 s | 捕获阈值 |
|---|---:|---:|---:|
| 连接点距离 / mm | 2.046601 | 0.176362 | ≤ 20 |
| 连接点相对速度 / m/s | 0.369260 | 0.043088 | ≤ 0.1 |
| 两机相对姿态 / rad | 0.121339 | 0.009950 | ≤ 0.08 |
| 两机相对角速度 / rad/s | 0.070163 | 0.009866 | ≤ 0.2 |
| 法向接触力 / N | 31.536357 | 3.449822 | 接触对象必须存在，无最低力阈值 |

首次接触时相对速度和相对姿态同时失败。初碰速度消减后，姿态仍超限，随后角速度也超限，最终脱离接触。控制/物理保护正确拒绝了不合格捕获。

| 失败判据 | [CONTACT, DOCK) 失败总时长 / s |
|---|---:|
| contact | 2.108 |
| position | 1.634 |
| linear_speed | 1.034 |
| orientation | 0.094 |
| angular_speed | 0.408 |

各判据失败可重叠，不能把时长相加。完整逐步判据及各控制采样驻留均见 CSV。

APPROACH、ALIGN、CAPTURE_PENDING 共用同一独立控制器和以指令时刻计时的五次多项式接近轨迹；CONTACT 没有控制器或参考分支切换。轨迹在 **3.200 s** 结束，而首次碰撞提前 **0.478 s** 发生。首次接触 Q1 的 Y 位置为 **-0.220619 m**，活动参考为 **-0.247678 m**；参考速度仍为 **0.150776 m/s**，加速度为 **-0.452911 m/s²**。两机在减速阶段仍需相反滚转：活动期望相对姿态为 **7.246469°**，真实相对姿态为 **6.952234°**，均高于捕获门槛 **4.583662°**。

所以存在“到达物理接触时轨迹还在减速、参考姿态尚未对齐”的时序问题；证据不支持参考坐标系错误、CONTACT 时任务级轨迹指令分支跳变或提前启用组合体参考。**C2 连续的是位置/速度/加速度指令；活动姿态参考由 Geometric 控制器根据这些指令和反馈状态重算，不能假定其也连续。**碰撞迅速改变速度反馈后，控制采样 2.720→2.726 s 的活动期望相对姿态由 7.246469° 降为 4.040581°，这是反馈诱导的姿态参考变化，不是新任务轨迹的切换。仅在实际 DOCK 后才初始化组合参考。若后续优化接近策略，应让实际触碰前的速度/姿态已进入捕获集合，保留现有物理保护。本次未为提高成功率调整任何阈值。

## 真实产物与复现

- [判据、距离、速度、接触力及 FSM 图](../results/event_attribution/contact_timeline/contact_to_dock.png)
- [全部 FSM 状态序列](../results/event_attribution/contact_timeline/full_fsm_sequence.png)
- [控制采样与驻留精确重放](../results/event_attribution/contact_timeline/capture_dwell.png)
- [首次接触前后的真实状态和活动参考](../results/event_attribution/contact_timeline/reference_at_contact.png)
- [逐物理步判据 CSV](../results/event_attribution/contact_timeline/physical_step_capture_criteria.csv)
- [逐控制采样判据和驻留 CSV](../results/event_attribution/contact_timeline/control_tick_capture_dwell.csv)
- [事件时间线 CSV](../results/event_attribution/contact_timeline/event_timeline.csv)
- [接触分段 CSV](../results/event_attribution/contact_timeline/contact_intervals.csv)
- [正法向力分段 CSV](../results/event_attribution/contact_timeline/positive_contact_force_intervals.csv)
- [数值汇总 JSON](../results/event_attribution/contact_timeline/summary.json)、[源文件 SHA-256 与复现配置](../results/event_attribution/contact_timeline/manifest.json)

```bash
python3 scripts/analyze_contact_timeline.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_contact_analysis.py -q
```

本分析不重新仿真，不把相邻样本的插值作为实际采样。事件之前的初始 t=0 状态未在原 CSV 保存；首个控制采样明确标记 `input_exact_available=false`，其余均有精确对应输入。新增单元测试覆盖各捕获边界/非有限值、零载荷接触、接触断续、控制输入错位、驻留清零和现有浮点驻留延迟；**10/10 通过**，见[测试日志](../results/event_attribution/contact_timeline/python_tests.log)和[JUnit XML](../results/event_attribution/contact_timeline/pytest.xml)。当前系统 anyio 自动加载插件与 pytest 版本不兼容，复现时关闭第三方插件自动加载；项目测试本身正常执行。
