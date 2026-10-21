# 第一、二阶段交付：物理诊断与名义组合控制

日期：2026-10-21。范围：按本次任务最后一段要求，完成代码根因审计、P0 物理模型及 P1 名义控制；未进入学习调参。环境、源码 SHA256、配置和原始数据见 `results/physics_validation/`。所有本轮图表均由真实 MuJoCo 轨迹生成。

## 1. 原六项异常的结论

完整原始证据见 [实验审计](stage1_experiment_audit.md) 和 [物理审计](stage1_physics_audit.md)，原 CSV 未改写。

| 原异常 | 查实的根因 | 本轮处置 |
|---|---|---|
| pitch 约 0.001° | 沿 Y 对接主要激发 roll；旧评估只看 Q1 pitch 且只取锁定后的 CONNECTED 段，遗漏此前碰撞和分离 | 新增两机真实 SO(3)、roll/pitch 跟踪误差及全程物理日志，分别保存接触、捕获就绪、锁定、释放时刻 |
| Fixed FF 比 ECLR 好 | 原数据本就有负结果；ECLR 训练/执行特征错位，八电机补偿主要作用 roll，评价却看 pitch | 保留负结果；暂时停用旧学习评测入口，后续独立修复，未改造数据使 ECLR 胜出 |
| 15 工况只有 33 行 | 旧代码仅保留成功样本，baseline/fixed/ILC/ECLR 分别 9/8/10/6 行，仅 4 个完整共同工况 | 已定位；本轮物理场景全数保存，未冒充完成 15×5 学习对照；其配对统计仍属下一阶段 |
| settling 全 1.2 s | 初始值设 1.2，全程不越界时从不修改；末点越界也未标截尾 | 已定位；本轮不使用旧恢复时间作验收指标。正式指标重写、边界测试及截尾统计待第三阶段 |
| 接触/约束作用不可核查 | 原日志未保存力与动量；捕获无接触门控，固定 weld 参考有约 11.3 mm 闭合误差 | 已修复并增加独立物理实验、分离的接触/weld 载荷及事件前后全状态 |
| surrogate 与 MuJoCo 不一致 | surrogate 直接作用角加速度；原 MuJoCo 确有电机链路，但事件特征和控制方向不匹配 | 明确分离两种数据；本轮全部使用 C++ 核心→分配→电机→MuJoCo，无学习补偿 |

原仿真还存在实际 250 Hz 却传 200 Hz 的 dt、积分器/分配器/参考跨试验污染、单机单位惯量、没有真正使用的 handover alpha、补偿绕过最终 slew 等问题；均在本轮运行链修复。初始 7 项测试通过并未发现这些问题，因此增加了独立物理不变量及跨模块验收。

## 2. 最终物理与控制实现

- 两个独立 `freejoint`、显式质量和惯量、八个旋翼 site motor 均保留。没有在事件时写入姿态、位置或速度。全仿真为 SI：m、kg、s、N、N·m、rad；CAD 文件仍保持原样。
- 世界 Z 向上、机体 FLU、四元数 wxyz。名义每机质量 1 kg，惯量 `diag(0.0082,0.0082,0.0145) kg·m²`，旋翼半径臂 0.17 m、坐标 ±0.1202081528 m、反扭矩系数 0.016 m。配置中的转向符号表示机体反作用力矩，实际桨旋向相反。
- 名义连接间距 0.44 m、总质量 2 kg，平行轴组合惯量 `diag(0.1132,0.0164,0.1258) kg·m²`。实际捕获后使用真实相对平移/朝向，旋转惯量与电机轴均同步更新。质量、惯量、旋翼位置/力矩参数、连接点和 YAML 不一致时立即报错。
- 接触机构为 40×20×40 mm **平面方形接触垫的近似模型**，表面位于机体系 Y=±0.22 m。没有磁吸、真实导向锥、释放器柔性或经过标定的制造几何。不得把 weld 激活解释成磁力动力学。
- 捕获同时要求真实连接器接触、连接点距离≤20 mm、相对姿态≤0.08 rad、连接点相对线速度≤0.10 m/s、相对角速度≤0.20 rad/s，并驻留 30 ms。冻结当前真实锚点及相对四元数，避免把机体拉到错误的固定名义参考。
- 经过独立核对，MuJoCo 编译后的 weld `eq_data` 为 `[anchor2, anchor1, relative_quaternion, torquescale]`；非零 anchor 时，MJCF 的平移量不能简单当作两机原点间距。本轮测试涵盖非名义位置与相对朝向的零初始约束误差。
- FSM 保留原细分模式 `FREE/APPROACH/ALIGN/CAPTURE_PENDING/CONNECTED/RELEASE_PREP/SEPARATING/RECOVERY/ABORT`，并输出任务所用 `phase` 名称。释放命令必须等待真实锁定后的最短 1 s 驻留，分离轨迹从实际释放时刻及状态初始化。
- 连接时只运行统一组合体控制器；共同坐标轴跟随 Q1，组合质心质量加权，角速度由总角动量及组合惯量恢复，避免四元数平均和错误坐标系混合。分离时恢复独立控制、重置积分、同步八电机历史，生成 C2 参考。
- 有界优化分配的主任务为可控的 `[Fz, τx, τy, τz]`。真实水平力照常作用物理系统并被记录，不能作为独立的零目标硬优化。每机不同的推力/变化率限制位于优化可行域内，末端电机链还有共同保护。
- 500 Hz 物理步上，200 Hz 控制按真实物理时间执行，间隔交替 4/6 ms。正常工作电机范围 0.05–5.5 N；真正关闭电机允许 0 N；最终目标和实际推力均有 45 N/s 限制与 25 ms 一阶响应。指令在旋翼推力空间衔接，不插值不同模式的扳手。

## 3. 修复过程中新增发现及处理

### 3.1 共面圆柱接触退化

平面对齐圆柱的临时修复版本，在预连接零弹簧悬停中出现接触/weld 数百牛顿互抵，而质心仍看似稳定。隔离关闭连接面碰撞后载荷消失，证明不能只看质心和高度判断通过。比较方形平面垫与球形接触后，选择方形垫；保留真实接触及 weld。10 s 预连接回归明确要求零弹簧初始夹具接触力、weld 力均小于 0.1 N，防止以冗余大内力掩盖问题。

### 3.2 不可控水平力目标导致内部预载

捕获相对 roll 只有约 −0.368°，临时六维目标却强求 Q1 坐标系水平力为零。两组略有倾斜的推力轴不能独立满足该目标，优化器用电机饱和换取很小的横向力降低，导致释放前约 1 N·m 内部 roll 力矩；weld 持续抵消这部分力矩。释放后电机状态和滞后会保留该负载，产生约 24.80° 真实姿态误差。不是 teleport，也不是评价轴假象。

改为四个可控任务后，同一工况释放前电机约 2.44–2.46 N、最大局部力矩约 0.00371 N·m；释放后前 1.2 s 姿态误差峰为 3.865°。这属于名义物理/控制修复，不是学习收益。`core/tests/test_core.cpp` 与闭环测试已增加小倾角捕获、推力裕度、内部力矩、释放角速度及姿态峰回归。该次中间异常轨迹保留在 `results/physics_validation/intermediate_six_dof_diagnostics.tar.gz`，不属于最终结果。

## 4. 零预载与弹簧预载的物理含义

真正的理想释放实验在零重力、八电机关闭、没有接触压缩且运动学一致的预连接夹具中进行。解除 weld 不改变任何 qpos/qvel，释放后也不产生新的速度或动能。

非零预载使用中心弹簧：刚度 60 N/m、初压缩 8 mm、阻尼 2 N·s/m。两个锚点位于接触面后方各 4 mm，力沿两锚点连线作用，保证等反力与净内部力矩为零。初始弹力 0.48 N、储能 0.00192 J。预充能/锁定装置的充能功显式记账，未仿真加载器的详细运动；这是仿真假设。释放不改速度，后续分离运动来自有限储能，卸载后弹簧力归零。

完整有控场景 `zero_preload=True` 仅表示关闭额外释放弹簧，接触/weld 仍可能存在小内部载荷。因此将其命名为“零释放弹簧场景”，不等同于理想零预载守恒实验。接触软约束和软 weld 的数值弹性能未纳入报告的机械能，守恒和能量耗散分别在无外力、已声明的初末条件下检查。

## 5. 数据与复现

```bash
python3 -m pip install -r requirements.txt
python3 scripts/validate_stage2.py
# 相同的完整入口：
python3 run_experiments.py --mode physics
# 视频（真实共享仿真循环，12 s / 360 frames）：
MUJOCO_GL=egl python3 scripts/render_video.py --output results/videos --width 960 --height 540
```

每个场景保存配置、500 Hz 原始 CSV、事件前后全状态 JSON、PASS/FAIL、全程及事件窗口图、三维轨迹图。`manifest.json` 保存源码 SHA256、解释器和库版本；`requirements-validation.lock.txt` 是本次 Ubuntu/Python 3.10 环境的版本记录。视频与交互显示均调用同一个 `SimulationRunner`，不再维护第二套飞行逻辑。

日志区分当前时刻接触力、上一积分步接触法向力/累计冲量、weld 等式行单独广义力，以及锚点处的等效力与自由力偶。冲量为引擎力的时间积分估计，包含持续接触的部分，不能直接称为纯冲击冲量。

`events.json` 的 CONTACT / CAPTURE_READY / DOCK / UNDOCK 区分接触起始、捕获条件、约束锁定及释放；`INITIAL_CONNECTED` 是测试初始条件，不计作飞行捕获成功。所有事件全状态采样在精确切换时刻。时序行在步末，首个新模式行比实际切换晚 2 ms；后续指标必须按事件字段对齐。

姿态误差由当前状态相对采样保持的活动控制器期望姿态重新计算；绝对角另列。连接时单机参考速度/加速度字段留空并设置无效标记，组合平移参考导数另列，避免把冻结的独立控制器参考作为学习数据。机体参考位置包括组合姿态与捕获力臂；其精确速度还需期望角运动导数，不在当前字段中伪造。

未捕获负例也保留完整时序及状态。`missed_capture` 采用 +30 mm 末端间距偏差，可能有短暂接触，但最终未满足捕获驻留；其预期结果为不激活 weld，而不是“全程完全无接触”。真正“距离容差内但无接触不应捕获”另有单元测试。

## 6. 本轮边界与后续任务

已完成：六项异常的代码级根因；P0/P1 在已声明名义与夹具场景下的物理、控制和可复现性验收；独立保留负例；完整真实数据及可视化。

尚未完成且没有宣称完成：正式恢复时间/截尾指标与置信区间；15 工况×5 控制器的公平矩阵；B3 观测器；修复 ECLR 特征及独立事件模型；ILC/ECLR 训练、泛化、多随机种子、模型失配/延迟与消融。旧代码的这些问题已定位，学习入口暂时停用，避免继续产出不公平结论。

接触垫刚度/摩擦、质量惯量、电机模型、预载均未实物标定；本轮不是制造级连接机构验证或 RA-L 算法优势证明。PX4/ROS 2 仍为未来适配设计，未运行 SITL、未改固件、未部署真机。后续必须继续按第三至六阶段顺序执行。

## 7. 主依据

- [MuJoCo weld XML 语义](https://mujoco.readthedocs.io/en/stable/XMLreference.html#equality-weld)：锚点、相对姿态及软约束定义。
- [MuJoCo simulation API 说明](https://mujoco.readthedocs.io/en/stable/programming/simulation.html)：运行时 equality 激活、外加载荷持久化及接触坐标系。
- [MuJoCo 官方约束实现](https://github.com/google-deepmind/mujoco/blob/main/src/engine/engine_core_constraint.c)：weld 锚点顺序及等式约束 Jacobian。

以下测试与数值表由本轮最终输出自动附加。

## 8. 最终运行实测

最终结果：12/12 个 C++ 案例、27/27 个 pytest 项、3/3 组独立物理实验、6/6 个闭环/负例场景均 PASS。闭环原始 CSV 共 25500 行。源 SHA256 与最终文件逐一核对一致。

以下位置/速度/姿态为末 0.5 s 最大活动参考误差或速度；释放姿态峰为真实释放后前 1.2 s 的两机最大 SO(3) 误差，未计算恢复时间。

| 场景 | 状态 | 末段位置误差 mm | 末段速度 mm/s | 末段姿态误差 deg | 释放后1.2s窗口峰 deg |
|---|---|---:|---:|---:|---:|
| independent_hover | PASS | 1.44134e-12 | 3.24948e-12 | 0 | — |
| preconnected_hover | PASS | 2.26626 | 0.234356 | 0.0747169 | — |
| connected_tracking | PASS | 2.63476 | 0.829183 | 0.048975 | — |
| contact_capture_release | PASS | 0.202212 | 0.505431 | 0.00158696 | 3.864548180313307 |
| preloaded_capture_release | PASS | 0.681495 | 0.482051 | 0.00165547 | 3.810676394095159 |
| missed_capture | PASS | 3.02475 | 1.5762 | 0.00256261 | — |

标称场景首次接触在 2.722 s、真实锁定在 4.906 s、真实释放在 6.110 s。首次碰撞显著早于锁定，后续学习与指标必须分别定义接触事件和锁定事件，不能再次忽略锁定前瞬态。

| 独立物理实验 | 状态 | 实测 |
|---|---|---|
| unforced_collision | PASS | 接触峰 34.1394 N；法向冲量估计 0.222635 N·s；线动量最大残差 6.94e-16 kg·m/s |
| zero_preload_release | PASS | 瞬时 Δqpos/Δqvel=0；释放后全速度变化 0；机械能变化 0 J |
| spring_preload_release | PASS | 初始储能 0.00192 J；释放前约束负载 0.481643 N；最终动能 0.00114345 J；弹簧最终储能 0 |

### 逐项测试状态

| 类型 | 测试 | 状态 |
|---|---|---|
| C++ | `test_math_foundation` | PASS |
| C++ | `test_geometric_se3_hover` | PASS |
| C++ | `test_geometric_attitude_error_scale` | PASS |
| C++ | `test_single_quad_allocator` | PASS |
| C++ | `test_connected_allocator_and_parallel_axis` | PASS |
| C++ | `test_connected_actual_capture_geometry` | PASS |
| C++ | `test_feasible_wrench_reconstruction_and_redistribution` | PASS |
| C++ | `test_allocator_limits_slew_and_handover` | PASS |
| C++ | `test_tilted_capture_rotor_axes` | PASS |
| C++ | `test_small_capture_tilt_does_not_create_control_preload` | PASS |
| C++ | `test_hybrid_fsm` | PASS |
| C++ | `test_safety_monitor` | PASS |
| pytest | `test_asymmetric_tilted_allocator_matches_independent_bvls` | PASS |
| pytest | `test_independent_closed_loop_hover` | PASS |
| pytest | `test_preconnected_aggregate_hover` | PASS |
| pytest | `test_preconnected_aggregate_tracks_smooth_reference` | PASS |
| pytest | `test_capture_release_and_independent_hover_recovery` | PASS |
| pytest | `test_reused_runner_resets_integrators_and_allocator_history` | PASS |
| pytest | `test_aggregate_attitude_is_invariant_to_antipodal_quaternion_encoding` | PASS |
| pytest | `test_docking_and_undocking_cycle` | PASS |
| pytest | `test_zero_preload_release_velocity_continuity` | PASS |
| pytest | `test_freefall_gravity` | PASS |
| pytest | `test_single_and_dual_hover_equilibrium` | PASS |
| pytest | `test_unforced_connector_collision_conserves_linear_momentum_and_dissipates_energy` | PASS |
| pytest | `test_xml_motors_really_turn_off_without_residual_idle_thrust` | PASS |
| pytest | `test_final_motor_path_obeys_amplitude_and_rate_limits_after_arbitrary_input` | PASS |
| pytest | `test_nonfinite_or_wrong_size_motor_commands_do_not_advance_simulation[command0]` | PASS |
| pytest | `test_nonfinite_or_wrong_size_motor_commands_do_not_advance_simulation[command1]` | PASS |
| pytest | `test_nonfinite_or_wrong_size_motor_commands_do_not_advance_simulation[command2]` | PASS |
| pytest | `test_reported_momentum_and_energy_match_independent_rigid_body_sum` | PASS |
| pytest | `test_capture_requires_real_contact_even_inside_position_tolerance` | PASS |
| pytest | `test_contact_capture_checks_speed_and_preserves_state_at_switch` | PASS |
| pytest | `test_weld_reference_uses_actual_relative_pose_without_initial_constraint_error` | PASS |
| pytest | `test_zero_preload_release_preserves_all_velocities_momentum_and_energy` | PASS |
| pytest | `test_preloaded_release_has_finite_energy_and_balanced_internal_force` | PASS |
| pytest | `test_preload_is_an_internal_wrench_with_transverse_connector_misalignment` | PASS |
| pytest | `test_modes_finite` | PASS |
| pytest | `test_learning_finite_and_bounded` | PASS |
| pytest | `test_invalid_mode` | PASS |
