# 双四旋翼 MuJoCo 对接 / 分离仿真

捕获机构力学、碰撞过滤和随机边界审计见 [捕获修复技术报告](reports/CAPTURE_PHYSICS_AUDIT_CN.md)。

当前交付为 **P0/P1 物理与名义控制验收**：两个独立六自由度刚体、八电机、被动接触锁扣、组合体 SE(3) 控制、实际捕获位姿的约束分配，以及有限储能分离。控制核心为同一套 C++17；无需 PX4 SITL、ROS 2 或 Gazebo。

第一阶段审计见 [实验与学习根因](reports/stage1_experiment_audit.md) 和 [物理根因](reports/stage1_physics_audit.md)。本轮验收和限制见 [阶段二技术报告](reports/STAGE2_REPORT_CN.md)。既有 `results/mujoco_eval/` 和 synthetic 数据已保留，不能作为修复后算法结论。旧 `full` 学习对照入口已停用，等待阶段三至五修复配对、指标和特征协议。

## 运行

已验证环境：Ubuntu 22.04、Python 3.10.12、MuJoCo 3.16.0、GCC 11、CMake；具体 Python 包版本写入每轮 `manifest.json`，另见 `requirements-validation.lock.txt`。编译需 Python 开发头文件；视频需 FFmpeg 和 EGL/OpenGL。

```bash
python3 -m pip install -r requirements.txt
# 构建、C++/Python 测试、独立物理守恒实验及六个闭环场景；任一门槛失败则非零退出。
python3 scripts/validate_stage2.py
# 等价主入口：python3 run_experiments.py --mode physics
```

输出目录 `results/physics_validation/`：

- `manifest.json`：源文件 SHA256、环境及包版本。
- `python_tests.log`、`pytest.xml`、`test_status.json`、`cpp_tests.log`：逐项测试状态。
- `invariants/`：无外力碰撞、理想零预载释放、带储能弹簧释放的 CSV/JSON/图。
- 每个闭环场景：`config.json`、500 Hz `timeseries.csv`、精确切换时刻的 `events.json`、门槛与实测 `summary.json`、诊断图及三维轨迹图。
- 总 `summary.json`：全部闭环场景的 PASS/FAIL。未捕获场景也保留原始记录。

```bash
python3 run_experiments.py --mode smoke
python3 scripts/validate_physical_events.py
# 三维视频与交互显示使用同一个 SimulationRunner 控制/物理循环。
MUJOCO_GL=egl python3 scripts/render_video.py --output results/videos --width 960 --height 540
python3 scripts/view_interactive.py --speed 1
```

## 模型约定

仿真使用 **SI 单位：m、kg、s、N、N·m、rad**；与 CAD 文件的 mm 单位不同。世界 Z 向上，机体系 FLU（X 前、Y 左、Z 上），四元数为 wxyz，自由关节平移速度在世界系、角速度在机体系。每机质量 1 kg、惯量 `diag(0.0082,0.0082,0.0145) kg·m²`；这些包含连接机构的总惯量均为仿真假设，尚未实物标定。原生 CAD 文件未改动，也未用 CAD 网格替代动力学模型。

MJCF 定义质量、惯量及几何；YAML 定义控制、捕获判据、电机动态与时序。runner 从引擎读取真实惯量，核对旋翼布局，加载 YAML；改变物理模型时应同步核对两者。配置文件位于 `configs/`，P0/P1 门槛与场景在 `physics_validation.yaml`。

两机连接点在机体系 Y=±0.22 m，名义质心距离 0.44 m。接触垫为 40×20×40 mm 方形近似体，表面与连接点重合。**没有经过硬件标定的磁力或真实导向锥模型**。接触后可接合一个有限行程的中心拉力元件：6 mm 进入包络、140 N/m 静态力尺度、10 N·s/m 阻尼，静态拉力最高 0.84 N，含阻尼合力限幅 6 N；两锚点各在接触面后方 4 mm。脱离包络后必须重新接触；锁定另需当前/40 ms 内接触确认及全部严格判据连续驻留 30 ms。接合时从有限内部预充能库转移储能，脱离时回收剩余储能，锁定时耗散剩余储能；耗散和转移单独记账，详见报告。锁定仅成对过滤两个接触垫，第三方碰撞保持有效。捕获需真实接触、位置/姿态/相对速度条件及持续驻留；weld 锁住当前锚点及相对姿态，只改变约束，不修改位置或速度。接触软约束和 weld 属于引擎数值近似，非制造级机构模型。

连接后统一使用质量加权质心状态、Q1 定义的共同姿态轴、组合惯量与八旋翼有界优化分配。分配包括真实捕获朝向的推力轴、力臂、反扭矩及每个电机的上下限/变化率。切换同步电机历史、重置积分状态，并从真实切换状态生成 C2 参考；不对不同模式的控制扳手插值。500 Hz 物理步上，200 Hz 控制以 4/6 ms 交替执行并传真实 dt。所有控制量最终再经公共电机限幅、45 N/s 变化率与 25 ms 一阶响应。

`zero_preload=True` 关闭释放弹簧；空中完整场景仍可能有接触及约束内部载荷，因此不能把它等同于理想无预载。**真正的零预载守恒测试**单独在零重力、关闭电机、无接触压缩的夹具中执行。非零预载采用刚度 60 N/m、初压缩 8 mm、阻尼 2 N·s/m 的有限中心弹簧，初始充能 0.00192 J 明确入账；释放无速度重置，力随弹簧卸载归零。

## 日志与后续阶段

每个物理步保存两机位置/四元数/速度/角速度、参考与真实姿态误差、八电机命令/目标/实际推力、接触点与法向力、weld 单独广义力及锚点等效力矩、弹簧能量、组合质心/姿态/角速度、线/角动量和机械能。接触冲量是积分估计；机械能不含引擎软接触/软 weld 的数值势能；不能在受控有重力场景里直接要求动量守恒。

`events.json` 在切换瞬间保存前后全状态；步末时序第一行相应模式比真实事件晚一个物理步，后续指标必须用事件时间字段对齐。`roll_deg/pitch_deg` 是绝对角；`roll_error_deg/pitch_error_deg` 和 `attitude_error_rad` 是相对当前活动控制参考的误差。期望姿态按控制采样保持，日志状态为步末。

下一阶段依次实施公平配对与恢复时间/截尾统计、B0–B3 传统基线、ILC/ECLR 特征协议与独立事件模型、泛化与消融。本轮未训练学习器，未生成 15×5 对照或优势结论；PX4/ROS 2 仍为接口设计，尚未部署。

## 捕获审计复现

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 scripts/validate_stage2.py --output results/capture_regression_NEW
python3 scripts/validate_capture_audit.py --output results/capture_study_NEW --seeds 20261021 1042 7301 --samples-per-class 6
python3 scripts/audit_cad_mujoco.py --output results/capture_cad_NEW.json
```

随机试验入口拒绝覆盖已有目录。每个成功、失败及异常工况均保存配置、完整物理步 CSV、事件 JSON 与汇总；安全/边界/拒绝工况在运行前分类。`core/build/` 由本机 CMake 重建，不再版本控制缓存与二进制。`event_interfaces.py` 提供事件前特征、有限时域响应、正式指标及因果补偿接口；本轮不训练学习器。
