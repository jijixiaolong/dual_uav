# 确认后的架构：MuJoCo 仿真 + PX4 真机控制 + ROS 2 协调

## 研究任务
两架 PX4 四旋翼水平空中对接、连接后飞行、分离；研究连接与释放事件造成的瞬态误差，利用少量重复试验历史形成**事件状态条件化短时补偿 ECLR**，比较传统 ILC、固定前馈和成熟名义组合体几何控制。

## 最重要的架构决定
**仿真只用 MuJoCo。** 不要求 Gazebo、CoppeliaSim，也不要求先集成 PX4 SITL。把可以部署于 PX4 的控制器核心写成不依赖 ROS2/MuJoCo 的纯 C++17 模块，在桌面仿真测试，并在未来编译接入 PX4。

```
       SIMULATION                                   HARDWARE (later)
  MuJoCo 2 rigid bodies                            two physical PX4 flight controllers
  contact + equality weld                          motors + latch mechanism
       | simulated sensed state                          ^ local feedback
       v                                                 |
  thin MuJoCo adapter ---> SAME C++ CONTROL CORE <--- thin PX4 uORB adapter
                                  ^
                                  | bounded learned parameters / event configuration
                           offline Python trainer
                                  ^
                            log records (simulation or ROS2)
                                  |
                       ROS2 in actual test: Vicon, multi-aircraft coordinator,
                       synchronization, event supervisor, learning data logging
```

## 飞控职责
- **每机 PX4**：低层姿态/角速度稳定、合法执行器接口、紧急回退；新增确定性事件补偿/协调控制模块（以 PX4 原控制栈接口为准）。不允许两套控制模块同时独立向同一电机发布互相冲突的命令。
- **ROS 2**：双机动捕与坐标变换、远端相对状态与任务模式协调、实验编排和日志、离线学习参数加载。通信必须设置时间戳与过期处理。
- **MuJoCo**：验证对接碰撞、磁力/连接件接触与刚性约束、释放时预载、八电机动力学、控制延迟及不确定性。
- **训练端**：仿真阶段 Python 拟合事件响应和优化前馈系数；每次新事件使用事前冻结的系数，PX4 内执行有界光滑补偿。

## 为什么代码要可移植
如果一开始用 Python 写名义控制器、ECLR、安全投影，后来全部重写进 PX4，论文实验与实机将不是相同实现，符号、采样时间和限幅也容易不一致。因此共用同一控制核心，MuJoCo/PX4 只提供不同状态输入和执行器输出适配器。

## 必须验证的物理细节
1. MuJoCo 两个 `freejoint` 刚体，每个四个独立电机；质量、惯量、力矩臂和电机模型参数化。
2. 接触/捕获与 weld 建立**不是同一件事**：先接触，再按明确定义的几何与锁定判据启用 weld，`mjData.eq_active` 只能表达连接约束，不能代替磁力。
3. weld 的目标相对位姿需与真实接触姿态一致，避免求解器因重接约束不一致产生虚假大冲量。
4. 无预载释放应保持速度连续；有预载释放要通过实际连接件弹性/力模型体现，而不是瞬间随机改变姿态角。
5. 每个事件分别测试纯几何控制、平滑切换、固定前馈、传统 ILC、ECLR，对应输入上限必须公平。
6. MuJoCo 的模型是否更接近实机，取决于实验测得的质量/惯量/电机响应/连接件刚度/预载，不能仅靠引擎名称推断。

## PX4 部署待固定的工程事实
- PX4 固件版本和飞控板型号/算力、内存；双机通信链路及更新频率；是否已存在自定义几何控制器。
- ROS 2 版本、`px4_msgs` 的匹配分支、PX4 uORB 订阅/发布点、外部 Vicon 数据进入 EKF 的流程。
- 组合体控制谁生成共同力矩目标、两机怎样同步分配、通信断开且机械连接仍存在时的安全状态。

先把接口抽象为 `State`, `Mode`, `EventFeatures`, `NominalWrench`, `Correction`, `AllocatedInputs` 六类数据结构，具体 uORB Topic 待固定 PX4 版本时再映射。注意 ROS ENU/FLU 与 PX4 NED/FRD 坐标差异。

## 相关官方文档
- MuJoCo equality constraints: https://mujoco.readthedocs.io/en/stable/programming/simulation.html
- PX4 external modules: https://docs.px4.io/main/en/advanced/out_of_tree_modules
- PX4 uORB: https://docs.px4.io/main/en/middleware/uorb
- PX4 ROS 2: https://docs.px4.io/main/en/ros/ros2_comm
- PX4 Vicon/OptiTrack: https://docs.px4.io/main/en/tutorials/motion-capture

详细执行任务以 `CODEX_TASK_PX4_MUJOCO.md` 为准。`CODEX_TASK.md` 是更早版本，不要再把其 Python-first / 可选 Gazebo 内容作为优先事项。
