# PX4 Firmware Out-of-Tree Porting Guide for Dual-Quadrotor Docking

## 1. 架构定位

本工程的核心算法库 `core/` 使用标准 **C++17** 编写，**零外部依赖**（不包含任何 ROS 2、uORB、MuJoCo 或 Linux 特有系统调用），采用头文件与纯函数/纯状态设计。这意味着在桌面 MuJoCo 仿真中经过严格测试的同一个控制核心库，可以直接复制或作为 git submodule 编译进 PX4 飞控固件。

```
                     PX4 Flight Controller (Single Quad)
+-------------------------------------------------------------------------+
|                                                                         |
|   uORB Subscriptions:                                                   |
|   - vehicle_local_position (NED position, velocity)                    |
|   - vehicle_attitude       (FRD quaternion)                            |
|   - vehicle_angular_velocity (body angular rates)                       |
|   - external_supervision   (FSM command from ROS 2 Vicon Coordinator)   |
|                                |                                        |
|                                v                                        |
|   +-----------------------------------------------------------------+   |
|   |                  PX4 Adapter Module (Thin Wrapper)              |   |
|   |  - Coordinate Transformation (NED/FRD <-> ENU/FLU)             |   |
|   |  - Converts uORB messages to docking::RigidBodyState            |   |
|   +----------------------------+------------------------------------+   |
|                                |                                        |
|                                v                                        |
|   +-----------------------------------------------------------------+   |
|   |               Pure C++17 Control Core (docking_core)            |   |
|   |  - docking::GeometricSE3Controller (Single or Connected)        |   |
|   |  - docking::SingleQuadAllocator / ConnectedQuadAllocator       |   |
|   |  - docking::HybridFSM (Mode Transitions & Bumpless Handover)    |   |
|   |  - docking::EventCompensator (Bounded Online Feedforward)       |   |
|   |  - docking::SafetyMonitor (Tilt & Rate Sanity Checks)           |   |
|   +----------------------------+------------------------------------+   |
|                                |                                        |
|                                v                                        |
|   uORB Publications:                                                    |
|   - vehicle_torque_setpoint [tx, ty, tz]                                |
|   - vehicle_thrust_setpoint [0, 0, -fz/fz_max]                          |
|   - actuator_motors         [T0, T1, T2, T3] normalized                |
+-------------------------------------------------------------------------+
```

---

## 2. 坐标系与符号映射 (Frame Transformations)

PX4 飞控内部采用 **NED (North-East-Down)** 导航系与 **FRD (Forward-Right-Down)** 机体坐标系。
本仿真核心库采用 **ENU (East-North-Up)** 导航系与 **FLU (Forward-Left-Up)** 机体坐标系。

### 2.1 位置与速度转换
$$p_{\text{FLU}} = \begin{bmatrix} 0 & 1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & -1 \end{bmatrix} p_{\text{FRD}}, \quad v_{\text{FLU}} = \mathcal{T}(v_{\text{FRD}})$$

### 2.2 四元数与角速度转换
- PX4 四元数表示机体 FRD 相对于导航 NED 的旋转：$q_{\text{NED}\to\text{FRD}}$。
- 转换到 FLU 姿态后，角速度向量分量转换：
  $$\omega_x^{\text{FLU}} = \omega_x^{\text{FRD}}, \quad \omega_y^{\text{FLU}} = -\omega_y^{\text{FRD}}, \quad \omega_z^{\text{FLU}} = -\omega_z^{\text{FRD}}$$

### 2.3 控制扳手输出转换
- 总推力 $F_z^{\text{FLU}} > 0$ 对应 PX4 负向下推力：
  $$\text{thrust\_body\_z}^{\text{PX4}} = -\frac{F_z}{T_{\max}^{\text{total}}}$$
- 滚转力矩 $\tau_x^{\text{FRD}} = \tau_x^{\text{FLU}}$
- 俯仰力矩 $\tau_y^{\text{FRD}} = -\tau_y^{\text{FLU}}$
- 偏航力矩 $\tau_z^{\text{FRD}} = -\tau_z^{\text{FLU}}$

---

## 3. Out-of-Tree 模块工程结构

在 PX4 官方推荐的架构中，用户自定义算法应存放在工作区外的独立代码树中编译：

```
px4_docking_module/
├── CMakeLists.txt
├── Kconfig
├── docking_control.cpp          # uORB 订阅/发布主工作线程
├── docking_control.hpp
└── core/ -> symlink to dual_quad_codex_sim/core/
```

### 3.1 `CMakeLists.txt` 示例
```cmake
px4_add_module(
    MODULE modules__docking_control
    MAIN docking_control
    STACK_MAIN 2500
    PRIORITY "SCHED_PRIORITY_ACTUATOR_OUTPUTS"
    SRCS
        docking_control.cpp
        core/src/geometric_se3.cpp
        core/src/allocator.cpp
        core/src/fsm.cpp
        core/src/compensator.cpp
        core/src/safety.cpp
    INCLUDES
        core/include
    DEPENDS
        px4_work_queue
)
```

---

## 4. 组合体控制权限与双机通信失效保护 (Failsafe)

1. **组合体主从控制划分**：
   - 处于 `CONNECTED` 状态时，由 Master 飞控（例如 Quad 1）计算全局 8 电机分配指令，并将 Quad 2 的 4 电机推力通过微秒级低延迟串口/CAN 链路下发给 Quad 2 飞控；
   - 若双机通信丢失（心跳超时 $> 40\,\text{ms}$），但物理 weld/锁扣仍处于锁定状态：
     > [!WARNING]
     > **严禁独立回退自由飞行模式**：物理连接未解除前，单机独立运行位置控制器会产生严重内力撕扯导致两机翻扣。两机必须保持最后有效的推力平衡基准，共同缓降平飞或实施协同停机保护。
2. **释放瞬间协同**：
   - 释放触发由硬件电磁锁驱动电平同步断开，两机飞控在检测到锁具开锁确认信号后，同步原子切换回 `SEPARATING` 独立模式。

