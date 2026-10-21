# ROS 2 Multi-Vehicle Coordination & Motion Capture Architecture

## 1. ROS 2 系统职责范围

根据 2026-10-21 确认的系统架构：
- **高频飞控闭环不在 ROS 2 中运行**：所有毫秒级姿态/力矩闭环与电机分配必须在飞控硬件（PX4）上以 200~400 Hz 确定性中断执行，避免无线通信/Linux 调度不可预测抖动；
- **ROS 2 的核心职责**：
  1. 动捕（Vicon / OptiTrack）全局毫米级位姿解算与坐标系重映射；
  2. 双机相对几何状态监控与高层试验模式编排（命令触发 `APPROACH`, `UNDOCK`）；
  3. 遥测日志记录（Bag 录制）与离线学习参数广播；
  4. 监视双机心跳与安全边界守护。

```
                     +---------------------------------------+
                     |        Vicon / OptiTrack System       |
                     +-------------------+-------------------+
                                         | 100~200 Hz
                                         v
                     +---------------------------------------+
                     |           ROS 2 Workstation           |
                     |  - mocap_vicon_node (Transforms)      |
                     |  - multi_vehicle_supervisor_node      |
                     |  - experiment_orchestrator & logger   |
                     +---------+-------------------+---------+
                               |                   |
                     Micro-XRCE-DDS      Micro-XRCE-DDS
                     (Wi-Fi / Serial)    (Wi-Fi / Serial)
                               v                   v
                     +-----------------+   +-----------------+
                     |   Quad 1 PX4    |   |   Quad 2 PX4    |
                     |  (docking_core) |   |  (docking_core) |
                     +-----------------+   +-----------------+
```

---

## 2. 动捕话题与 PX4 EKF2 接口映射

PX4 飞控标准接入外部动捕的接口为 `px4_msgs::msg::VehicleVisualOdometry`。

### 2.1 ROS 2 话题流向
1. 动捕节点发布：`/vicon/quad1/pose` (geometry_msgs/msg/PoseStamped, ENU)
2. 转换适配节点输出：`/fmu/in/vehicle_visual_odometry` (px4_msgs/msg/VehicleVisualOdometry, NED)
3. PX4 参数配置：
   - `EKF2_EV_CTRL = 15` (熔合外部视觉/动捕水平位置、高度、偏航与线速度)
   - `EKF2_HGT_REF = 3` (使用 Vision 作为主高度参考源)

---

## 3. 试验编排状态机与安全守卫 (Supervisor Node)

ROS 2 试验协调器发布轻量自定义消息：
```
# msg/DockingSupervision.msg
std_msgs/Header header
uint8 commanded_state # 0: IDLE, 1: APPROACH, 2: DOCK, 3: UNDOCK, 4: ABORT
float32 approach_offset_y
bool emergency_stop
```

### 心跳与网络超时处理：
- PX4 适配模块以 20 Hz 检查 `external_supervision` 消息时间戳；
- 若时间戳滞后 $> 100\,\text{ms}$，PX4 自动保持当前高度并锁定水平位置悬停，不执行未经确认的逼近动作。

