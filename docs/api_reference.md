# API 参考文档

本节介绍 Bessica-D SDK 的核心类与方法接口。


##  控制接口：`bessica_d_sdk.api.synria_b_robot_api.SynriaBessicaRobotAPI`

```python
from bessica_d_sdk.api import SynriaBessicaRobotAPI
from bessica_d_sdk.hardware import ServoDriver

robot = SynriaBessicaRobotAPI(ServoDriver(port=args.port, baudrate=args.baudrate, debug_mode=False))
```

### 主要方法一览：

#### 连接管理：
- `connect()`  
  连接机械臂并启动状态更新线程

- `disconnect()`  
  断开机械臂连接并停止更新线程

#### 运动控制：
- `set_home(arm="both", speed_factor=1.0)`  
  移动机械臂到初始位置（零位）
  - `arm`: `"left"`, `"right"` 或 `"both"`（默认）
  - `speed_factor`: 速度因子（默认 1.0）

- `set_joint_target(target_joints, target_joints_second=None, arm=None, joint_format="deg", wait=True, speed_factor=0.5)`  
  移动单臂或双臂到目标关节角度
  - `target_joints`: `List[float]`，长度为 7 的关节角度列表（单位：弧度或度）
  - `target_joints_second`: `Optional[List[float]]`，双臂模式下右臂的关节角度列表（默认 `None`）
  - `arm`: `"left"`, `"right"` 或 `"both"`（默认根据 `default_arm`）
  - `joint_format`: `"rad"` 或 `"deg"`（默认 `"deg"`）
  - `wait`: 是否等待运动完成（默认 `True`）
  - `speed_factor`: 速度因子（默认 0.5）
  - **注意**：当 `arm="both"` 时，需要提供 `target_joints_second` 参数

- `set_pose_target(target_pose, target_pose_second_arm=None, backend='numpy', method='dls', display=True, tolerance=1e-4, max_iters=100, multi_start=0, use_random_init=False, speed_factor=1.0, arm="both", execute=True)`  
  使用逆运动学移动末端执行器到目标位姿
  - `target_pose`: `List[float]`，目标位姿 `[x, y, z, qx, qy, qz, qw]`（单臂或左臂）
  - `target_pose_second_arm`: `Optional[List[float]]`，双臂模式下右臂的目标位姿（默认 `None`）
  - `arm`: `"left"`, `"right"` 或 `"both"`（默认 `"both"`）
  - **注意**：当 `arm="both"` 时，需要提供 `target_pose_second_arm` 参数，会对左右臂分别求解 IK
  - `backend`: `'numpy'` 或 `'torch'`（默认 `'numpy'`）
  - `method`: IK 求解方法 `'dls'`, `'pinv'` 或 `'transpose'`（默认 `'dls'`）
  - `display`: 是否打印求解细节（默认 `True`）
  - `tolerance`: 位置与姿态容差（默认 `1e-4`）
  - `max_iters`: 最大迭代次数（默认 `100`）
  - `multi_start`: 多起点尝试次数（默认 `0`）
  - `use_random_init`: 是否使用随机初值（默认 `False`）
  - `speed_factor`: 运动速度因子（默认 `1.0`）
  - `execute`: 是否执行得到的关节解（默认 `True`）
  - **返回**: 
    - 单臂模式：`Dict` 包含 `success`, `q`, `iters`, `pos_err`, `ori_err`, `message`, `motion_executed` 等字段
    - 双臂模式（`arm="both"`）：返回元组 `(ik_result_l, ik_result_r)`，每个元素为上述字典结构，分别对应左右臂的 IK 求解结果

#### 状态获取：
- `get_joints(arm=None)`  
  返回当前关节角度
  - `arm`: `"left"`, `"right"` 或 `both`（默认 `None`，返回 `"both"`）
  - **返回**: 
    - 单臂：`List[float]`（7 个关节角度，弧度）
    - 双臂：`List[List[float]]`（`[[left_7_joints], [right_7_joints]]`）

- `get_pose(arm=None)`  
  获取当前末端执行器位置与姿态
  - `arm`: `"left"`, `"right"` 或 `"both"`（默认 `None`，使用 `default_arm`）
  - **返回**: `Dict` 包含：
    - 单臂模式：`transform`, `position`, `rotation`, `euler_xyz`, `quaternion_xyzw`, `output_to_ik`
    - 双臂模式（`arm="both"`）：上述字段为列表格式，每个元素对应左右臂：
      - `transform`: `[T_fk_l, T_fk_r]`
      - `position`: `[position_l, position_r]`
      - `rotation`: `[rotation_l, rotation_r]`
      - `euler_xyz`: `[euler_l, euler_r]`
      - `quaternion_xyzw`: `[quat_l, quat_r]`
      - `output_to_ik`: `[output_to_ik_l, output_to_ik_r]`（可直接用于 `set_pose_target`）

- `get_gripper(arm=None)`  
  返回当前夹爪开合度
  - `arm`: `"left"`, `"right"`, `"both"` 或 `None`（默认 `None`，使用 `default_arm`）
  - **返回**: 
    - 单臂：`float`（0-100 度对应的弧度值）
    - 双臂：`Tuple[float, float]`（`(left_gripper, right_gripper)`）

- `print_state(arm="both", output_format="deg")`  
  打印当前机械臂信息
  - `arm`: `"left"`, `"right"` 或 `"both"`（默认 `"both"`）
  - `output_format`: `"deg"` 或 `"rad"`（默认 `"deg"`）

#### 夹爪控制：
- `set_gripper_target(arm, command=None, value=None, wait_for_completion=True, timeout=1.0, tolerance=0.1)`  
  控制夹爪位置
  - `arm`: `"left"`, `"right"` 或 `"both"`（必需，若为 `None` 则使用 `default_arm`）
  - `command`: `'open'` 或 `'close'`（与 `value` 二选一）
    - `'open'` 对应值为 `0.1` 度
    - `'close'` 对应值为 `99.9` 度
  - `value`: `float`（0-100 度）（与 `command` 二选一）
  - `wait_for_completion`: 是否等待完成（默认 `True`）
  - `timeout`: 超时时间（秒，默认 `1.0`）
  - `tolerance`: 误差容忍范围（弧度，默认 `0.1`）

#### 系统控制：
- `torque_control(command, arm='both')`  
  启用或关闭扭矩（'on' 或 'off'）
  - `command`: `'on'` 或 `'off'`
  - `arm`: `"left"`, `"right"` 或 `"both"`（默认 `"both"`）

- `set_zero(arm='both')`  
  执行归零校准流程：交互式提示 → 关闭扭矩 → 手动拖动 → 重启扭矩 → 记录零点
  - `arm`: `"left"`, `"right"` 或 `"both"`（默认 `"both"`）

---

##  硬件层接口：`bessica_d_sdk.hardware.ServoDriver`

提供底层串口通信、数据解析和电机控制功能。

主要方法包括：
- `connect()` / `disconnect()`
- `read_joint_angles(arm='both')` / `set_joint_angles(joint_angles, arm, ...)`
- `read_joint_state(arm='both')` / `read_gripper_data(arm='both')`
- `set_gripper(angle_rad, arm, ...)`
- `enable_torque(arm)` / `disable_torque(arm)`
- `set_zero_position(arm)`

不推荐用户直接使用此类，建议通过 `SynriaBessicaRobotAPI` 高级接口操作。

---

##  工具函数：`bessica_d_sdk.utils.control`

该模块提供一组高级控制函数，用于简化常用操作。

### 关节控制函数：

- `move_joint(controller, joint_id, angle_deg, arm, interpolate=True)`  
  控制单个关节到指定角度（度）

- `move_joints(controller, angles_deg, arm, interpolate=True)`  
  设置单臂全部 7 个关节角度（度）

- `move_joints_dual_arm(controller, left_angles_deg, right_angles_deg, interpolate=True)`  
  控制双臂全部 14 个关节（度）

- `move_to_zero(controller, arm, interpolate=True)`  
  将指定机械臂移动到零位

### 夹爪控制函数：

- `set_gripper_angle(controller, angle_deg, arm="left", wait=True)`  
  设置单臂夹爪角度（0~100 度）

- `set_dual_gripper(controller, left_deg, right_deg, wait=True)`  
  分别设置左右臂夹爪角度

- `open_gripper(controller, angle_deg=100.0, arm="both", wait=True)`  
  将夹爪张开到指定角度（默认最大 100 度）

- `close_gripper(controller, arm="both", wait=True)`  
  关闭夹爪（设为 0 度）

### 状态读取与打印：

- `print_joint_angles(controller, arm="both", read_gripper=True)`  
  以角度形式（单位：度）打印当前关节和夹爪角度

- `print_gripper_angles(controller, arm="both")`  
  单独打印夹爪角度（单位：度）

---

##  RoboCore 集成

SDK 集成了 [RoboCore](https://github.com/Synria-Robotics/RoboCore) 库，提供高性能运动学和轨迹规划功能：

### 运动学功能（来自 robocore.kinematics）：
- `forward_kinematics(robot_model, q, backend='numpy', return_end=True)`
- `inverse_kinematics(robot_model, pose, q_init, backend='numpy', method='dls', ...)`
- `jacobian(robot_model, q, backend='numpy', method='analytic')`

<!-- ### 轨迹规划功能（来自 robocore.planning）：
- `cubic_polynomial_trajectory(q_start, q_end, duration, num_points)`
- `quintic_polynomial_trajectory(q_start, q_end, duration, num_points)`
- `linear_joint_trajectory(q_start, q_end, duration, num_points)`
- `linear_cartesian_trajectory(robot_model, pose_start, pose_end, duration, ...)`
- `trapezoidal_velocity_profile(distance, max_vel, max_acc)` -->

---

## 注意事项

### 双臂系统特点：
- Bessica-D 是双臂系统，支持 `"left"`, `"right"` 和 `"both"` 三种模式
- 单臂控制时需明确指定 `arm` 参数
- 关节角度为 7 个（而非单臂系统的 6 个）

---

##  示例程序（Examples）

SDK 提供了多个示例程序，位于 `examples/` 目录下，展示了如何使用各种功能：

### 基础连接与状态读取：

- **`00_demo_test_connect.py`**  
  测试机械臂连接功能
  ```bash
  python examples/00_demo_test_connect.py --port /dev/ttyACM0
  ```

- **`03_demo_read_states.py`**  
  读取并打印机械臂状态（关节角度、位姿、夹爪）
  ```bash
  python examples/03_demo_read_states.py --arm left
  ```
  - 支持 `--arm` 参数：`left`、`right` 或 `both`

### 运动控制：

- **`05_demo_move_joint.py`**  
  控制关节运动示例
  ```bash
  python examples/05_demo_move_joint.py --arm left
  ```
  - 演示单臂/双臂关节角度控制
  - 展示 `set_joint_target()` 使用方法

- **`04_demo_move_gripper.py`**  
  控制夹爪开合示例
  - 演示单臂/双臂夹爪控制

### 运动学：

- **`07_demo_forward_kinematics.py`**  
  正运动学计算示例
  ```bash
  python examples/07_demo_forward_kinematics.py --arm left
  ```
  - 从关节角度计算末端执行器位姿
  - 显示位置、旋转矩阵、欧拉角、四元数

- **`08_demo_inverse_kinematics.py`**  
  逆运动学控制示例
  ```bash
  python examples/08_demo_inverse_kinematics.py --arm left
  ```
  - 通过目标位姿求解关节角度并执行运动
  - 演示 `set_pose_target()` 的完整流程

### 系统功能：

- **`02_demo_zero_calibration.py`**  
  零点校准程序
  ```bash
  python examples/02_demo_zero_calibration.py
  ```
  - 交互式归零流程
  - **警告**：执行前确保机械臂周围无障碍物

- **`01_torque_switch.py`**  
  扭矩开关控制示例

### 高级功能：


- **`06_demo_mujoco.py`**  
  MuJoCo 仿真相关示例


### 运行示例程序：

所有示例程序都支持命令行参数：

```bash
# 基本参数
--port <串口路径>      # 例如: /dev/ttyACM0 或 COM3（留空自动查找）
--baudrate <波特率>    # 默认: 1000000

# 部分示例支持
--arm <left|right|both>  # 指定控制的机械臂
```



如需更多细节，请参考源码文档或查看日志文件输出。
