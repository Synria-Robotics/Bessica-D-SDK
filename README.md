# Bessica-D-SDK  

Bessica-D-SDK 是一个用于控制 【灵越 Bessica-D】 系列七轴机械双臂（带夹爪）的 Python 工具包。它提供了通过串口与机械臂通信、控制关节运动、操作夹爪以及读取状态信息的功能。

SDK 集成了 [RoboCore](https://github.com/Synria-Robotics/RoboCore) 库，提供高性能运动学和轨迹规划功能。

---

## 主要特性

*   **关节控制**: 设置和读取七个关节的角度，支持单臂和双臂控制。
*   **夹爪控制**: 控制夹爪的开合角度。
*   **力矩控制**: 启用或禁用机械臂关节力矩，允许自由拖动或锁定位置。
*   **零点设置**: 将机械臂当前位置设置为新的零点。
*   **状态读取**: 获取关节角度、夹爪角度和按钮状态的实时数据。
*   **运动学**: 基于 RoboCore 的正逆运动学计算。
*   **轨迹规划**: 基于 RoboCore 的轨迹规划功能。
*   **串口通信**: 自动查找或指定串口进行连接。

---

## 目录结构

```
Bessica-D-SDK/
├── bessica_d_sdk/                # 核心 SDK 模块
│   ├── __init__.py
│   ├── controller.py             # 主控制类 ArmController
│   ├── api/                      # 高级 API 接口
│   │   ├── __init__.py
│   │   └── synria_b_robot_api.py
│   ├── hardware/                 # 硬件层
│   │   ├── __init__.py
│   │   ├── servo_driver.py       # 底层驱动
│   │   ├── serial_comm.py        # 串口通信
│   │   └── data_parser.py        # 数据解析
│   └── utils/                    # 控制工具函数
│       ├── __init__.py
│       └── control.py
│
├── examples/                     # 示例脚本
│   ├── 00_demo_test_connect.py          # 连接测试
│   ├── 01_torque_switch.py              # 扭矩开关
│   ├── 02_demo_zero_calibration.py      # 零点校准
│   ├── 03_demo_read_states.py           # 读取状态
│   ├── 04_demo_move_gripper.py          # 夹爪控制
│   ├── 05_demo_move_joint.py            # 关节运动
│   ├── 06_demo_mujoco.py                # MuJoCo 仿真
│   ├── 07_demo_forward_kinematics.py    # 正运动学
│   ├── 08_demo_inverse_kinematics.py   # 逆运动学
│   ├── 09_demo_drage_teaching.py        # 拖拽示教
│   └── 10_demo_mujoco_real_robot_bridge.py      # mujoco控制真机
│
├── docs/                         # 文档
│   ├── api_reference.md          # API 参考
│   └── ...
│
├── requirements.txt             # 项目依赖
├── README.md                    # 项目简介
├── setup.py                     # 安装脚本
└── .gitignore                   # 忽略 pycache、log、*.pyc
```

---

## 快速开始

1.  **安装**: 请参照安装指南进行安装和配置。
2.  **运行示例**:
    进入 `examples` 目录，尝试运行一个示例脚本：

    ```sh
    # 连接测试
    python examples/00_demo_test_connect.py --port /dev/ttyACM0
    
    # 关节运动控制
    python examples/05_demo_move_joint.py --port /dev/ttyACM0
    
    # 逆运动学控制
    python examples/08_demo_inverse_kinematics.py --port /dev/ttyACM0
    

    ```

3.  **基本使用**:
    ```python
    from bessica_d_sdk.api import SynriaBessicaRobotAPI
    from bessica_d_sdk.hardware import ServoDriver
    
    # 初始化并连接
    robot = SynriaBessicaRobotAPI(ServoDriver(port="", baudrate=1000000))
    robot.connect()
    
    # 控制左臂到目标关节角度（度）
    robot.set_joint_target(
        target_joints=[10, 20, 20, 10, 45, 20, 0],
        arm="left_arm",
        joint_format="deg"
    )
    
    # 断开连接
    robot.disconnect()
    ```

---

## 文档

*   [API 参考](docs/api_reference.md) - 详细的 API 接口说明和示例

---

## 示例脚本列表

| 示例脚本 | 功能说明 |
|----------|----------|
| `00_demo_test_connect.py` | 测试机械臂连接 |
| `01_torque_switch.py` | 扭矩开关控制 |
| `02_demo_zero_calibration.py` | 手动拖拽设置零点 |
| `03_demo_read_states.py` | 读取关节和夹爪状态 |
| `04_demo_move_gripper.py` | 控制夹爪张合及角度设置 |
| `05_demo_move_joint.py` | 控制单臂/双臂关节运动 |
| `06_demo_mujoco.py` | MuJoCo 仿真相关示例 |
| `07_demo_forward_kinematics.py` | 正运动学计算示例 |
| `08_demo_inverse_kinematics.py` | 逆运动学控制示例 |
| `09_demo_drage_teaching.py` | 拖拽示教模式 |
| `demo_interactivate_mode.py` | 综合交互式测试菜单 |

---

## 依赖

主要依赖包括：
- `pyserial` - 串口通信
- `numpy` - 数值计算
- `robocore` - 运动学和轨迹规划

完整依赖列表请参见 `requirements.txt`。

---

## 许可证

GPL v3.0

---

## 贡献

欢迎提交 Issue 和 Pull Request。
