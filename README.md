# Bessica-D-SDK  

Bessica-D-SDK 是一个用于控制 【灵越 Bessica-D】 系列七轴机械双臂（带夹爪）的 Python 工具包。它提供了通过串口与机械臂通信、控制关节运动、操作夹爪以及读取状态信息的功能。

---
## 主要特性

*   **关节控制**: 设置和读取七个关节的角度。
*   **夹爪控制**: 控制夹爪的开合角度。
*   **力矩控制**: 启用或禁用机械臂关节力矩，允许自由拖动或锁定位置。
*   **零点设置**: 将机械臂当前位置设置为新的零点。
*   **状态读取**: 获取关节角度、夹爪角度和按钮状态的实时数据。
*   **串口通信**: 自动查找或指定串口进行连接。
*   **数据解析**: 解析来自机械臂的反馈数据帧。
---
## 目录结构

```
Bessica-D-SDK/
├── bessica_d_sdk/                # 核心 SDK 模块
│   ├── __init__.py
│   ├── controller.py             # 主控制类 ArmController
│   ├── data_parser.py            # 数据解析
│   ├── serial_comm.py            # 串口通信
│   └── utils/                    # 控制工具函数
│       ├── __init__.py
│       └── control.py
│
├── examples/                     # 示例脚本
│   ├── demo_dual_arm.py
│   ├── demo_gripper.py
│   ├── demo_interactivate_mode.py
│   ├── demo_read_state.py
│   ├── demo_single_arm.py
│   └── demo_zero_calibration.py
│
├── docs/                         # 文档
│   ├── api_reference.md
│   ├── examples.md
│   └── installation.md
│
├── requirements.txt             # 项目依赖
├── README.md                    # 项目简介
├── setup.py                     # 安装脚本
└── .gitignore                   # 忽略 pycache、log、*.pyc

```
---
## 快速开始

1.  **安装**: 请参照 [docs/installation.md](docs/installation.md) 进行安装和配置。
2.  **运行示例**:
    进入 `examples` 目录，尝试运行一个示例脚本，例如读取机械臂角度：
    ```sh
    cd examples
    python3 demo_read_state.py
    ```
    或者控制单臂运动：
    ```sh
    python3 demo_single_arm.py
    ```

---
## 文档

*   [安装指南](docs/installation.md)
*   [示例说明](docs/examples.md)
*   [API 参考](docs/api_reference.md)

---
## 示例脚本列表

| 示例脚本 | 功能说明 |
|----------|----------|
| `demo_read_state.py` | 持续读取关节和夹爪状态 |
| `demo_single_arm.py` | 控制单臂关节和归零动作 |
| `demo_dual_arm.py` | 同步控制双臂角度 |
| `demo_gripper.py` | 控制夹爪张合及角度设置 |
| `demo_zero_calibration.py` | 手动拖拽设置当前为零点 |
| `demo_interactivate_mode.py` | 综合交互式测试菜单与教学模式 |
