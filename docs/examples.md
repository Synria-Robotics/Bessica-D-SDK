# 示例代码说明

本目录包含使用 Bessica-D SDK 与机械臂交互的演示脚本，涵盖基本控制、状态读取、夹爪操作和示教功能。

## 如何运行示例

1.  确保您已完成 [安装指南](installation.md) 中的所有步骤。
2.  打开终端或命令提示符。
3.  进入 `examples` 目录：
    ```bash
    cd examples
    ```
4.  运行某个示例：
    ```bash
    python3 demo_read_state.py
    ```

---

## 示例脚本列表

| 脚本文件 | 功能说明 |
|----------|----------|
| `demo_read_state.py` | 读取指定机械臂当前的关节角度和夹爪角度，支持持续输出 |
| `demo_single_arm.py` | 控制单臂的七个关节：设置角度、归零、单关节操作 |
| `demo_dual_arm.py` | 同步控制双臂的全部关节角度，可分别控制左右臂某个关节 |
| `demo_gripper.py` | 控制夹爪开合角度，支持单/双臂夹爪设置和交互式输入 |
| `demo_zero_calibration.py` | 关闭扭矩后手动拖拽，将当前位置设置为零点 |
| `demo_interactivate_mode.py` | 综合控制界面，支持关节操作、夹爪控制、示教往复、扭矩开关等交互模式 |

---

## 推荐阅读顺序

1. `demo_read_state.py`：了解状态读取机制  
2. `demo_single_arm.py`：学习单臂控制流程  
3. `demo_dual_arm.py`：扩展到双臂控制  
4. `demo_gripper.py`：掌握夹爪使用方式  
5. `demo_zero_calibration.py`：了解零点设置方法  
6. `demo_interactivate_mode.py`：实际部署或测试前的综合调试入口

---

## 注意事项

- 所有脚本均使用 `ArmController` 类作为控制入口。
- 请确保运行前机械臂已正确连接并上电。
- 在操作夹爪和归零前，请注意保持机械臂物理安全。

