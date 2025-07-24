# API 参考

本文档提供了 Bessica-D SDK 中主要类和方法的详细参考。

## 核心模块

SDK 的核心功能主要由以下模块提供：

*   [`bessica_d_sdk.controller`](../bessica_d_sdk/controller.py): 包含主要的 `ArmController` 类，用于与机械臂交互。
*   [`bessica_d_sdk.data_parser`](../bessica_d_sdk/data_parser.py): 包含 `DataParser` 类和 `JointState` 命名元组，用于解析和存储来自机械臂的数据。
*   [`bessica_d_sdk.serial_comm`](../bessica_d_sdk/serial_comm.py): 包含 `SerialComm` 类，处理底层串口通信。

对于大多数用户而言，主要交互将通过 `ArmController` 类进行。

## `bessica_d_sdk.controller.ArmController`

此类是控制 Bessica-D 机械臂的主要接口。

```python
from bessica_d_sdk.controller import ArmController
```

### 常量

*   `ArmController.DEG_TO_RAD`: `float`
    将角度从度转换为弧度的系数 (`math.pi / 180.0`)。
*   `ArmController.RAD_TO_DEG`: `float`
    将角度从弧度转换为度的系数 (`180.0 / math.pi`)。

### 初始化

*   `__init__(self, port: str = "", baudrate: int = 921600, debug_mode: bool = False)`
    初始化机械臂控制器。
    *   **参数**:
        *   `port` (`str`, 可选): 串口名称 (例如, Linux 上的 `"/dev/ttyUSB0"` 或 Windows 上的 `"COM3"`)。如果留空，SDK 将尝试自动搜索可用串口。默认为 `""`。
        *   `baudrate` (`int`, 可选): 串口通信的波特率。默认为 `921600`。
        *   `debug_mode` (`bool`, 可选): 是否启用调试模式。启用后，将输出更详细的日志信息，包括发送和接收的数据帧。默认为 `False`。

### 连接与断开

*   `connect(self) -> bool`
    连接到机械臂。如果 `port` 未在初始化时指定，则会尝试自动查找。
    *   **返回**: `bool` - 如果连接成功则为 `True`，否则为 `False`。

*   `disconnect(self)`
    断开与机械臂的连接并关闭串口。

### 读取数据

*   `read_joint_angles(self) -> Optional[List[float]]`
    读取机械臂六个关节的当前角度。
    此方法会尝试读取新的数据帧，如果成功且为关节数据，则解析并返回最新角度。如果未能读取到新的关节数据，则返回上一次已知的关节角度。
    *   **返回**: `Optional[List[float]]` - 包含六个关节角度（单位：弧度）的列表。如果无法获取状态（例如，在初始连接且未收到任何数据之前），理论上可能返回基于内部默认值的状态，但通常在连接后很快就会有实际数据。

*   `read_gripper_data(self, arm: str = 'both') -> Union[float, Tuple[float, float]]`
    读取夹爪的当前角度
    此方法会尝试读取新的数据帧，如果成功且为夹爪数据，则解析并返回最新状态。如果未能读取到新的夹爪数据，则返回上一次已知的夹爪状态。
    *   **返回**: `Union[float, Tuple[float, float]]` - 单臂夹爪的弧度数据或者双臂夹爪弧度元组：
        *   单臂：夹爪角度
        *   双臂：（左臂夹爪角度， 右臂夹爪角度）

*   `read_joint_state(self, arm: str = 'both') -> Optional[Union[JointState, JointStateDict]]`
    读取完整的机械臂状态，包括双臂所有关节角度以及夹爪角度。
    此方法会尝试读取并解析最新的数据帧（无论是关节数据还是夹爪数据），并更新内部状态。
    *   **返回**: [`[Union[JointState, JointStateDict]]`](#bessica_d_sdkdata_parserjointstate) - 一个包含指定机械臂完整状态的对象, 或者包含双臂完整状态的dict。

### 设置与控制

*   `set_joint_angles(self, joint_angles: List[float], gripper_angle: float = None, wait_for_completion: bool = True, timeout: float = 10.0, tolerance: float = 0.08) -> bool`
    设置机械臂六个关节的目标角度。
    *   **参数**:
        *   `joint_angles` (`List[float]`): 包含七个目标关节角度（单位：弧度）的列表。列表长度必须为6。
        *   `gripper_angle` (`float`, 可选): 夹爪的目标角度（单位：弧度）。如果提供此参数，则在设置关节角度后会接着发送夹爪控制命令。默认为 `None` (不控制夹爪)。
        *   `wait_for_completion` (`bool`, 可选): 是否等待运动完成后再返回。默认为 `True`。
        *   `timeout` (`float`, 可选): 等待运动完成的最大时间（单位：秒）。默认为 `5.0`。
        *   `tolerance` (`float`, 可选): 判断运动是否完成的角度误差容忍度（单位：弧度）。默认为 `0.08`。
    *   **返回**: `bool` - 如果命令成功发送并执行（如果等待完成）则为 `True`，否则为 `False`。

*   `set_gripper(self, angle_rad: float) -> bool`
    设置夹爪的开合角度。
    *   **参数**:
        *   `angle_rad` (`float`): 夹爪的目标角度（单位：弧度）。通常 0 表示完全张开，某个正值（例如 `100 * DEG_TO_RAD`）表示完全闭合，具体范围取决于夹爪硬件。
    *   **返回**: `bool` - 如果命令成功发送则为 `True`，否则为 `False`。

*   `set_zero_position(self) -> bool`
    将机械臂当前的姿态设置为新的零点位置。
    *   **返回**: `bool` - 如果命令成功发送则为 `True`，否则为 `False`。

*   `enable_torque(self) -> bool`
    使能所有关节的力矩。机械臂将尝试保持当前位置，抵抗外力。
    *   **返回**: `bool` - 如果命令成功发送则为 `True`，否则为 `False`。

*   `disable_torque(self) -> bool`
    禁用所有关节的力矩。机械臂关节将可以被自由拖动。
    *   **返回**: `bool` - 如果命令成功发送则为 `True`，否则为 `False`。


```python
from bessica_d_sdk.data_parser import JointState
```

## `bessica_d_sdk.data_parser.JointStateDict`

```python
from bessica_d_sdk.data_parser import JointStateDict
```

`JointStateDict` 是一个字典类型，结构为 `Dict[str, JointState]`，用于同时存储左右机械臂的状态。

### 示例结构：

```python
{
    "left_arm": JointState(angles=[...], gripper=..., timestamp=...),
    "right_arm": JointState(angles=[...], gripper=..., timestamp=...)
}
```

### 特点：

* `left_arm` 与 `right_arm` 是键，对应左右臂
* 每个值都是一个 `JointState` 实例
* 通过 `controller.read_joint_state(arm="both")` 获取该结构
* 常用于双臂同步操作、状态监测等场景


### 属性

*   `angles`: `List[float]`
    一个包含六个关节角度（单位：弧度）的列表。
*   `gripper`: `float`
    夹爪的当前角度（单位：弧度）。
*   `timestamp`: `float`
    状态数据最后更新的时间戳（`time.time()` 的结果，单位：秒）。


## 内部模块 (简述)

### `bessica_d_sdk.data_parser.DataParser`

*   此类负责解析从串口接收到的原始字节数据帧，将其转换为结构化的信息，如关节角度、夹爪状态等，并更新 `JointState`。
*   主要方法:
    *   `parse_frame(self, frame: List[int]) -> Optional[Dict]`: 解析单个数据帧。
    *   `get_joint_state(self) -> JointState`: 获取当前解析的最新状态。

### `bessica_d_sdk.serial_comm.SerialComm`

*   此类封装了与串口设备进行通信的底层逻辑。
*   主要方法:
    *   `connect(self) -> bool`: 打开并配置串口连接。
    *   `disconnect(self)`: 关闭串口连接。
    *   `send_data(self, data: List[int]) -> bool`: 将字节列表发送到串口。
    *   `read_frame(self) -> Optional[List[int]]`: 从串口读取一个完整的数据帧。
    *   `find_serial_port(self) -> str`: 自动查找可用的串口设备。

用户通常不需要直接与 `DataParser` 或 `SerialComm` 类交互，因为 `ArmController` 已经处理了这些细节。

## `bessica_d_sdk.utils.control`

该模块提供一组高级控制函数，用于简化常用操作，如移动关节、控制夹爪、打印状态等。

```python
from bessica_d_sdk.utils import move_joint, move_joints, open_gripper, ...
```

### 关节控制函数

#### `move_joint(controller, joint_id, angle_deg, arm, interpolate=True)`
控制单个关节到指定角度。

- `controller`: `ArmController` 实例  
- `joint_id`: 目标关节索引（0-6）  
- `angle_deg`: 目标角度（单位：度）  
- `arm`: `"left_arm"` 或 `"right_arm"`  
- `interpolate`: 是否插值移动（默认 True）  
- **返回**: `bool`

#### `move_joints(controller, angles_deg, arm, interpolate=True)`
设置单臂全部 7 个关节角度。

- `angles_deg`: `List[float]`，长度为 7，单位：度  
- 其他参数同上  
- **返回**: `bool`

#### `move_joint_dual_arm(controller, left_joint_id, right_joint_id, angles_left_deg, angles_right_deg, interpolate=True)`
分别控制左右臂的某个关节角度。

- `angles_left_deg`, `angles_right_deg`: 单个角度（度）
- **返回**: `bool`

#### `move_joints_dual_arm(controller, left_angles_deg, right_angles_deg, interpolate=True)`
控制双臂 14 个关节。

- `left_angles_deg`, `right_angles_deg`: 长度为 7 的角度列表
- **返回**: `bool`

#### `move_to_zero(controller, arm, interpolate=True)`
将指定机械臂移动到零位（全 0 角度）。

---

### 夹爪控制函数

#### `set_gripper_angle(controller, angle_deg, arm="left_arm", wait=True)`
设置单臂夹爪角度（0~100 度）

#### `set_dual_gripper(controller, left_deg, right_deg, wait=True)`
分别设置左右臂夹爪角度。

#### `open_gripper(controller, angle_deg=100.0, arm="both", wait=True)`
将夹爪张开到指定角度（默认最大 100 度）

#### `close_gripper(controller, arm="both", wait=True)`
关闭夹爪（设为 0 度）

---

### 状态读取与打印

#### `print_joint_angles(controller, arm="both", read_gripper=True)`
以角度形式（单位：度）打印当前关节和夹爪角度。

#### `print_gripper_angles(controller, arm="both")`
单独打印夹爪角度（单位：度）
