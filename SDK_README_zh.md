# rebot_control

reBot B601-RS 六轴机械臂 MIT 位置控制程序（Python），基于 MotorBridge 通过 CAN 总线控制 RobStride 电机。

## 警告

> **本项目仅提供机械臂控制接口，不包含软件限位功能。** 不会对关节角度或工作空间做软件层面的限制与校验；你设置的目标角度会直接下发给电机。
>
> **使用时请将机械臂限制在约 70% 工作空间内运行**，避免靠近关节极限或奇异位形。超出机械限位可能导致碰撞、损坏或失控；请在上层应用自行实现限位，或严格控制目标角度与运动范围。

## 硬件配置

- 机械臂：reBot B601-RS（6 关节 + 末端夹爪）
- 电机：J1-J3 为 RS06，J4-J6 为 RS00，夹爪 CAN ID **7**（RS00）
- 通信：CAN 总线（默认 `can0`，主机 ID `0xFD`）

## 代码结构

```
rebot_control/
├── config/
│   └── rebotarm_rs.yaml            # 配置文件：CAN / 电机 / 温度阈值 / 回零参数
├── rebot/                      # 核心包
│   ├── __init__.py             # 对外导出公共 API
│   ├── config.py               # 配置层：读取 YAML，dataclass 定义与校验
│   └── controller.py           # 接口层：ReBotRSMITController（纯控制逻辑）
└── examples/                   # 示例与教程脚本
    ├── _bootstrap.py           # 路径引导（供示例直接运行）
    ├── mit_position_control.py # 完整演示：可编辑目标角度与速度
    ├── quick_start.py          # 教程 1：快速开始
    ├── custom_config.py        # 教程 2：指定配置文件
    ├── monitor_status.py       # 教程 3：运行中监控状态
    ├── single_joint_adjust.py  # 教程 4：单关节微调
    ├── read_joint_angles.py    # 教程 5：读取实际位置
    ├── stop_options.py         # 教程 6：安全停止选项
    └── recommended_structure.py # 教程 7：推荐程序结构
```

分层说明：

- **配置文件**（`config/rebotarm_rs.yaml`）：所有可调参数的单一来源，改参数只需编辑 YAML，不碰代码。
- **配置层**（`rebot/config.py`）：`load_config()` 读取 YAML 并构造 `ControllerConfig` 等 dataclass，做合法性校验（阈值顺序、频率、未知键笔误等）。
- **接口层**（`rebot/controller.py`）：`ReBotRSMITController`，只依赖 `ControllerConfig`，不含任何硬编码数值和演示参数。
- **示例层**（`examples/`）：可运行的教程脚本与完整演示，对应 README「调用教程」各节。

## 功能特性

- MIT 模式持续发送位置指令（默认 200 Hz）
- 七电机（J1–J6 机械臂 + CAN ID 7 夹爪），各关节独立速度限制（默认 20°/s，控制循环内平滑）
- 实时监控各电机 MOS 温度（默认 2 Hz）
- 三级温度保护：
  - ≥ 80°C：温度报警，继续运行
  - ≥ 100°C：停止运动，缓慢回零后失能
  - ≥ 140°C：立即紧急失能，不再回零
- 安全退出：按 Esc、Ctrl+C 或调用 `arm.stop()`，机械臂以 smoothstep 轨迹缓慢回零后失能
- 回零过程中再次按 Ctrl+C，立即中止回零并失能
- 启动时先读取实际位置作为初始指令，避免使能后突然跳动

## 依赖安装

```bash
pip install motorbridge pynput pyyaml
```

`pynput` 用于 Esc 键监听，未安装时仅 Esc 功能失效，Ctrl+C 和 `stop()` 仍然有效。

## CAN 接口准备

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
```

RobStride 电机默认波特率为 1 Mbps，请按实际配置调整。

## 使用方法

1. 编辑 `config/rebotarm_rs.yaml` 调整 CAN 接口、温度阈值、电机参数等。
2. 修改 `examples/mit_position_control.py` 顶部的 `TARGET_ANGLES` 和 `JOINT_SPEEDS_DEG_S`。
3. 运行：

```bash
python3 examples/mit_position_control.py
```

### 配置文件（`config/rebotarm_rs.yaml`）

| 配置 | 说明 | 默认值 |
|---|---|---|
| `can.channel` | CAN 接口名 | `can0` |
| `can.host_id` | 主机 ID | `0xFD` |
| `control.control_hz` | MIT 指令发送频率（非运动速度） | 200 Hz |
| `control.telemetry_hz` | 温度读取频率 | 2 Hz |
| `temperatures.alarm_c` | 温度报警阈值 | 80°C |
| `temperatures.return_zero_c` | 高温回零阈值 | 100°C |
| `temperatures.disconnect_c` | 紧急失能阈值 | 140°C |
| `return_zero.max_speed_deg_s` | 普通回零峰值速度 | 30°/s |
| `return_zero.thermal_max_speed_deg_s` | 高温触发的回零峰值速度 | 30°/s |
| `return_zero.min_time_s` | 最短回零时间 | 3.0 s |
| `motors` | 各电机 ID / 型号 / MIT `kp` / `kd` 列表 | 见 YAML 注释 |

YAML 中未填写的项自动使用代码内默认值；填了不存在的键会直接报错，防止笔误。

## 开放接口说明

`rebot` 包通过 `rebot/__init__.py` 导出以下公共 API：

| 名称 | 类型 | 说明 |
|---|---|---|
| `ReBotRSMITController` | 类 | 机械臂 MIT 控制器（主接口） |
| `load_config` | 函数 | 从 YAML 加载 `ControllerConfig` |
| `ControllerConfig` | dataclass | 控制器完整配置 |
| `MotorConfig` | dataclass | 单个电机配置（ID、型号、kp、kd） |
| `TemperatureThresholds` | dataclass | 三级温度阈值 |
| `ReturnZeroConfig` | dataclass | 回零轨迹参数 |
| `DEFAULT_CONFIG_PATH` | `Path` | 默认配置文件路径 |
| `DEFAULT_MOTORS` | tuple | B601-RS 默认电机列表（含夹爪 ID 7） |
| `GRIPPER_MOTOR_ID` | 常量 | 夹爪 CAN 电机 ID（7） |
| `GRIPPER_JOINT_ID` | 常量 | 夹爪在 API 中的关节序号（7，即 `set_joint_angle(7, …)`） |

### 控制器生命周期

典型调用顺序如下（**必须先 `start()` 再发运动指令**）：

```
创建实例 → start() → set_max_speeds() → set_joint_angles() → … → stop()
```

| 阶段 | 方法 | 说明 |
|---|---|---|
| 创建 | `ReBotRSMITController(config=None)` | 不传 `config` 时自动加载 `config/rebotarm_rs.yaml` |
| 连接 | `connect()` | 打开 CAN、注册电机、切 MIT、使能；`start()` 内部会自动调用 |
| 启动 | `start(enable_esc=True, install_signal_handlers=True)` | 启动控制线程与温度监控；可选 Esc 监听与 SIGINT/SIGTERM 处理 |
| 运动 | `set_max_speeds()` / `set_joint_angles()` / `set_joint_angle()` | 设置速度与目标角度（度） |
| 查询 | `get_target_angles()` / `get_command_angles()` / `read_joint_angles()` | 读目标、平滑指令、实际位置（度） |
| 停止 | `stop()` 或 `request_stop()` | 默认缓慢回零后失能并关闭 CAN |

### `ReBotRSMITController` API 参考

#### 构造与连接

| 方法 | 参数 | 返回值 | 说明 |
|---|---|---|---|
| `__init__(config=None)` | `config`: `ControllerConfig \| None` | — | 创建控制器；`config=None` 时加载默认 YAML |
| `connect()` | — | `None` | 连接 CAN、切 MIT、使能后读当前角度；重复调用无副作用 |
| `disable_motors()` | — | `None` | 失能所有电机，不关闭 CAN（便于手动推动后读位置） |
| `start(...)` | `enable_esc`: 是否监听 Esc；`install_signal_handlers`: 是否注册 Ctrl+C / SIGTERM | `None` | 启动后台线程；重复调用无副作用 |

#### 运动控制（单位：度、度/秒）

| 方法 | 参数 | 返回值 | 说明 |
|---|---|---|---|
| `set_joint_angles(angles_deg)` | 长度为 7 的序列，J1–J6 + 夹爪 J7 | `None` | 设置全部关节与夹爪目标 |
| `set_joint_angle(joint_id, angle_deg)` | `joint_id`: 1–7；J7 为夹爪 | `None` | 只改单个关节/夹爪目标 |
| `set_max_speeds(speeds_deg_s)` | 长度为 7 的序列 | `None` | 各关节/夹爪最大运动速度 |

#### 状态读取

| 方法 / 属性 | 返回值 | 说明 |
|---|---|---|
| `get_target_angles()` | `list[float]` | 用户设置的最终目标角度（度） |
| `get_command_angles()` | `list[float]` | 经速度限制后实际下发的平滑角度（度） |
| `read_joint_angles()` | `list[float]` | 同步读取电机实际机械位置（度），会占用 CAN |
| `last_temperatures` | `list[float \| None]` | 最近一次遥测的各电机 MOS 温度（°C）；`None` 表示未读到 |
| `last_error` | `Exception \| None` | 控制线程通信异常时记录的错误 |
| `is_stopped` | `bool` | 安全停止流程是否已完成 |
| `config` | `ControllerConfig` | 当前使用的配置对象 |

#### 停止与安全退出

| 方法 | 参数 | 说明 |
|---|---|---|
| `stop(return_to_zero=True, wait=True)` | `return_to_zero`: 是否先回零；`wait`: 是否阻塞到停止完成 | 最常用的停止接口 |
| `request_stop(...)` | `reason`, `return_to_zero`, `thermal`, `emergency`, `wait` | 高级停止；`emergency=True` 时立即失能、不回零 |
| `wait_until_stopped(timeout=None)` | `timeout`: 最长等待秒数 | 阻塞等待停止完成；返回是否在超时前完成 |

### 配置 API

```python
from rebot import load_config, ControllerConfig, DEFAULT_CONFIG_PATH

# 默认配置文件
config = load_config()

# 指定路径
config = load_config("config/rebotarm_rs.yaml")

# 默认路径常量
print(DEFAULT_CONFIG_PATH)
```

`ControllerConfig.from_yaml(path)` 也可直接从 YAML 构造，效果与 `load_config(path)` 相同。

## 调用教程

以下示例均可从项目根目录直接运行。完整代码见 `examples/` 目录。

每个示例文件顶部有 **Expected motion / 预期动作** 说明，运行时会打印 `[Expected / 预期]` 一行，便于对照观察机械臂实际运动。

| 教程 | 示例文件 | 运行命令 |
|---|---|---|
| 1. 快速开始 | `examples/quick_start.py` | `python3 examples/quick_start.py` |
| 2. 指定配置文件 | `examples/custom_config.py` | `python3 examples/custom_config.py` |
| 3. 运行中监控状态 | `examples/monitor_status.py` | `python3 examples/monitor_status.py` |
| 4. 单关节微调 | `examples/single_joint_adjust.py` | `python3 examples/single_joint_adjust.py` |
| 5. 读取实际位置 | `examples/read_joint_angles.py` | `python3 examples/read_joint_angles.py` |
| 6. 安全停止选项 | `examples/stop_options.py` | `python3 examples/stop_options.py default` |
| 7. 推荐程序结构 | `examples/recommended_structure.py` | `python3 examples/recommended_structure.py` |
| 完整参数演示 | `examples/mit_position_control.py` | `python3 examples/mit_position_control.py` |

### 1. 快速开始（最小示例）

见 `examples/quick_start.py`：创建控制器 → `start()` → 设速度与目标角度 → **等待运动到位** → `stop()`。

> `set_joint_angles()` 只更新目标角度，机械臂按 `set_max_speeds()` 限速逐渐逼近；若立刻 `stop()`，可能几乎看不到向目标的运动。

### 2. 指定配置文件

见 `examples/custom_config.py`：通过 `load_config("config/rebotarm_rs.yaml")` 传入 `ReBotRSMITController`。

也可在命令行指定其他 YAML：

```bash
python3 examples/custom_config.py config/rebotarm_rs.yaml
```

### 3. 运行中监控状态

见 `examples/monitor_status.py`。更完整的可编辑版本见 `examples/mit_position_control.py`。

说明：

- **目标 vs 发送**：`set_joint_angles()` 设置的是目标；控制环按 `set_max_speeds()` 限速后逐步逼近，因此发送角度会滞后于目标。
- **温度**：由后台线程按 `telemetry_hz` 刷新，读 `last_temperatures` 即可，无需自己轮询 CAN。

### 4. 单关节微调

见 `examples/single_joint_adjust.py`。默认目标：J1=+25°、J2=+15°、J3=+15°、J4=-15°、J5/J6=0°、夹爪 J7=180°。`joint_id` 1–6 为臂关节，**7 为夹爪**（CAN ID 7）。

### 5. 读取实际位置

见 `examples/read_joint_angles.py`。流程：`connect()`（切 MIT、使能、读当前角度）→ `disable_motors()`（失能，便于手动推动）→ 循环 `read_joint_angles()` 打印实际角度。按 Ctrl+C 结束，`stop(return_to_zero=False)` 关闭 CAN，不回零。

`read_joint_angles()` 会同步访问 CAN，不宜在极高频率循环中调用；监控下发进度用 `get_command_angles()` 即可。

### 6. 安全停止的几种方式

见 `examples/stop_options.py`，通过参数选择停止模式：

```bash
python3 examples/stop_options.py default      # 缓慢回零后失能（推荐）
python3 examples/stop_options.py no_return    # 不回零，直接失能
python3 examples/stop_options.py async        # stop(wait=False) + wait_until_stopped()
python3 examples/stop_options.py emergency    # 紧急失能，不回零
```

用户交互退出：运行中按 **Esc** 或 **Ctrl+C** 也会触发与 `stop()` 类似的回零流程；回零过程中再次 **Ctrl+C** 会立即中止回零并失能。

### 7. 推荐程序结构

见 `examples/recommended_structure.py`：`try` / `except` / `finally` 模板，确保异常时也能安全 `stop()`。

## 单位约定

- 外部接口（目标角度、速度）：度、度/秒
- MotorBridge MIT 指令内部：弧度、弧度/秒

## 注意事项

- **无软件限位**：本仓库不提供关节或工作空间软件限位；使用前请阅读上文「警告」。
- 运行前请确认机械臂周围无遮挡，首次使用建议先用小角度、低速度测试，并保持在约 70% 工作空间内。
- 通信异常时程序不会尝试回零，而是立即紧急失能。
- `control_hz` 只是指令发送频率，机械臂实际运动速度由 `set_max_speeds` 决定。
