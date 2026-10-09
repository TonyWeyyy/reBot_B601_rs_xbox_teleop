# reBot B601-RS Xbox 360 手柄遥操作（独立项目）

用 Xbox 360 手柄直接点动（Jog）reBot B601-RS 的 6 个关节 + 夹爪，全程带软件限位、死人开关和安全回零。

**本项目完全独立**：rebot 控制 SDK（`rebot/` 包）和配置文件（`config/rebotarm_rs.yaml`）已内嵌，不需要放在任何其他项目目录里，也不需要再克隆官方仓库。

## 项目结构

```
rebot_xbox_teleop/
├── xbox_teleop.py          # 主程序（可调参数集中在文件顶部）
├── rebot/                  # 内嵌的 rebot 控制 SDK（ReBotRSMITController）
│   ├── __init__.py
│   ├── config.py
│   └── controller.py
├── config/
│   └── rebotarm_rs.yaml    # CAN / 电机 kp·kd / 温度保护 / 回零参数
├── requirements.txt        # pygame + motorbridge + pynput + pyyaml
├── install.sh              # 一键安装依赖（自动识别 conda 环境或建 .venv）
├── SDK_README_zh.md        # 内嵌 SDK 的官方说明（备查）
└── README.md               # 本文件
```

## 一、控制原理

```
Xbox360 手柄 (USB/接收器)
  → pygame 50Hz 轮询（死区 + 立方曲线）
  → 各关节速度指令（度/秒）
  → 积分成目标角度 + 软件限位钳制（URDF 限位 × 0.7 安全余量）
  → rebot SDK: set_joint_angles(7 个目标)
  → SDK 内部 200Hz 限速平滑（MIT 模式）
  → SocketCAN can0 @ 1Mbps → RobStride 电机 J1~J7
```

选关节空间点动而非笛卡尔控制：不需要 IK，没有奇异位形风险；官方 SDK 的 MIT 位置控制自带 200Hz 速度平滑，摇杆积分目标天然平滑。官方明确警告 SDK **不提供任何软件关节限位**且要求限制在约 70% 工作空间内 —— 本方案把 URDF 限位（×0.7 余量）做进代码，这是关键保护层。

## 二、安全机制

| 机制 | 实现 |
|---|---|
| 死人开关 | 必须按住 **LB** 才积分目标，松开立即冻结 |
| 软件限位 | URDF 关节限位 × 0.7（约 70% 工作空间，官方要求） |
| 安全停止 | B / Esc / Ctrl+C → 缓慢回零后失能。**绝不在弯肘位姿直接失能**（自由下落风险） |
| 速度三档 | RB 循环 25% → 50% → 100%，默认最低档启动 |
| 手柄掉线 | 立即冻结并安全停止 |
| 单一 CAN 主站 | 夹爪走 SDK 的 J7 通道，不另开 motorbridge 连接 |
| 温度保护 | SDK 自带三级 MOS 温度保护（报警 → 回零失能 → 紧急失能），主程序周期打印温度 |

## 三、手柄映射

| 输入 | 关节 | 输入 | 功能 |
|---|---|---|---|
| 左摇杆 X/Y | J1 底座旋转 / J2 肩 | LB（按住） | 死人开关 |
| 右摇杆 X/Y | J4 腕旋转 / J3 肘 | RB | 速度档位切换 |
| 十字键 ↑↓/←→ | J5 / J6 | Y | 全部关节回零 |
| LT / RT 扳机 | 夹爪闭 / 开 | B / Esc / Ctrl+C | 安全停止退出 |

方向手感不对时，改 `xbox_teleop.py` 顶部 `SIGN` 列表对应项的符号。

**换手柄 / 非标准手柄（强烈推荐）**：运行交互式映射向导，按提示一个一个按键/推摇杆，自动识别并生成 `gamepad_map.json`，主程序启动时自动加载：

```bash
python3 setup_gamepad.py
```

每一步按回车后操作手柄即可自动识别；某项不想设置就 5 秒不碰手柄跳过（该项沿用默认 Xbox360 布局）。扳机是轴还是按键、十字键是 hat 还是按键，都会自动适配。

也可以手动排查原始编号：

```bash
python3 xbox_teleop.py --debug-pad   # 实时打印所有轴/键/十字键原始值
```

程序已内置 1 秒启动保护窗口，自动忽略部分手柄初始化时发出的杂散按键事件。

## 四、安装与运行

```bash
cd rebot_xbox_teleop
bash install.sh        # 自动识别：conda 环境直接装；否则创建 .venv
```

手动安装也可以：`pip install -r requirements.txt`（需要 Python ≥ 3.9，推荐官方教程的 conda rebot 环境，Python 3.12）。

配置 CAN（每次重插 USB2CAN 后都要重做）：

```bash
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 up type can bitrate 1000000
```

运行（项目目录下，或任意目录用绝对路径调用都行）：

```bash
python3 xbox_teleop.py --dry-run   # 先空跑：不连机械臂，打印目标角度，验证映射
python3 xbox_teleop.py             # 实机运行
python3 xbox_teleop.py --config /path/to/rebotarm_rs.yaml   # 用别的配置
```

前提：机械臂已按官方快速入门完成校零、并在 motorbridge-studio 套用了默认电机参数模板。电机 kp/kd、温度阈值、回零速度等在 `config/rebotarm_rs.yaml` 里调整，不用改代码。

## 五、首次使用检查单

1. `--dry-run` 下推满每个摇杆，确认打印的目标角度不超限、方向符合预期；
2. 实机首次运行：不按 LB，机械臂应纹丝不动；
3. 按住 LB 轻推 J1 摇杆，确认转向正确（不对就改 `SIGN[0]`），逐个关节验证；
4. 按 Y 确认回零正常；按 B 确认缓慢回零后失能；
5. 长时间运行注意 J3（肘）电机温度，零位姿对 J3 重力矩最小，休息时可回零。
