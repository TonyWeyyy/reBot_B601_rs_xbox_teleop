#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rebot B601-RS — 手柄关节点动遥操作（Jog Teleop）

原理：
  手柄摇杆 -> 各关节速度指令（度/秒）-> 积分成目标角度 -> 软件限位钳制
  -> rebot SDK 的 set_joint_angles()（SDK 内部 200Hz 限速平滑下发到电机）

手柄映射：
  默认使用文件顶部 DEFAULT_MAP 的标准 Xbox360 布局。
  如果同目录下存在 gamepad_map.json（用 setup_gamepad.py 向导生成），
  则自动改用它；也可以用 --map 指定其他映射文件。

安全设计：
  1. 死人开关：必须按住才允许任何运动，松开即冻结。
  2. 软件关节限位（URDF 限位 × 0.7 安全余量，官方建议约 70% 工作空间）。
  3. 摇杆死区 + 立方曲线：低速精细、高速受限。
  4. 三档全局速度（25% / 50% / 100%），默认最低档。
  5. 回零键回零；退出键 / Ctrl+C / Esc 触发安全停止（缓慢回零后失能）。
     —— 绝不在非零位姿直接失能电机（弯肘失能 = 手臂自由下落）。
  6. 手柄拔出 / 掉线：立即冻结目标并安全停止。
  7. 全程只有 rebot SDK 一个 CAN 主站（夹爪走 J7 通道）。
  8. 启动 1 秒保护窗口，忽略部分手柄初始化时的杂散按键事件。

本目录已内嵌 rebot SDK（rebot/ 包 + config/rebotarm_rs.yaml），完全独立。

运行：
  python3 setup_gamepad.py          # 首次：手柄映射向导（生成 gamepad_map.json）
  python3 xbox_teleop.py            # 实机
  python3 xbox_teleop.py --dry-run  # 不接机械臂，只打印目标角度（调映射用）
  python3 xbox_teleop.py --debug-pad    # 打印手柄原始轴/键值
  python3 xbox_teleop.py --map my.json  # 指定映射文件
  python3 xbox_teleop.py --config /path/to/rebotarm_rs.yaml
"""

import argparse
import json
import sys
import time
from pathlib import Path

# 让“脚本所在目录”始终在 import 路径中：内嵌的 rebot 包在脚本同级，
# 这样从任何工作目录运行都能找到它。
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

DEFAULT_CONFIG = SCRIPT_DIR / "config" / "rebotarm_rs.yaml"
DEFAULT_MAP_PATH = SCRIPT_DIR / "gamepad_map.json"

# ======================== 可调参数区 ========================

LOOP_HZ = 50.0                # 手柄轮询 / 目标更新频率（SDK 内部 200Hz 平滑下发）

DEADZONE = 0.12               # 摇杆死区（0~1）
USE_CUBIC_CURVE = True        # True: 输出 = x^3（低速更精细）

# 各关节最大点动速度（度/秒），100% 档位时的上限。
# 首次使用建议保持这个保守值，熟练后再调大。
JOG_SPEEDS_DEG_S = [30.0, 20.0, 20.0, 30.0, 40.0, 60.0, 60.0]   # J1..J6, J7夹爪

# 软件关节限位（度）。来源：ReBot_Arm_RS.urdf 的 <limit>（弧度换算），
# 再乘以 LIMIT_SAFE_SCALE 安全余量（官方建议限制在约 70% 工作空间内）。
# 注意：rebot SDK 本身不做任何限位，这里是唯一的软件保护，勿随意放开。
LIMIT_SAFE_SCALE = 0.7
URDF_LIMITS_DEG = [           # J1..J6 的 (lower, upper)，来自 URDF
    (-160.4, 160.4),          # J1  ±2.8 rad
    ( -90.0,  90.0),          # J2  URDF 为 0~3.14 rad，相对电机零点对称化
    ( -90.0,  90.0),          # J3  同上
    ( -90.0,  90.0),          # J4  ±1.57 rad
    ( -90.0,  90.0),          # J5  ±1.57 rad
    (-180.0, 180.0),          # J6  ±3.14 rad
]

GRIPPER_MIN_DEG = 0.0         # 夹爪全闭（零点校准位）
GRIPPER_MAX_DEG = 170.0       # 夹爪张开上限（官方示例用 180°，留 10° 余量）

JOINT_LIMITS = [
    (lo * LIMIT_SAFE_SCALE, hi * LIMIT_SAFE_SCALE) for (lo, hi) in URDF_LIMITS_DEG
] + [(GRIPPER_MIN_DEG, GRIPPER_MAX_DEG)]

SPEED_LEVELS = [0.25, 0.5, 1.0]   # 循环切换的全局速度倍率，默认从 0.25 起步

# 各轴方向修正：某关节运动方向与手感不符就改对应符号（+1 / -1）
SIGN = [+1, -1, -1, +1, +1, +1, +1]   # J1..J6, J7夹爪

TEMP_PRINT_INTERVAL = 2.0     # 秒，周期打印电机 MOS 温度

STARTUP_GRACE_S = 1.0         # 启动保护窗口：忽略此期间内的退出/回零按键事件
                              # （部分安卓协议手柄会在初始化时发出杂散按键事件）

# ---- 默认手柄映射（标准 Xbox360 / Linux xpad 布局）----
# 结构同 setup_gamepad.py 生成的 gamepad_map.json：
#   button 型: {"type": "button", "index": i}
#   axis 型:   {"type": "axis", "index": i, "value": v, "rest": r}
#              （摇杆：value 为录制方向上的值，自动定正负；扳机：rest/pressed 归一化）
#   hat 型:    {"type": "hat", "index": h, "value": (x, y)}
DEFAULT_MAP = {
    "name": "default-xbox360",
    "controls": {
        "lb":    {"type": "button", "index": 4},
        "rb":    {"type": "button", "index": 5},
        "home":  {"type": "button", "index": 3},
        "quit":  {"type": "button", "index": 1},
        "lx":    {"type": "axis", "index": 0, "value": -1.0, "rest": 0.0},
        "ly":    {"type": "axis", "index": 1, "value": -1.0, "rest": 0.0},
        "rx":    {"type": "axis", "index": 3, "value": -1.0, "rest": 0.0},
        "ry":    {"type": "axis", "index": 4, "value": -1.0, "rest": 0.0},
        "lt":    {"type": "axis", "index": 2, "value": 1.0, "rest": -1.0},
        "rt":    {"type": "axis", "index": 5, "value": 1.0, "rest": -1.0},
        "up":    {"type": "hat", "index": 0, "value": [0, 1]},
        "down":  {"type": "hat", "index": 0, "value": [0, -1]},
        "left":  {"type": "hat", "index": 0, "value": [-1, 0]},
        "right": {"type": "hat", "index": 0, "value": [1, 0]},
    },
}

# ============================================================


def apply_curve(x: float) -> float:
    """死区 + 立方曲线，返回 -1..1。"""
    if abs(x) < DEADZONE:
        return 0.0
    x = (abs(x) - DEADZONE) / (1.0 - DEADZONE) * (1 if x > 0 else -1)
    return x * x * x if USE_CUBIC_CURVE else x


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def load_mapping(map_path: str | None) -> dict:
    """加载映射：--map 指定 > 同目录 gamepad_map.json > 内置默认。"""
    path = Path(map_path) if map_path else DEFAULT_MAP_PATH
    if path.is_file():
        data = json.loads(path.read_text(encoding="utf-8"))
        # 合并：向导里跳过的项回落到默认映射
        merged = {k: dict(v) for k, v in DEFAULT_MAP["controls"].items()}
        merged.update(data.get("controls", {}))
        print(f"[映射] 使用 {path}（手柄: {data.get('name', '?')}）")
        return {"name": data.get("name", "?"), "controls": merged}
    if map_path:
        print(f"[警告] 映射文件不存在: {path}，改用内置默认。")
    print("[映射] 使用内置默认 Xbox360 布局（可运行 setup_gamepad.py 生成自定义映射）")
    return DEFAULT_MAP


class XboxTeleop:
    def __init__(self, dry_run: bool, config_path: str | None = None,
                 map_path: str | None = None):
        self.dry_run = dry_run
        self.targets = [0.0] * 7       # 目标角度（度），启动后用当前实际角度初始化
        self.speed_level = 0           # SPEED_LEVELS 下标
        self.running = True
        self.arm = None
        self.t_start = time.monotonic()   # 用于启动保护窗口
        self.map = load_mapping(map_path)

        # ---- 手柄初始化 ----
        import pygame
        self.pygame = pygame
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            print("[错误] 未检测到手柄。请确认手柄已连接。")
            sys.exit(1)
        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        print(f"[手柄] {self.js.get_name()}  axes={self.js.get_numaxes()} "
              f"buttons={self.js.get_numbuttons()} hats={self.js.get_numhats()}")

        # ---- 机械臂初始化 ----
        if not dry_run:
            from rebot import ReBotRSMITController, load_config
            cfg_path = Path(config_path) if config_path else DEFAULT_CONFIG
            if not cfg_path.is_file():
                print(f"[错误] 配置文件不存在: {cfg_path}")
                sys.exit(1)
            self.arm = ReBotRSMITController(load_config(str(cfg_path)))
            print(f"[配置] {cfg_path}")
            self.arm.start(enable_esc=True)     # connect+使能+控制线程，Esc 可急停
            self.arm.set_max_speeds(self._scaled_speeds())
            # 用当前实际角度做目标初值，避免启动瞬间跳动
            try:
                self.targets = list(self.arm.get_command_angles())
            except Exception:
                pass
            print("[机械臂] 已连接并使能，当前角度:", [round(a, 1) for a in self.targets])
        else:
            print("[DRY-RUN] 不连接机械臂，仅打印目标角度。")

        print("\n=== 操作说明 ===")
        print("  左摇杆  : J1 底座旋转 / J2 肩")
        print("  右摇杆  : J4 腕旋转   / J3 肘")
        print("  十字键  : J5 / J6")
        print("  LT / RT : 夹爪闭 / 开")
        print("  死人开关(按住): 运动使能，松开即停")
        print("  速度键  : 速度档位 25% -> 50% -> 100%")
        print("  回零键  : 全部关节回零")
        print("  退出键 / Esc / Ctrl+C : 安全停止（缓慢回零后失能）\n")

    def _scaled_speeds(self):
        k = SPEED_LEVELS[self.speed_level]
        return [s * k for s in JOG_SPEEDS_DEG_S]

    # ---------- 映射解析 ----------

    def _ctl(self, name):
        return self.map["controls"].get(name)

    def _read_button(self, name) -> bool:
        c = self._ctl(name)
        if not c:
            return False
        js = self.js
        if c["type"] == "button":
            return js.get_numbuttons() > c["index"] and bool(js.get_button(c["index"]))
        if c["type"] == "hat":
            if js.get_numhats() <= c["index"]:
                return False
            return tuple(js.get_hat(c["index"])) == tuple(c["value"])
        if c["type"] == "axis":   # 某些手柄的“键”其实是轴
            if js.get_numaxes() <= c["index"]:
                return False
            span = c["value"] - c.get("rest", 0.0)
            if abs(span) < 1e-6:
                return False
            return ((js.get_axis(c["index"]) - c.get("rest", 0.0)) / span) > 0.5
        return False

    def _read_axis(self, name) -> float:
        """读取摇杆类轴，返回 -1..1（录制方向为正）。非轴型映射返回 0。"""
        c = self._ctl(name)
        if not c or c["type"] != "axis" or self.js.get_numaxes() <= c["index"]:
            return 0.0
        v = self.js.get_axis(c["index"])
        # 录制时“期望方向”的值决定了正负
        return -v if c.get("value", 1.0) < 0 else v

    def _read_trigger(self, name) -> float:
        """扳机归一化到 0..1；button 型则 0/1。"""
        c = self._ctl(name)
        if not c:
            return 0.0
        if c["type"] == "button":
            return 1.0 if self._read_button(name) else 0.0
        if c["type"] == "axis" and self.js.get_numaxes() > c["index"]:
            rest, pressed = c.get("rest", -1.0), c.get("value", 1.0)
            span = pressed - rest
            if abs(span) < 1e-6:
                return 0.0
            return clamp((self.js.get_axis(c["index"]) - rest) / span, 0.0, 1.0)
        return 0.0

    # ---------- 输入解析 ----------

    def _read_inputs(self):
        """返回 (jog[7] in -1..1, deadman, events dict)"""
        pg = self.pygame
        ev = {"speed_up": False, "home": False, "quit": False}
        in_grace = (time.monotonic() - self.t_start) < STARTUP_GRACE_S
        for e in pg.event.get():
            if e.type == pg.JOYBUTTONDOWN:
                if in_grace:
                    print(f"[忽略] 启动保护期内的按键事件: button {e.button}")
                    continue
                # 事件只用于即时动作；与映射里的逻辑键比对
                if self._match_event("rb", e.button):
                    ev["speed_up"] = True
                elif self._match_event("home", e.button):
                    ev["home"] = True
                elif self._match_event("quit", e.button):
                    # 退出前确认按键仍处于按下状态，进一步排除杂散事件
                    if self._read_button("quit"):
                        ev["quit"] = True
                    else:
                        print(f"[忽略] 按键 {e.button} 未保持按下，判定为杂散事件")
            elif e.type == pg.JOYDEVICEREMOVED:
                print("\n[警告] 手柄已拔出！冻结所有运动。")
                ev["quit"] = True
            elif e.type == pg.QUIT:
                ev["quit"] = True

        jog = [0.0] * 7
        jog[0] = apply_curve(self._read_axis("lx"))       # J1
        jog[1] = apply_curve(self._read_axis("ly"))       # J2
        jog[2] = apply_curve(self._read_axis("ry"))       # J3
        jog[3] = apply_curve(self._read_axis("rx"))       # J4
        jog[4] = float(self._read_button("up")) - float(self._read_button("down"))    # J5
        jog[5] = float(self._read_button("left")) - float(self._read_button("right")) # J6
        jog[6] = apply_curve(self._read_trigger("rt")) \
               - apply_curve(self._read_trigger("lt"))    # 夹爪

        deadman = self._read_button("lb")
        # 死人开关也可用事件外的瞬时状态，实时性好；事件循环只处理单次动作
        if deadman and not in_grace:
            # 支持 home/quit/speed 映射成十字键或轴的极端情况：轮询状态沿触发
            pass
        return jog, deadman, ev

    def _match_event(self, name, button_index) -> bool:
        c = self._ctl(name)
        return bool(c and c["type"] == "button" and c["index"] == button_index)

    # ---------- 主循环 ----------

    def run(self):
        dt = 1.0 / LOOP_HZ
        last_temp_print = 0.0
        # 状态沿检测：速度/回零键若被映射成非 button 类型（hat/axis），用轮询沿
        prev_speed, prev_home, prev_quit = False, False, False
        try:
            while self.running:
                t0 = time.monotonic()
                jog, deadman, ev = self._read_inputs()

                # 非 button 型逻辑键的轮询沿触发
                if (time.monotonic() - self.t_start) >= STARTUP_GRACE_S:
                    cur_speed = (self._ctl("rb") and self._ctl("rb")["type"] != "button"
                                 and self._read_button("rb"))
                    cur_home = (self._ctl("home") and self._ctl("home")["type"] != "button"
                                and self._read_button("home"))
                    cur_quit = (self._ctl("quit") and self._ctl("quit")["type"] != "button"
                                and self._read_button("quit"))
                    if cur_speed and not prev_speed:
                        ev["speed_up"] = True
                    if cur_home and not prev_home:
                        ev["home"] = True
                    if cur_quit and not prev_quit:
                        ev["quit"] = True
                    prev_speed, prev_home, prev_quit = cur_speed, cur_home, cur_quit

                if ev["quit"]:
                    break
                if ev["speed_up"]:
                    self.speed_level = (self.speed_level + 1) % len(SPEED_LEVELS)
                    print(f"[速度] 全局倍率 -> {SPEED_LEVELS[self.speed_level]*100:.0f}%")
                    if self.arm:
                        self.arm.set_max_speeds(self._scaled_speeds())
                if ev["home"]:
                    self.targets[:6] = [0.0] * 6
                    print("[回零] 关节目标归零（夹爪保持）。")

                if deadman:
                    k = SPEED_LEVELS[self.speed_level]
                    for i in range(7):
                        delta = SIGN[i] * jog[i] * JOG_SPEEDS_DEG_S[i] * k * dt
                        lo, hi = JOINT_LIMITS[i]
                        self.targets[i] = clamp(self.targets[i] + delta, lo, hi)

                if self.arm:
                    self.arm.set_joint_angles(self.targets)
                    now = time.monotonic()
                    if now - last_temp_print > TEMP_PRINT_INTERVAL:
                        last_temp_print = now
                        temps = getattr(self.arm, "last_temperatures", None)
                        if temps:
                            # 温度可能尚未轮询到（None），显示为 "--"
                            tstr = [f"{t:.0f}" if isinstance(t, (int, float)) else "--"
                                    for t in temps]
                            print("[温度]", tstr,
                                  " 目标:", [round(a, 1) for a in self.targets])
                else:
                    print("\r[DRY-RUN] 目标:", [round(a, 1) for a in self.targets],
                          f" deadman={deadman}   ", end="", flush=True)

                elapsed = time.monotonic() - t0
                if elapsed < dt:
                    time.sleep(dt - elapsed)
        except KeyboardInterrupt:
            print("\n[Ctrl+C] 收到退出信号。")
        finally:
            self.shutdown()

    def shutdown(self):
        print("[停止] 缓慢回零并失能电机……")
        if self.arm:
            # 关键安全点：必须 return_to_zero=True, wait=True。
            # 绝不在非零位姿直接失能（弯肘失能 = 自由下落）。
            self.arm.stop(return_to_zero=True, wait=True)
        self.pygame.quit()
        print("[完成] 已安全停止。")


def debug_pad():
    """手柄映射诊断：实时打印原始轴/键/十字键值，用于手动排查。
    不连接机械臂。按 Ctrl+C 退出。"""
    import pygame
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("[错误] 未检测到手柄。")
        sys.exit(1)
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"[手柄] {js.get_name()}  axes={js.get_numaxes()} "
          f"buttons={js.get_numbuttons()} hats={js.get_numhats()}")
    print("逐个推摇杆 / 按键，观察对应的 axis/button 编号和方向。")
    print("（一般情况下用 setup_gamepad.py 向导更方便）\n")
    try:
        while True:
            pygame.event.pump()
            ax = [f"{i}:{js.get_axis(i):+.2f}" for i in range(js.get_numaxes())]
            bt = [str(i) for i in range(js.get_numbuttons()) if js.get_button(i)]
            hats = [str(js.get_hat(h)) for h in range(js.get_numhats())]
            print(f"\raxes[{' '.join(ax)}]  buttons[{','.join(bt)}]  hats{hats}   ",
                  end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[完成] 退出诊断。")
        pygame.quit()


def main():
    ap = argparse.ArgumentParser(description="reBot B601-RS 手柄遥操作")
    ap.add_argument("--dry-run", action="store_true", help="不连接机械臂，仅打印目标")
    ap.add_argument("--debug-pad", action="store_true",
                    help="手柄映射诊断：打印原始轴/键值（不连接机械臂）")
    ap.add_argument("--map", default=None,
                    help="手柄映射 json（默认自动找同目录 gamepad_map.json，找不到用内置默认）")
    ap.add_argument("--config", default=None,
                    help="rebotarm_rs.yaml 路径（默认用项目自带 config/ 下的）")
    args = ap.parse_args()
    if args.debug_pad:
        debug_pad()
        return
    XboxTeleop(dry_run=args.dry_run, config_path=args.config,
               map_path=args.map).run()


if __name__ == "__main__":
    main()
