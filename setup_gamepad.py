#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
setup_gamepad.py — 手柄映射向导

逐个提示你按按键 / 推摇杆，自动识别物理编号，生成 gamepad_map.json。
xbox_teleop.py 启动时如果检测到同目录下有 gamepad_map.json，会自动使用它。

用法：
  python3 setup_gamepad.py            # 完整向导
  python3 setup_gamepad.py -o my.json # 输出到指定文件

每一步：按回车开始 -> 按提示操作手柄 -> 自动识别。
不想设置的项：按回车后 5 秒内不碰手柄，自动跳过（沿用默认 Xbox360 映射）。
"""

import argparse
import json
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent

AXIS_THRESHOLD = 0.5      # 轴变化超过该值判定为"有动作"
SETTLE_S = 0.3            # 识别后等待回弹时间
TIMEOUT_S = 5.0           # 每步超时（超时 = 跳过）
POLL_HZ = 200.0


def snapshot(js):
    """读取全部轴/键/十字键的当前状态。"""
    axes = [js.get_axis(i) for i in range(js.get_numaxes())]
    buttons = [js.get_button(i) for i in range(js.get_numbuttons())]
    hats = [js.get_hat(h) for h in range(js.get_numhats())]
    return axes, buttons, hats


def wait_for_action(js, baseline, timeout=TIMEOUT_S):
    """轮询直到检测到手柄动作。返回描述 dict 或 None（超时）。

    返回类型：
      {"type": "button", "index": i}
      {"type": "axis",   "index": i, "value": v, "rest": rest_v}
      {"type": "hat",    "index": h, "value": (x, y)}
    """
    import pygame
    base_axes, base_buttons, base_hats = baseline
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        pygame.event.pump()
        # 按键：松开 -> 按下
        for i in range(js.get_numbuttons()):
            if not base_buttons[i] and js.get_button(i):
                return {"type": "button", "index": i}
        # 十字键
        for h in range(js.get_numhats()):
            if js.get_hat(h) != (0, 0) and js.get_hat(h) != base_hats[h]:
                return {"type": "hat", "index": h, "value": js.get_hat(h)}
        # 轴：相对基线变化超过阈值
        for i in range(js.get_numaxes()):
            v = js.get_axis(i)
            if abs(v - base_axes[i]) > AXIS_THRESHOLD:
                return {"type": "axis", "index": i, "value": v, "rest": base_axes[i]}
        time.sleep(1.0 / POLL_HZ)
    return None


def wait_release(js, timeout=3.0):
    """等待所有输入回弹（按键松开、轴回中），避免一次动作被识别两次。"""
    import pygame
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        pygame.event.pump()
        if (not any(js.get_button(i) for i in range(js.get_numbuttons()))
                and all(abs(js.get_axis(i)) < 0.3 for i in range(js.get_numaxes()))
                and all(js.get_hat(h) == (0, 0) for h in range(js.get_numhats()))):
            return
        time.sleep(0.02)


# 向导步骤：(逻辑名, 提示语, 期望类型)
# 期望类型仅用于提示，实际按识别到的为准（扳机/十字键在不同手柄上可能是轴也可能是键）
STEPS = [
    ("lb",     "死人开关（默认 LB）：请【按住不放】的那个肩键", None),
    ("rb",     "速度切换（默认 RB）：另一个肩键", None),
    ("home",   "回零键（默认 Y / 三角）", None),
    ("quit",   "退出键（默认 B / 圆圈）—— 按它会安全停止退出", None),
    ("lx",     "左摇杆【向左】推到底（J1 底座旋转）", "axis"),
    ("ly",     "左摇杆【向上】推到底（J2 肩）", "axis"),
    ("rx",     "右摇杆【向左】推到底（J4 腕旋转）", "axis"),
    ("ry",     "右摇杆【向上】推到底（J3 肘）", "axis"),
    ("lt",     "左扳机 LT【按到底】（夹爪闭合）", None),
    ("rt",     "右扳机 RT【按到底】（夹爪张开）", None),
    ("up",     "十字键【上】（J5 正方向）", None),
    ("down",   "十字键【下】", None),
    ("left",   "十字键【左】（J6 正方向）", None),
    ("right",  "十字键【右】", None),
]


def describe(a):
    if a["type"] == "button":
        return f"button {a['index']}"
    if a["type"] == "axis":
        return f"axis {a['index']} (rest={a['rest']:+.2f}, pressed={a['value']:+.2f})"
    return f"hat {a['index']} value={a['value']}"


def main():
    ap = argparse.ArgumentParser(description="手柄映射向导")
    ap.add_argument("-o", "--output", default=str(SCRIPT_DIR / "gamepad_map.json"),
                    help="输出文件路径（默认 ./gamepad_map.json）")
    args = ap.parse_args()

    import pygame
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("[错误] 未检测到手柄，请连接后重试。")
        sys.exit(1)
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"[手柄] {js.get_name()}  axes={js.get_numaxes()} "
          f"buttons={js.get_numbuttons()} hats={js.get_numhats()}")
    print("\n接下来会逐项提示。每项：按回车 -> 按提示操作手柄 -> 自动识别。")
    print("跳过某项：按回车后 5 秒内不碰手柄即可（该项沿用默认映射）。\n")

    mapping = {"name": js.get_name(), "controls": {}}
    for name, prompt, _ in STEPS:
        input(f"[{name:>5}] {prompt}\n        准备好后按回车，然后操作手柄…")
        pygame.event.pump()
        time.sleep(0.1)
        baseline = snapshot(js)
        act = wait_for_action(js, baseline)
        if act is None:
            print(f"        -> 超时，跳过（沿用默认）\n")
            continue
        print(f"        -> 识别到: {describe(act)}\n")
        mapping["controls"][name] = act
        wait_release(js)
        time.sleep(SETTLE_S)

    out = Path(args.output)
    out.write_text(json.dumps(mapping, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[完成] 映射已保存到 {out}")
    print("现在运行 python3 xbox_teleop.py 会自动加载该映射。")
    print("如果某个方向反了，改 xbox_teleop.py 顶部的 SIGN 列表即可。")
    pygame.quit()


if __name__ == "__main__":
    main()
