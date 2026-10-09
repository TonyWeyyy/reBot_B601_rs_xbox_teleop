#!/usr/bin/env bash
# rebot B601-RS Xbox360 遥操作 —— 一键安装脚本
# 用法: bash install.sh
#
# 策略:
#   1. 如果当前已在某个 conda 环境(如 rebot)中 -> 直接 pip 安装到该环境;
#   2. 否则在项目目录创建 .venv 虚拟环境并安装。
# rebot SDK 已内嵌在 ./rebot,无需再克隆任何仓库。

set -e
cd "$(dirname "$0")"

if [ -n "$CONDA_DEFAULT_ENV" ]; then
    echo "[install] 检测到 conda 环境: $CONDA_DEFAULT_ENV,直接安装依赖"
    pip install -r requirements.txt
    PYTHON_BIN="$(command -v python3)"
else
    echo "[install] 创建虚拟环境 .venv"
    python3 -m venv .venv
    ./.venv/bin/pip install --upgrade pip
    ./.venv/bin/pip install -r requirements.txt
    PYTHON_BIN="$(pwd)/.venv/bin/python3"
fi

echo
echo "[install] 依赖安装完成。"
echo "[install] 验证 import ..."
"$PYTHON_BIN" - <<'EOF'
import sys
sys.path.insert(0, ".")
import pygame, yaml
from rebot import ReBotRSMITController, load_config
cfg = load_config("config/rebotarm_rs.yaml")
print("pygame OK, rebot SDK OK, 配置加载 OK, CAN 通道:", cfg.can.channel)
EOF

echo
echo "运行方式:"
echo "  $PYTHON_BIN xbox_teleop.py --dry-run   # 先空跑验证手柄映射"
echo "  $PYTHON_BIN xbox_teleop.py             # 实机运行"
echo
echo "提醒: 实机前请先配置 CAN: sudo ip link set can0 down; sudo ip link set can0 up type can bitrate 1000000"
