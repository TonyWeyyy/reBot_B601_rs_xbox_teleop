#!/usr/bin/env python3
"""
Interface layer: reBot B601-RS MIT controller.

Features:
1. Continuously sends position commands in MIT mode
2. Independent speed limit for each joint
3. Real-time monitoring of RobStride MOS temperature
4. Three-level protection: temperature alarm, over-temperature return-to-zero, and emergency disable
5. Slowly returns to zero when Esc or Ctrl+C is pressed or arm.stop() is called
6. Pressing Ctrl+C again during return-to-zero immediately aborts the return and disables the motors

接口层：reBot B601-RS MIT 控制器。

功能：
1. MIT 模式持续发送位置指令
2. 各关节独立的速度限制
3. 实时监控 RobStride MOS 温度
4. 温度报警、超温回零、紧急失能三级保护
5. 按 Esc、Ctrl+C 或调用 arm.stop() 时缓慢回到零点
6. 回零过程中再次按 Ctrl+C，立即中止回零并失能

All hardware and safety parameters come from the configuration layer (rebot/config.py reads config/*.yaml).
所有硬件与安全参数来自配置层（rebot/config.py 读取 config/*.yaml）。

Angle units:
    External target angles: degrees
    MotorBridge MIT commands: radians
角度单位：
    外部目标角度：度
    MotorBridge MIT 指令：弧度
"""

from __future__ import annotations

import math
import signal
import threading
import time
from contextlib import contextmanager
from typing import Iterator, Sequence

from motorbridge import Controller, Mode

try:
    from pynput import keyboard
except Exception:
    keyboard = None

from .config import (
    DEFAULT_CONFIG_PATH,
    ControllerConfig,
    load_config,
)


class ReBotRSMITController:
    """reBot B601-RS MIT controller. / reBot B601-RS MIT 控制器。"""

    # Max wait for io_lock during shutdown; avoids deadlock if CAN I/O is stuck.
    # 关机时等待 io_lock 的最长时间；CAN I/O 卡住时避免死锁。
    _IO_LOCK_SHUTDOWN_TIMEOUT_S = 2.0

    # Clear alarm latch when temperature drops this many °C below alarm_c.
    # 温度低于 alarm_c 这么多 °C 时清除报警锁存。
    _TEMP_ALARM_HYSTERESIS_C = 2.0

    def __init__(
        self,
        config: ControllerConfig | None = None,
    ) -> None:
        if config is None:
            config = load_config()

            print(
                "[Config / 配置] Loaded config file: "
                f"{DEFAULT_CONFIG_PATH}"
                " / 已加载配置文件："
                f"{DEFAULT_CONFIG_PATH}"
            )

        self.config = config

        self.channel = self.config.channel
        self.control_hz = float(self.config.control_hz)
        self.telemetry_hz = float(
            self.config.telemetry_hz
        )

        self.temp_alarm_c = float(
            self.config.temperatures.alarm_c
        )
        self.temp_return_zero_c = float(
            self.config.temperatures.return_zero_c
        )
        self.temp_disconnect_c = float(
            self.config.temperatures.disconnect_c
        )

        self.motor_count = len(self.config.motors)

        self.controller: Controller | None = None
        self.motors = []

        # Final target set by the user, unit: radians. / 用户设置的最终目标，单位：弧度。
        self.target_positions = [
            0.0 for _ in self.config.motors
        ]

        # The target actually sent to the motors after speed limiting, unit: radians.
        # 经过速度限制后实际发送给电机的目标，单位：弧度。
        self.command_positions = [
            0.0 for _ in self.config.motors
        ]

        self.max_speeds_rad_s = [
            math.radians(20.0)
            for _ in self.config.motors
        ]

        self.target_lock = threading.RLock()
        self.io_lock = threading.RLock()
        self.shutdown_lock = threading.Lock()

        self.worker_stop_event = threading.Event()
        self.abort_return_event = threading.Event()
        self.shutdown_done_event = threading.Event()

        self.control_thread: threading.Thread | None = None
        self.telemetry_thread: threading.Thread | None = None
        self.shutdown_thread: threading.Thread | None = None

        self.keyboard_listener = None

        self.started = False
        self.shutdown_started = False

        self.return_to_zero_requested = True
        self.thermal_return_requested = False
        self.emergency_disconnect_requested = False

        self.shutdown_reason = ""
        self.signal_count = 0
        self.last_error: Exception | None = None

        self.last_temperatures: list[float | None] = [
            None for _ in self.config.motors
        ]

        # Edge-triggered temperature alarm state per motor. / 每个电机的温度报警边沿状态。
        self._temp_alarm_active = [
            False for _ in self.config.motors
        ]

    # -------------------------------------------------------------------------
    # IO lock helpers / IO 锁辅助
    # -------------------------------------------------------------------------

    def _try_acquire_io_lock(
        self,
        timeout: float | None = None,
    ) -> bool:
        """Acquire io_lock; timeout=None blocks. / 获取 io_lock；timeout=None 时阻塞等待。"""

        if timeout is None:
            self.io_lock.acquire()
            return True

        return self.io_lock.acquire(timeout=timeout)

    @contextmanager
    def _io_lock_guard(
        self,
        timeout: float | None = None,
    ) -> Iterator[None]:
        """Context manager for io_lock with optional timeout. / 带可选超时的 io_lock 上下文管理器。"""

        acquired = self._try_acquire_io_lock(timeout)

        if not acquired:
            raise TimeoutError(
                f"Timed out waiting for io_lock after {timeout}s"
                f" / 等待 io_lock 超时（{timeout} 秒）"
            )

        try:
            yield
        finally:
            self.io_lock.release()

    # -------------------------------------------------------------------------
    # Connection and startup / 连接与启动
    # -------------------------------------------------------------------------

    def connect(self) -> None:
        """Connect CAN, register motors, switch to MIT mode, and enable.
        连接 CAN、注册电机、切换 MIT 模式并使能。"""

        if self.controller is not None:
            return

        print(f"[Connect / 连接] Opening CAN interface: {self.channel} / 打开 CAN 接口：{self.channel}")

        self.controller = Controller(self.channel)

        try:
            for motor_config in self.config.motors:
                motor = self.controller.add_robstride_motor(
                    motor_config.motor_id,
                    self.config.host_id,
                    motor_config.model,
                )

                self.motors.append(motor)

                print(
                    f"[Connect / 连接] Motor {motor_config.motor_id} "
                    f"({motor_config.model}) registered / "
                    f"电机 {motor_config.motor_id} "
                    f"({motor_config.model}) 已注册"
                )

            for index, motor in enumerate(self.motors):
                motor_id = self.config.motors[index].motor_id

                print(
                    f"[Connect / 连接] Motor {motor_id} "
                    "switching to MIT mode / "
                    f"电机 {motor_id} "
                    "切换到 MIT 模式"
                )

                with self._io_lock_guard():
                    motor.ensure_mode(
                        Mode.MIT,
                        1000,
                    )

                time.sleep(0.05)

            # mechPos (0x7019) is reliably readable after enable.
            # 使能后 mechPos (0x7019) 才可稳定读取。
            with self._io_lock_guard():
                self.controller.enable_all()

            time.sleep(0.30)

            print("[Connect / 连接] All motors enabled / 所有电机已使能")

            # Read actual positions after enable to seed targets and avoid a jump.
            # 使能后读取实际位置，作为目标初值，避免突然跳动。
            current_positions = self._read_positions_rad()

            with self.target_lock:
                self.target_positions[:] = current_positions
                self.command_positions[:] = current_positions

            print(
                "[Connect / 连接] Current angles: "
                f"{[round(math.degrees(x), 2) for x in current_positions]}"
                " / 当前角度："
                f"{[round(math.degrees(x), 2) for x in current_positions]}"
            )

        except Exception:
            self._disable_and_close()
            raise

    def start(
        self,
        *,
        enable_esc: bool = True,
        install_signal_handlers: bool = True,
    ) -> None:
        """Start the MIT control thread and the temperature monitoring thread.
        启动 MIT 控制线程和温度监控线程。"""

        if self.started:
            return

        self.connect()

        self.worker_stop_event.clear()
        self.abort_return_event.clear()
        self.shutdown_done_event.clear()

        self.shutdown_started = False
        self.signal_count = 0
        self.last_error = None
        self._temp_alarm_active = [
            False for _ in self.config.motors
        ]
        self.started = True

        self.control_thread = threading.Thread(
            target=self._control_loop,
            name="rebot-rs-mit-control",
            daemon=True,
        )
        self.control_thread.start()

        self.telemetry_thread = threading.Thread(
            target=self._temperature_loop,
            name="rebot-rs-temperature",
            daemon=True,
        )
        self.telemetry_thread.start()

        if enable_esc:
            self._start_keyboard_listener()

        if (
            install_signal_handlers
            and threading.current_thread()
            is threading.main_thread()
        ):
            signal.signal(
                signal.SIGINT,
                self._signal_handler,
            )
            signal.signal(
                signal.SIGTERM,
                self._signal_handler,
            )

        print(
            f"[Start / 启动] MIT rate: "
            f"{self.control_hz:.0f} Hz"
            f" / MIT 频率："
            f"{self.control_hz:.0f} Hz"
        )
        print(
            f"[Start / 启动] Temperature refresh: "
            f"{self.telemetry_hz:.1f} Hz"
            f" / 温度刷新："
            f"{self.telemetry_hz:.1f} Hz"
        )
        print(
            "[Start / 启动] Temperature thresholds: "
            f"alarm={self.temp_alarm_c:.1f}°C, "
            f"return-to-zero={self.temp_return_zero_c:.1f}°C, "
            f"disconnect={self.temp_disconnect_c:.1f}°C"
            " / 温度阈值："
            f"报警={self.temp_alarm_c:.1f}°C，"
            f"回零={self.temp_return_zero_c:.1f}°C，"
            f"断开={self.temp_disconnect_c:.1f}°C"
        )
        print(
            "[Start / 启动] Esc / Ctrl+C / arm.stop() "
            "will slowly return to zero and then disable / "
            "Esc / Ctrl+C / arm.stop() "
            "会缓慢回零后失能"
        )

    # -------------------------------------------------------------------------
    # MIT control / MIT 控制
    # -------------------------------------------------------------------------

    def _control_loop(self) -> None:
        """Continuously generate speed-limited targets and send MIT commands.
        持续生成带速度限制的目标并发送 MIT 指令。"""

        period = 1.0 / self.control_hz
        next_tick = time.perf_counter()

        while not self.worker_stop_event.is_set():
            try:
                with self.target_lock:
                    for index in range(self.motor_count):
                        target = self.target_positions[index]
                        command = self.command_positions[index]

                        error = target - command

                        max_step = (
                            self.max_speeds_rad_s[index]
                            * period
                        )

                        step = max(
                            -max_step,
                            min(max_step, error),
                        )

                        self.command_positions[index] += step

                    commands = self.command_positions.copy()

                self._send_mit_positions(commands)

            except Exception as error:
                self.last_error = error

                print(
                    f"\n[Control error / 控制错误] {error}"
                )

                # Do not attempt return-to-zero on a communication failure.
                # 通信异常时不再尝试回零。
                self.request_stop(
                    reason=(
                        f"Control communication error: {error}"
                        f" / 控制通信异常：{error}"
                    ),
                    return_to_zero=False,
                    emergency=True,
                    wait=False,
                )
                return

            next_tick += period
            sleep_time = next_tick - time.perf_counter()

            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.perf_counter()

    def _send_mit_positions(
        self,
        positions_rad: Sequence[float],
        *,
        lock_timeout: float | None = None,
    ) -> None:
        """Send a set of MIT position commands to all motors.
        向全部电机发送一组 MIT 位置指令。"""

        with self._io_lock_guard(lock_timeout):
            for index, motor in enumerate(self.motors):
                motor_config = self.config.motors[index]

                motor.send_mit(
                    float(positions_rad[index]),
                    0.0,
                    float(motor_config.kp),
                    float(motor_config.kd),
                    0.0,
                )

    # -------------------------------------------------------------------------
    # External control interface / 外部控制接口
    # -------------------------------------------------------------------------

    def set_joint_angles(
        self,
        angles_deg: Sequence[float],
    ) -> None:
        """
        Set the target angles of all joints.

        Example:
            arm.set_joint_angles(
                [50, 0, 0, 0, 0, 0, 0]
            )

        设置全部关节与夹爪目标角度。

        示例：
            arm.set_joint_angles(
                [50, 0, 0, 0, 0, 0, 0]
            )
        """

        if self.shutdown_started:
            return

        if len(angles_deg) != self.motor_count:
            raise ValueError(
                f"Must provide {self.motor_count} angles, "
                f"received {len(angles_deg)}"
                f" / 必须提供 {self.motor_count} 个角度，"
                f"当前收到 {len(angles_deg)} 个"
            )

        positions = []

        for index, angle in enumerate(angles_deg):
            value = float(angle)

            if not math.isfinite(value):
                raise ValueError(
                    f"Invalid angle for joint {index + 1}"
                    f" / 关节 {index + 1} 的角度无效"
                )

            positions.append(math.radians(value))

        with self.target_lock:
            self.target_positions[:] = positions

    def set_joint_angle(
        self,
        joint_id: int,
        angle_deg: float,
    ) -> None:
        """Modify the target angle of a single joint only. / 只修改一个关节的目标角度。"""

        if self.shutdown_started:
            return

        if not 1 <= joint_id <= self.motor_count:
            raise ValueError(
                f"joint_id must be 1-{self.motor_count}"
                f" / joint_id 必须为 1-{self.motor_count}"
            )

        value = float(angle_deg)

        if not math.isfinite(value):
            raise ValueError("Invalid angle_deg / angle_deg 无效")

        with self.target_lock:
            self.target_positions[joint_id - 1] = (
                math.radians(value)
            )

    def set_max_speeds(
        self,
        speeds_deg_s: Sequence[float],
    ) -> None:
        """Set the maximum motion speed of each joint, unit: degrees/second.
        设置各关节的最大运动速度，单位：度/秒。"""

        if len(speeds_deg_s) != self.motor_count:
            raise ValueError(
                f"Must provide {self.motor_count} speeds"
                f" / 必须提供 {self.motor_count} 个速度"
            )

        speeds_rad_s = []

        for index, speed in enumerate(speeds_deg_s):
            value = float(speed)

            if not math.isfinite(value) or value <= 0:
                raise ValueError(
                    f"Speed of joint {index + 1} must be greater than 0"
                    f" / 关节 {index + 1} 的速度必须大于 0"
                )

            speeds_rad_s.append(
                math.radians(value)
            )

        with self.target_lock:
            self.max_speeds_rad_s[:] = speeds_rad_s

    def get_target_angles(self) -> list[float]:
        """Return the final target angles, unit: degrees. / 返回最终目标角度，单位：度。"""

        with self.target_lock:
            values = self.target_positions.copy()

        return [
            math.degrees(value)
            for value in values
        ]

    def get_command_angles(self) -> list[float]:
        """Return the smoothed target angles currently being sent, unit: degrees.
        返回当前实际发送的平滑目标角度，单位：度。"""

        with self.target_lock:
            values = self.command_positions.copy()

        return [
            math.degrees(value)
            for value in values
        ]

    def read_joint_angles(self) -> list[float]:
        """Read the actual mechanical positions once, unit: degrees.
        读取一次实际机械位置，单位：度。"""

        return [
            math.degrees(value)
            for value in self._read_positions_rad()
        ]

    def disable_motors(self) -> None:
        """Disable all motors without closing CAN.
        失能所有电机，但不关闭 CAN 连接。"""

        if self.controller is None:
            return

        with self._io_lock_guard():
            self.controller.disable_all()

        print(
            "[Disable / 失能] All motors disabled / "
            "所有电机已失能"
        )

    # -------------------------------------------------------------------------
    # Position and temperature reading / 位置与温度读取
    # -------------------------------------------------------------------------

    def _read_positions_rad(self) -> list[float]:
        """Read the RobStride mechPos parameter 0x7019. / 读取 RobStride mechPos 参数 0x7019。"""

        positions = []

        with self._io_lock_guard():
            for index, motor in enumerate(self.motors):
                try:
                    position = motor.robstride_get_param_f32(
                        0x7019,
                        timeout_ms=500,
                    )

                    positions.append(float(position))

                except Exception as error:
                    motor_id = (
                        self.config.motors[index].motor_id
                    )

                    raise RuntimeError(
                        f"Failed to read position of motor {motor_id}: "
                        f"{error}"
                        f" / 电机 {motor_id} 位置读取失败："
                        f"{error}"
                    ) from error

        return positions

    @staticmethod
    def _safe_float(value) -> float | None:
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None

        if not math.isfinite(number):
            return None

        return number

    def _read_temperatures_once(
        self,
    ) -> list[float | None]:
        """Request new feedback and read the MOS temperature of each motor.
        请求新反馈并读取每个电机的 MOS 温度。"""

        temperatures: list[float | None] = []

        with self._io_lock_guard():
            for motor in self.motors:
                state = None

                try:
                    motor.request_feedback()
                except Exception:
                    pass

                try:
                    if self.controller is not None:
                        self.controller.poll_feedback_once()
                except Exception:
                    pass

                try:
                    state = motor.get_state()
                except Exception:
                    state = None

                if state is None:
                    temperatures.append(None)
                    continue

                temperatures.append(
                    self._safe_float(
                        getattr(state, "t_mos", None)
                    )
                )

        self.last_temperatures = temperatures
        return temperatures

    def _report_temperature_alerts(
        self,
        temperatures: list[float | None],
    ) -> None:
        """Log temperature alarms once per over-threshold episode per motor.
        每个电机每次超温只报警一次。"""

        hysteresis = self._TEMP_ALARM_HYSTERESIS_C

        for index, temp in enumerate(temperatures):
            if temp is None:
                continue

            motor_id = self.config.motors[index].motor_id

            if temp >= self.temp_alarm_c:
                if not self._temp_alarm_active[index]:
                    self._temp_alarm_active[index] = True

                    print(
                        "\n[Temp alarm / 温度报警] "
                        f"Motor {motor_id} MOS="
                        f"{temp:.1f}°C, "
                        "arm keeps running / "
                        f"电机 {motor_id} MOS="
                        f"{temp:.1f}°C，"
                        "机械臂继续运行"
                    )

            elif temp < self.temp_alarm_c - hysteresis:
                self._temp_alarm_active[index] = False

    def _temperature_loop(self) -> None:
        """Execute the three-level temperature protection. / 执行三级温度保护。"""

        period = 1.0 / self.telemetry_hz

        while not self.worker_stop_event.wait(period):
            try:
                temperatures = self._read_temperatures_once()

            except Exception as error:
                print(
                    f"\n[Temp / 温度] Feedback read failed: {error}"
                    f" / 反馈读取失败：{error}"
                )
                continue

            valid_temperatures = [
                (index, temp)
                for index, temp in enumerate(temperatures)
                if temp is not None
            ]

            if not valid_temperatures:
                continue

            hottest_index, hottest_temp = max(
                valid_temperatures,
                key=lambda item: item[1],
            )

            hottest_motor_id = self.config.motors[
                hottest_index
            ].motor_id

            if hottest_temp >= self.temp_disconnect_c:
                print(
                    "\n[Emergency disconnect / 紧急断开] "
                    f"Motor {hottest_motor_id} MOS="
                    f"{hottest_temp:.1f}°C, "
                    "disabling immediately, no return-to-zero / "
                    f"电机 {hottest_motor_id} MOS="
                    f"{hottest_temp:.1f}°C，"
                    "立即失能，不再回零"
                )

                self.request_stop(
                    reason=(
                        f"Motor {hottest_motor_id} "
                        "reached the emergency disconnect temperature"
                        f" / 电机 {hottest_motor_id} "
                        "达到紧急断开温度"
                    ),
                    return_to_zero=False,
                    emergency=True,
                    wait=False,
                )
                return

            if hottest_temp >= self.temp_return_zero_c:
                print(
                    "\n[High-temp return / 高温回零] "
                    f"Motor {hottest_motor_id} MOS="
                    f"{hottest_temp:.1f}°C, "
                    "stopping motion and slowly returning to zero / "
                    f"电机 {hottest_motor_id} MOS="
                    f"{hottest_temp:.1f}°C，"
                    "停止运动并缓慢回零"
                )

                self.request_stop(
                    reason=(
                        f"Motor {hottest_motor_id} "
                        "reached the return-to-zero temperature"
                        f" / 电机 {hottest_motor_id} "
                        "达到回零温度"
                    ),
                    return_to_zero=True,
                    thermal=True,
                    wait=False,
                )
                return

            self._report_temperature_alerts(temperatures)

    # -------------------------------------------------------------------------
    # Esc and Ctrl+C / Esc 和 Ctrl+C
    # -------------------------------------------------------------------------

    def _start_keyboard_listener(self) -> None:
        if keyboard is None:
            print(
                "[Keyboard / 键盘] pynput is not installed or unavailable, "
                "Esc disabled; Ctrl+C and stop() still work / "
                "未安装或无法使用 pynput，"
                "Esc 功能关闭；Ctrl+C 和 stop() 仍然有效"
            )
            return

        try:
            self.keyboard_listener = keyboard.Listener(
                on_press=self._on_key_press
            )
            self.keyboard_listener.start()

        except Exception as error:
            self.keyboard_listener = None

            print(
                f"[Keyboard / 键盘] Failed to start Esc listener: {error}"
                f" / Esc 监听启动失败：{error}"
            )

    def _on_key_press(self, key) -> bool | None:
        if keyboard is not None and key == keyboard.Key.esc:
            print(
                "\n[Exit / 退出] Esc detected, "
                "slowly returning to zero / "
                "检测到 Esc，"
                "正在缓慢回到零点"
            )

            self.request_stop(
                reason="User pressed Esc / 用户按下 Esc",
                return_to_zero=True,
                wait=False,
            )
            return False

        return None

    def _signal_handler(self, signum, _frame) -> None:
        self.signal_count += 1

        if self.signal_count == 1:
            print(
                f"\n[Exit / 退出] Received signal {signum}, "
                "slowly returning to zero / "
                f"收到信号 {signum}，"
                "正在缓慢回到零点"
            )

            self.request_stop(
                reason=f"Received signal {signum} / 收到信号 {signum}",
                return_to_zero=True,
                wait=False,
            )

        else:
            print(
                "\n[Exit / 退出] Second Ctrl+C received, "
                "aborting return-to-zero and disabling immediately / "
                "第二次收到 Ctrl+C，"
                "立即中止回零并失能"
            )

            self.abort_return_event.set()
            self.worker_stop_event.set()

    # -------------------------------------------------------------------------
    # Safe stop and return-to-zero / 安全停止与回零
    # -------------------------------------------------------------------------

    @staticmethod
    def _smoothstep(alpha: float) -> float:
        alpha = max(0.0, min(1.0, alpha))
        return alpha * alpha * (3.0 - 2.0 * alpha)

    def request_stop(
        self,
        *,
        reason: str = "stop() called / 调用 stop()",
        return_to_zero: bool = True,
        thermal: bool = False,
        emergency: bool = False,
        wait: bool = False,
    ) -> None:
        """Request a stop. The first call creates the safe shutdown thread.
        请求停止。第一次调用会创建安全停止线程。"""

        with self.shutdown_lock:
            if not self.shutdown_started:
                self.shutdown_started = True
                self.shutdown_reason = reason

                self.return_to_zero_requested = bool(
                    return_to_zero
                )
                self.thermal_return_requested = bool(
                    thermal
                )
                self.emergency_disconnect_requested = bool(
                    emergency
                )

                self.shutdown_thread = threading.Thread(
                    target=self._shutdown_worker,
                    name="rebot-rs-safe-shutdown",
                    daemon=True,
                )
                self.shutdown_thread.start()

        if (
            wait
            and self.shutdown_thread is not None
            and threading.current_thread()
            is not self.shutdown_thread
        ):
            self.shutdown_done_event.wait()

    def stop(
        self,
        *,
        return_to_zero: bool = True,
        wait: bool = True,
    ) -> None:
        """
        Stop interface called by external programs.

        Default:
            slowly return to zero -> disable -> close CAN

        外部程序调用的停止接口。

        默认：
            缓慢回零 -> 失能 -> 关闭 CAN
        """

        self.request_stop(
            reason="Program called arm.stop() / 程序调用 arm.stop()",
            return_to_zero=return_to_zero,
            wait=wait,
        )

    def _shutdown_worker(self) -> None:
        print(
            f"\n[Stop / 停止] Reason: {self.shutdown_reason}"
            f" / 原因：{self.shutdown_reason}"
        )

        # First stop the normal MIT control and temperature threads.
        # 先停止正常 MIT 控制和温度线程。
        self.worker_stop_event.set()

        current_thread = threading.current_thread()

        for thread_name, thread in (
            ("control", self.control_thread),
            ("temperature", self.telemetry_thread),
        ):
            if (
                thread is not None
                and thread is not current_thread
                and thread.is_alive()
            ):
                thread.join(timeout=3.0)

                if thread.is_alive():
                    print(
                        f"\n[Stop / 停止] {thread_name} thread did not exit "
                        "within 3 s (CAN I/O may be stuck); "
                        "continuing shutdown / "
                        f"{thread_name} 线程 3 秒内未退出"
                        "（CAN I/O 可能卡住）；"
                        "继续关机"
                    )

        if self.keyboard_listener is not None:
            try:
                self.keyboard_listener.stop()
            except Exception:
                pass

        try:
            if (
                self.return_to_zero_requested
                and not self.emergency_disconnect_requested
                and self.controller is not None
                and self.motors
            ):
                self._safe_return_to_zero()

        except Exception as error:
            print(
                f"\n[Return-to-zero error / 回零错误] {error}"
            )

        finally:
            self._disable_and_close()

            self.started = False
            self.shutdown_done_event.set()

            print("[Stop / 停止] Safe stop sequence complete / 安全停止流程完成")

    def _safe_return_to_zero(self) -> None:
        """Use a smoothstep trajectory to slowly return all joints to zero.
        使用 smoothstep 轨迹缓慢返回全部关节零点。"""

        try:
            start_positions = self._read_positions_rad()

        except Exception as error:
            print(
                "[Return-to-zero / 回零] Cannot read actual angles, "
                f"using current command angles instead: {error}"
                " / 无法读取实际角度，"
                f"改用当前发送角度：{error}"
            )

            with self.target_lock:
                start_positions = (
                    self.command_positions.copy()
                )

        target_positions = [
            0.0 for _ in self.config.motors
        ]

        return_zero_config = self.config.return_zero

        speed_deg_s = (
            return_zero_config.thermal_max_speed_deg_s
            if self.thermal_return_requested
            else return_zero_config.max_speed_deg_s
        )

        # The maximum slope of smoothstep is 1.5. / smoothstep 的最大斜率为 1.5。
        required_times = [
            (
                1.5
                * abs(math.degrees(position))
                / speed_deg_s
            )
            for position in start_positions
        ]

        duration_s = max(
            return_zero_config.min_time_s,
            *required_times,
        )

        steps = max(
            2,
            int(round(duration_s * self.control_hz)),
        )

        print(
            "[Return-to-zero / 回零] Slowly returning to zero: "
            f"estimated {duration_s:.1f} s, "
            f"peak speed no more than {speed_deg_s:.1f}°/s"
            " / 正在缓慢回零："
            f"预计 {duration_s:.1f} 秒，"
            f"峰值速度不超过 {speed_deg_s:.1f}°/s"
        )
        print(
            "[Return-to-zero / 回零] Press Ctrl+C again to abort immediately"
            " / 再次按 Ctrl+C 可立即中止回零"
        )

        period = 1.0 / self.control_hz
        next_tick = time.perf_counter()
        last_thermal_check = -float("inf")

        completed = True

        for step_index in range(1, steps + 1):
            if self.abort_return_event.is_set():
                print(
                    "\n[Return-to-zero / 回零] Return-to-zero aborted by user"
                    " / 用户中止回零"
                )
                completed = False
                break

            now = time.monotonic()

            # Keep checking the emergency disconnect threshold during a thermal return-to-zero.
            # 高温回零过程中继续检查紧急断开阈值。
            if (
                self.thermal_return_requested
                and now - last_thermal_check
                >= 1.0 / self.telemetry_hz
            ):
                last_thermal_check = now

                temperatures = (
                    self._read_temperatures_once()
                )

                valid_temperatures = [
                    (index, temp)
                    for index, temp
                    in enumerate(temperatures)
                    if temp is not None
                ]

                if valid_temperatures:
                    hottest_index, hottest_temp = max(
                        valid_temperatures,
                        key=lambda item: item[1],
                    )

                    if (
                        hottest_temp
                        >= self.temp_disconnect_c
                    ):
                        motor_id = self.config.motors[
                            hottest_index
                        ].motor_id

                        print(
                            "\n[Emergency disconnect / 紧急断开] During return-to-zero "
                            f"motor {motor_id} MOS="
                            f"{hottest_temp:.1f}°C, "
                            "aborting return-to-zero immediately / "
                            "回零过程中 "
                            f"电机 {motor_id} MOS="
                            f"{hottest_temp:.1f}°C，"
                            "立即中止回零"
                        )
                        completed = False
                        break

            alpha = self._smoothstep(
                step_index / steps
            )

            commands = [
                start
                + alpha * (target - start)
                for start, target in zip(
                    start_positions,
                    target_positions,
                )
            ]

            try:
                self._send_mit_positions(
                    commands,
                    lock_timeout=self._IO_LOCK_SHUTDOWN_TIMEOUT_S,
                )
            except TimeoutError as error:
                print(
                    "\n[Return-to-zero / 回零] I/O lock timeout, "
                    f"aborting return-to-zero: {error}"
                    " / I/O 锁超时，"
                    f"中止回零：{error}"
                )
                completed = False
                break

            with self.target_lock:
                self.command_positions[:] = commands
                self.target_positions[:] = commands

            next_tick += period
            sleep_time = next_tick - time.perf_counter()

            if sleep_time > 0:
                time.sleep(sleep_time)

        if completed:
            try:
                self._send_mit_positions(
                    target_positions,
                    lock_timeout=self._IO_LOCK_SHUTDOWN_TIMEOUT_S,
                )
            except TimeoutError as error:
                print(
                    "\n[Return-to-zero / 回零] I/O lock timeout at final zero: "
                    f"{error}"
                    " / 发送最终零点时 I/O 锁超时："
                    f"{error}"
                )
                completed = False

        if completed:
            with self.target_lock:
                self.command_positions[:] = target_positions
                self.target_positions[:] = target_positions

            time.sleep(
                self.config.return_zero.settle_time_s
            )

            print(
                "[Return-to-zero / 回零] Zero sent and hold complete, "
                "ready to disable / "
                "已发送零点并完成保持，"
                "准备失能"
            )

    # -------------------------------------------------------------------------
    # Resource cleanup / 资源清理
    # -------------------------------------------------------------------------

    def _disable_and_close(self) -> None:
        """Disable the motors and close MotorBridge resources.
        失能电机并关闭 MotorBridge 资源。"""

        if self.controller is not None:
            acquired = self._try_acquire_io_lock(
                self._IO_LOCK_SHUTDOWN_TIMEOUT_S
            )

            if acquired:
                try:
                    self.controller.disable_all()
                    print(
                        "[Disable / 失能] All motors disabled / "
                        "所有电机已失能"
                    )
                except Exception as error:
                    print(
                        f"[Disable / 失能] disable_all failed: {error}"
                        f" / disable_all 失败：{error}"
                    )
                finally:
                    self.io_lock.release()
            else:
                print(
                    "[Disable / 失能] io_lock unavailable "
                    "(communication may be stuck), "
                    "skipping disable_all / "
                    "io_lock 不可用（通信可能卡住），"
                    "跳过 disable_all"
                )

        for motor in self.motors:
            try:
                motor.close()
            except Exception:
                pass

        self.motors.clear()

        if self.controller is not None:
            try:
                self.controller.shutdown()
            except Exception:
                pass

            try:
                self.controller.close()
            except Exception:
                pass

        self.controller = None

    @property
    def is_stopped(self) -> bool:
        return self.shutdown_done_event.is_set()

    def wait_until_stopped(
        self,
        timeout: float | None = None,
    ) -> bool:
        return self.shutdown_done_event.wait(timeout)
