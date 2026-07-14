#!/usr/bin/env python3
"""Safe ZMQ position controller for an SO100/SO101 follower arm.

The robot process owns the serial bus. ZMQ clients send desired joint positions;
this process clamps them to calibrated limits and applies velocity- and
acceleration-limited trajectories before writing motor goals.
"""

from __future__ import annotations

import argparse
import math
import signal
import time
import traceback
from dataclasses import dataclass
from typing import Any

import zmq

from lerobot.motors import MotorNormMode
from lerobot.robots.so_follower.config_so_follower import SO100FollowerConfig
from lerobot.robots.so_follower.so_follower import SO100Follower

# ---------------------------------------------------------------------------
# USER-TUNABLE SAFETY DEFAULTS
#
# Before operating a different arm, payload, voltage, or mounting orientation,
# review DEFAULT_MAX_SPEED and DEFAULT_MAX_ACCELERATION below. Start lower than
# these values and increase only after unloaded testing. Temperature/current
# thresholds are CLI options; run this file with --help. Only change
# CURRENT_AMPS_PER_RAW_UNIT after checking the motor model and firmware manual.
# ---------------------------------------------------------------------------
JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)

DEFAULT_MAX_SPEED = {
    "shoulder_pan": 25.0,
    "shoulder_lift": 20.0,
    "elbow_flex": 25.0,
    "wrist_flex": 30.0,
    "wrist_roll": 40.0,
    "gripper": 30.0,
}
DEFAULT_MAX_SPEED_SCALAR = 0.7
DEFAULT_MAX_SPEED = {joint: speed * DEFAULT_MAX_SPEED_SCALAR for joint, speed in DEFAULT_MAX_SPEED.items()}

DEFAULT_MAX_ACCELERATION = {
    "shoulder_pan": 60.0,
    "shoulder_lift": 50.0,
    "elbow_flex": 60.0,
    "wrist_flex": 80.0,
    "wrist_roll": 100.0,
    "gripper": 80.0,
}
DEFAULT_MAX_ACCELERATION_SCALAR = 0.7
DEFAULT_MAX_ACCELERATION = {
    joint: accel * DEFAULT_MAX_ACCELERATION_SCALAR for joint, accel in DEFAULT_MAX_ACCELERATION.items()
}

CURRENT_AMPS_PER_RAW_UNIT = 0.0065


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def current_magnitude(raw: int | float | None) -> int | None:
    """Decode the magnitude of the STS3215 sign-magnitude current register."""
    if raw is None:
        return None
    return int(raw) & 0x7FFF


@dataclass
class TrajectoryLimiter:
    limits: dict[str, tuple[float, float]]
    max_speed: dict[str, float]
    max_acceleration: dict[str, float]
    commanded: dict[str, float]
    target: dict[str, float]
    velocity: dict[str, float]

    @classmethod
    def from_positions(
        cls,
        positions: dict[str, float],
        limits: dict[str, tuple[float, float]],
        max_speed: dict[str, float],
        max_acceleration: dict[str, float],
    ) -> "TrajectoryLimiter":
        commanded = {
            joint: clamp(float(positions[joint]), *limits[joint])
            for joint in JOINTS
        }
        return cls(
            limits=limits,
            max_speed=max_speed,
            max_acceleration=max_acceleration,
            commanded=commanded,
            target=dict(commanded),
            velocity=dict.fromkeys(JOINTS, 0.0),
        )

    def set_targets(self, requested: dict[str, Any]) -> dict[str, float]:
        accepted: dict[str, float] = {}
        for joint, value in requested.items():
            if joint not in self.target:
                raise ValueError(f"Unknown joint: {joint}")
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"Position for {joint} must be numeric")
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f"Position for {joint} must be finite")
            accepted[joint] = clamp(value, *self.limits[joint])
        if not accepted:
            raise ValueError("No joint positions supplied")
        self.target.update(accepted)
        return accepted

    def hold(self) -> None:
        self.target = dict(self.commanded)

    def reset(self, positions: dict[str, float]) -> None:
        for joint in JOINTS:
            position = clamp(float(positions[joint]), *self.limits[joint])
            self.commanded[joint] = position
            self.target[joint] = position
            self.velocity[joint] = 0.0

    def step(self, dt: float) -> dict[str, float]:
        dt = clamp(dt, 0.001, 0.1)
        for joint in JOINTS:
            position = self.commanded[joint]
            error = self.target[joint] - position
            acceleration = self.max_acceleration[joint]

            if abs(error) < 1e-4 and abs(self.velocity[joint]) < 1e-3:
                self.commanded[joint] = self.target[joint]
                self.velocity[joint] = 0.0
                continue

            braking_speed = math.sqrt(max(0.0, 2.0 * acceleration * abs(error)))
            desired_velocity = math.copysign(
                min(self.max_speed[joint], braking_speed), error
            ) if error else 0.0
            velocity_delta = clamp(
                desired_velocity - self.velocity[joint],
                -acceleration * dt,
                acceleration * dt,
            )
            velocity = self.velocity[joint] + velocity_delta
            next_position = position + velocity * dt

            if (error > 0 and next_position >= self.target[joint]) or (
                error < 0 and next_position <= self.target[joint]
            ):
                next_position = self.target[joint]
                velocity = 0.0

            low, high = self.limits[joint]
            bounded = clamp(next_position, low, high)
            if bounded != next_position:
                velocity = 0.0
            self.commanded[joint] = bounded
            self.velocity[joint] = velocity

        return dict(self.commanded)


class SafeSOFollowerServer:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        config = SO100FollowerConfig(port=args.port, id=args.robot_id)
        self.robot = SO100Follower(config)
        self.running = True
        self.control_enabled = False
        self.fault: str | None = None
        self.warnings: list[str] = []
        self.last_command_time: float | None = None
        self.watchdog_active = False
        self.loop_hz = 0.0
        self.communication_errors = 0
        self.overcurrent_since = dict.fromkeys(JOINTS)
        self.positions: dict[str, float] = dict.fromkeys(JOINTS, 0.0)
        self.active_joints = tuple(
            joint for joint in JOINTS if args.enable_gripper or joint != "gripper"
        )
        self.telemetry = {
            joint: {
                "position": None,
                "velocity_raw": None,
                "load_raw": None,
                "voltage_v": None,
                "temperature_c": None,
                "current_raw": None,
                "current_a": None,
                "moving": None,
                "status": None,
            }
            for joint in JOINTS
        }
        self.limits: dict[str, tuple[float, float]] = {}
        self.limiter: TrajectoryLimiter | None = None

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.MAXMSGSIZE, 64 * 1024)
        self.socket.bind(args.bind)

    def derive_limits(self) -> dict[str, tuple[float, float]]:
        limits: dict[str, tuple[float, float]] = {}
        for joint, motor in self.robot.bus.motors.items():
            calibration = self.robot.calibration[joint]
            if motor.norm_mode is MotorNormMode.RANGE_0_100:
                low, high = 0.0, 100.0
                margin = min(self.args.limit_margin, 10.0)
            elif motor.norm_mode is MotorNormMode.DEGREES:
                resolution = self.robot.bus.model_resolution_table[motor.model] - 1
                half_range = (calibration.range_max - calibration.range_min) * 180.0 / resolution
                low, high = -half_range, half_range
                margin = min(self.args.limit_margin, max(0.0, half_range * 0.1))
            else:
                low, high = -100.0, 100.0
                margin = min(self.args.limit_margin, 10.0)
            if high - low <= 2 * margin:
                margin = 0.0
            limits[joint] = (low + margin, high - margin)
        return limits

    def read_positions(self) -> dict[str, float]:
        observation = self.robot.get_observation()
        positions = {
            key.removesuffix(".pos"): float(value)
            for key, value in observation.items()
            if key.endswith(".pos")
        }
        if set(positions) != set(JOINTS):
            raise RuntimeError(f"Incomplete position response: {sorted(positions)}")
        self.positions = positions
        for joint, value in positions.items():
            self.telemetry[joint]["position"] = value
        return positions

    def _sync_read_optional(self, register: str) -> dict[str, int] | None:
        try:
            values = self.robot.bus.sync_read(register, normalize=False, num_retry=1)
            return {joint: int(value) for joint, value in values.items()}
        except Exception as exc:
            self.warnings.append(f"{register}: {exc}")
            return None

    def read_extended_telemetry(self, now: float) -> None:
        self.warnings = []
        registers = {
            "Present_Velocity": "velocity_raw",
            "Present_Load": "load_raw",
            "Present_Voltage": "voltage_raw",
            "Present_Temperature": "temperature_c",
            "Present_Current": "current_raw",
            "Moving": "moving",
            "Status": "status",
        }
        readings = {register: self._sync_read_optional(register) for register in registers}

        for joint in JOINTS:
            for register, field in registers.items():
                values = readings[register]
                self.telemetry[joint][field] = values[joint] if values is not None else None

            voltage_raw = self.telemetry[joint].pop("voltage_raw", None)
            self.telemetry[joint]["voltage_v"] = (
                float(voltage_raw) / 10.0 if voltage_raw is not None else None
            )

            magnitude = current_magnitude(self.telemetry[joint]["current_raw"])
            self.telemetry[joint]["current_a"] = (
                magnitude * CURRENT_AMPS_PER_RAW_UNIT if magnitude is not None else None
            )

            temperature = self.telemetry[joint]["temperature_c"]
            if temperature is not None and temperature >= self.args.max_temperature:
                self.trip_fault(f"Over-temperature on {joint}: {temperature} C")
                return
            if temperature is not None and temperature >= self.args.warn_temperature:
                self.warnings.append(f"High temperature on {joint}: {temperature} C")

            if magnitude is not None and magnitude >= self.args.max_current_raw:
                if self.overcurrent_since[joint] is None:
                    self.overcurrent_since[joint] = now
                elif now - float(self.overcurrent_since[joint]) >= self.args.overcurrent_seconds:
                    self.trip_fault(
                        f"Over-current on {joint}: {magnitude} raw "
                        f"(~{magnitude * CURRENT_AMPS_PER_RAW_UNIT:.2f} A)"
                    )
                    return
            else:
                self.overcurrent_since[joint] = None

    def trip_fault(self, reason: str) -> None:
        if self.fault is not None:
            return
        self.fault = reason
        self.control_enabled = False
        if self.limiter is not None:
            self.limiter.hold()
        try:
            self.robot.bus.disable_torque()
        except Exception as exc:
            self.warnings.append(f"Could not disable torque: {exc}")
        print(f"SAFETY FAULT: {reason}")

    def reset_fault(self) -> None:
        self.read_positions()
        self.read_extended_telemetry(time.monotonic())
        if any(
            data["temperature_c"] is None or data["temperature_c"] >= self.args.warn_temperature
            for data in self.telemetry.values()
        ):
            raise RuntimeError("Cannot reset: temperature is unavailable or still high")
        if any(
            current_magnitude(data["current_raw"]) is None
            or current_magnitude(data["current_raw"]) >= self.args.max_current_raw
            for data in self.telemetry.values()
        ):
            raise RuntimeError("Cannot reset: current is unavailable or still high")
        self.robot.bus.enable_torque(list(self.active_joints))
        if not self.args.enable_gripper:
            self.robot.bus.disable_torque("gripper")
        assert self.limiter is not None
        self.limiter.reset(self.positions)
        self.fault = None
        self.control_enabled = False
        self.last_command_time = None
        self.watchdog_active = False
        self.overcurrent_since = dict.fromkeys(JOINTS)

    def handle_request(self, request: Any, now: float) -> dict[str, Any]:
        if not isinstance(request, dict):
            raise ValueError("Request must be a JSON object")
        request_type = request.get("type", "get_state")

        if request_type == "get_state":
            return {"ok": True}
        if request_type == "set_targets":
            if self.fault:
                raise RuntimeError(f"Controller is faulted: {self.fault}")
            positions = request.get("positions")
            if not isinstance(positions, dict):
                raise ValueError("positions must be a JSON object")
            assert self.limiter is not None
            ignored = []
            if not self.args.enable_gripper and "gripper" in positions:
                positions = dict(positions)
                positions.pop("gripper")
                ignored.append("gripper")
            if not positions:
                raise ValueError("Gripper is disabled; no enabled joint targets were supplied")
            accepted = self.limiter.set_targets(positions)
            self.last_command_time = now
            self.watchdog_active = False
            self.control_enabled = True
            return {"ok": True, "accepted": accepted, "ignored_disabled_joints": ignored}
        if request_type == "hold":
            assert self.limiter is not None
            self.limiter.hold()
            self.last_command_time = now
            return {"ok": True}
        if request_type == "estop":
            self.trip_fault("Remote emergency stop")
            return {"ok": True}
        if request_type == "reset_fault":
            self.reset_fault()
            return {"ok": True}
        raise ValueError(f"Unknown request type: {request_type}")

    def state_snapshot(self, now: float) -> dict[str, Any]:
        assert self.limiter is not None
        command_age = None if self.last_command_time is None else now - self.last_command_time
        return {
            "server_time": time.time(),
            "port": self.args.port,
            "bind": self.args.bind,
            "control_enabled": self.control_enabled,
            "active_joints": list(self.active_joints),
            "gripper_enabled": self.args.enable_gripper,
            "fault": self.fault,
            "warnings": self.warnings[-12:],
            "watchdog_active": self.watchdog_active,
            "command_age_s": command_age,
            "watchdog_timeout_s": self.args.watchdog_timeout,
            "loop_hz": self.loop_hz,
            "communication_errors": self.communication_errors,
            "limits": {joint: list(values) for joint, values in self.limits.items()},
            "max_speed": self.limiter.max_speed,
            "max_acceleration": self.limiter.max_acceleration,
            "target": self.limiter.target,
            "commanded": self.limiter.commanded,
            "trajectory_velocity": self.limiter.velocity,
            "joints": self.telemetry,
            "current_scale_a_per_raw": CURRENT_AMPS_PER_RAW_UNIT,
            "safety": {
                "warn_temperature_c": self.args.warn_temperature,
                "max_temperature_c": self.args.max_temperature,
                "max_current_raw": self.args.max_current_raw,
                "overcurrent_seconds": self.args.overcurrent_seconds,
                "limit_margin": self.args.limit_margin,
            },
        }

    def process_zmq(self, now: float) -> None:
        try:
            request = self.socket.recv_json(flags=zmq.NOBLOCK)
        except zmq.Again:
            return
        except Exception as exc:
            self.socket.send_json(
                {"ok": False, "error": f"Invalid JSON request: {exc}", "state": self.state_snapshot(now)}
            )
            return
        try:
            response = self.handle_request(request, now)
        except Exception as exc:
            response = {"ok": False, "error": str(exc)}
        response["state"] = self.state_snapshot(now)
        self.socket.send_json(response)

    def run(self) -> None:
        print(f"Connecting to SO follower on {self.args.port}...")
        self.robot.connect(calibrate=not self.args.no_auto_calibrate)
        if not self.args.enable_gripper:
            self.robot.bus.disable_torque("gripper")
            print("Gripper motor is disabled and torque is OFF (use --enable-gripper to enable it).")
        positions = self.read_positions()
        self.limits = self.derive_limits()
        self.limiter = TrajectoryLimiter.from_positions(
            positions, self.limits, DEFAULT_MAX_SPEED, DEFAULT_MAX_ACCELERATION
        )
        print(f"ZMQ controller listening on {self.args.bind}")
        print("No motion is sent until the first valid set_targets request.")
        print(f"Calibrated software limits: {self.limits}")
        print(
            "Tune DEFAULT_MAX_SPEED and DEFAULT_MAX_ACCELERATION near the top "
            "of this file for your payload and hardware; begin with lower values."
        )

        control_period = 1.0 / self.args.control_rate
        telemetry_period = 1.0 / self.args.telemetry_rate
        last_control = time.monotonic()
        next_control = last_control
        next_telemetry = last_control
        loop_counter = 0
        rate_started = last_control

        while self.running:
            now = time.monotonic()
            self.process_zmq(now)

            if now >= next_control:
                dt = now - last_control
                last_control = now
                next_control = now + control_period

                cycle_error: Exception | None = None
                try:
                    self.read_positions()
                except Exception as exc:
                    cycle_error = exc
                    self.warnings.append(f"Position read: {exc}")

                assert self.limiter is not None
                if (
                    self.control_enabled
                    and self.last_command_time is not None
                    and now - self.last_command_time > self.args.watchdog_timeout
                ):
                    self.limiter.hold()
                    self.watchdog_active = True

                command = self.limiter.step(dt)
                if self.control_enabled and self.fault is None:
                    try:
                        self.robot.send_action(
                            {
                                f"{joint}.pos": command[joint]
                                for joint in self.active_joints
                            }
                        )
                    except Exception as exc:
                        cycle_error = exc
                        self.warnings.append(f"Command write: {exc}")

                if cycle_error is None:
                    self.communication_errors = 0
                else:
                    self.communication_errors += 1
                    if self.communication_errors >= self.args.max_communication_errors:
                        self.trip_fault(f"Repeated serial communication errors: {cycle_error}")

                loop_counter += 1
                elapsed = now - rate_started
                if elapsed >= 1.0:
                    self.loop_hz = loop_counter / elapsed
                    loop_counter = 0
                    rate_started = now

            if now >= next_telemetry:
                next_telemetry = now + telemetry_period
                try:
                    self.read_extended_telemetry(now)
                except Exception as exc:
                    self.warnings.append(f"Telemetry: {exc}")

            time.sleep(0.001)

    def close(self) -> None:
        self.running = False
        try:
            if self.robot.is_connected:
                self.robot.disconnect()
        finally:
            self.socket.close(0)
            self.context.term()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0", help="Feetech serial port")
    parser.add_argument("--robot-id", default=None, help="LeRobot calibration ID")
    parser.add_argument("--bind", default="tcp://127.0.0.1:5555", help="ZMQ REP bind endpoint")
    parser.add_argument(
        "--enable-gripper",
        action="store_true",
        help="Enable gripper commands and torque (disabled by default)",
    )
    parser.add_argument("--control-rate", type=float, default=30.0, help="Control loop frequency in Hz")
    parser.add_argument("--telemetry-rate", type=float, default=5.0, help="Extended telemetry frequency in Hz")
    parser.add_argument("--watchdog-timeout", type=float, default=0.5, help="Seconds without targets before hold")
    parser.add_argument("--limit-margin", type=float, default=2.0, help="Inset from calibrated position limits")
    parser.add_argument("--warn-temperature", type=float, default=55.0, help="Temperature warning in C")
    parser.add_argument("--max-temperature", type=float, default=65.0, help="Temperature torque-off threshold in C")
    parser.add_argument("--max-current-raw", type=int, default=500, help="Current torque-off threshold (6.5 mA/raw)")
    parser.add_argument("--overcurrent-seconds", type=float, default=0.5, help="Time above current threshold before fault")
    parser.add_argument("--max-communication-errors", type=int, default=3)
    parser.add_argument("--no-auto-calibrate", action="store_true", help="Fail instead of starting calibration")
    args = parser.parse_args()
    if args.control_rate <= 0 or args.telemetry_rate <= 0:
        parser.error("control and telemetry rates must be positive")
    if args.watchdog_timeout <= 0:
        parser.error("watchdog timeout must be positive")
    return args


def main() -> None:
    args = parse_args()
    server = SafeSOFollowerServer(args)

    def stop_handler(_signum, _frame):
        server.running = False

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    try:
        server.run()
    except Exception:
        traceback.print_exc()
    finally:
        server.close()


if __name__ == "__main__":
    main()
