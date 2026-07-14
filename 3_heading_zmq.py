#!/usr/bin/env python3
"""Single-slider keypose client for 2_s100_keyboard_joint_control_zmq.py.

The shoulder-pan slider selects a piecewise-linear interpolation of KEYPOSES.
The first keypose has the minimum pan angle and the last has the maximum.
"""

# Copyright 2026 Abhijeet Kulkarni.
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

import argparse
import bisect
import math
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Any

import zmq

# USER-TUNABLE KEYPOSES
#
# Replace these examples with poses tested on your arm. Keyposes must have the
# same joints and strictly increasing shoulder_pan values. With three or more
# entries, interpolation is piecewise between adjacent keyposes. Omitted joints
# retain their previous server target. Add "gripper" to every keypose only when
# the server is intentionally started with --enable-gripper.
KEYPOSES: list[dict[str, float]] = [
    {
        "shoulder_pan": -90.0,
        "shoulder_lift": 0.0,
        "elbow_flex": 0.0,
        "wrist_flex": 0.0,
        "wrist_roll": 0.0,
    },
    {
        "shoulder_pan": 0.0,
        "shoulder_lift": 0.0,
        "elbow_flex": 0.0,
        "wrist_flex": 0.0,
        "wrist_roll": 0.0,
    },
    {
        "shoulder_pan": 90.0,
        "shoulder_lift": 0.0,
        "elbow_flex": 0.0,
        "wrist_flex": 0.0,
        "wrist_roll": 0.0,
    },
]


def validate_keyposes(keyposes: list[dict[str, float]]) -> tuple[tuple[str, ...], tuple[float, ...]]:
    """Validate keypose shape and return joint names and ordered pan values."""
    if len(keyposes) < 2:
        raise ValueError("KEYPOSES must contain at least two poses")
    if not isinstance(keyposes[0], dict) or "shoulder_pan" not in keyposes[0]:
        raise ValueError("Every keypose must contain shoulder_pan")

    joints = tuple(keyposes[0])
    joint_set = set(joints)
    pan_values: list[float] = []
    for index, pose in enumerate(keyposes, start=1):
        if not isinstance(pose, dict) or set(pose) != joint_set:
            raise ValueError(f"Keypose {index} must contain exactly these joints: {joints}")
        for joint, value in pose.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"Keypose {index} value for {joint} must be numeric")
            if not math.isfinite(float(value)):
                raise ValueError(f"Keypose {index} value for {joint} must be finite")
        pan_values.append(float(pose["shoulder_pan"]))

    if any(right <= left for left, right in zip(pan_values, pan_values[1:])):
        raise ValueError("KEYPOSES shoulder_pan values must be strictly increasing")
    return joints, tuple(pan_values)


def interpolate_keyposes(
    keyposes: list[dict[str, float]], pan_values: tuple[float, ...], desired_pan: float
) -> tuple[dict[str, float], int, float]:
    """Return the piecewise-linear pose, left segment index, and interpolation fraction."""
    pan = max(pan_values[0], min(pan_values[-1], float(desired_pan)))
    if pan <= pan_values[0]:
        return {joint: float(value) for joint, value in keyposes[0].items()}, 0, 0.0
    if pan >= pan_values[-1]:
        last_segment = len(keyposes) - 2
        return {joint: float(value) for joint, value in keyposes[-1].items()}, last_segment, 1.0

    right = bisect.bisect_right(pan_values, pan)
    left = right - 1
    fraction = (pan - pan_values[left]) / (pan_values[right] - pan_values[left])
    pose = {
        joint: float(keyposes[left][joint])
        + fraction * (float(keyposes[right][joint]) - float(keyposes[left][joint]))
        for joint in keyposes[left]
    }
    return pose, left, fraction


class HeadingGUI:
    def __init__(self, root: tk.Tk, endpoint: str, poll_ms: int):
        self.root = root
        self.endpoint = endpoint
        self.poll_ms = poll_ms
        self.joints, self.pan_values = validate_keyposes(KEYPOSES)
        self.context = zmq.Context()
        self.socket: zmq.Socket | None = None
        self.streaming = False
        self.connected = False
        self.effective_limits = (self.pan_values[0], self.pan_values[-1])

        self.pan_var = tk.DoubleVar(value=self.pan_values[0])
        self.pan_text = tk.StringVar(value=f"{self.pan_values[0]:.1f}°")
        self.status_var = tk.StringVar(value="Connecting…")
        self.detail_var = tk.StringVar(value=f"Server: {endpoint}")
        self.segment_var = tk.StringVar(value="Waiting for server state…")

        self.root.title("SO100/SO101 Heading Keyposes")
        self.root.geometry("760x285")
        self.root.minsize(620, 255)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._create_socket()
        self._build_ui()
        self.root.after(10, self.poll)

    def _create_socket(self) -> None:
        if self.socket is not None:
            self.socket.close(0)
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.SNDTIMEO, 80)
        self.socket.setsockopt(zmq.RCVTIMEO, 80)
        self.socket.connect(self.endpoint)

    def _build_ui(self) -> None:
        frame = ttk.Frame(self.root, padding=14)
        frame.pack(fill=tk.BOTH, expand=True)

        self.status_label = tk.Label(
            frame,
            textvariable=self.status_var,
            anchor=tk.W,
            font=("TkDefaultFont", 13, "bold"),
            fg="#a06000",
        )
        self.status_label.pack(fill=tk.X)
        ttk.Label(frame, textvariable=self.detail_var).pack(fill=tk.X, pady=(2, 12))

        slider_frame = ttk.LabelFrame(frame, text="Desired shoulder pan", padding=12)
        slider_frame.pack(fill=tk.X)
        slider_row = ttk.Frame(slider_frame)
        slider_row.pack(fill=tk.X)
        ttk.Label(slider_row, text="Pan", width=8).pack(side=tk.LEFT)
        self.pan_scale = ttk.Scale(
            slider_row,
            variable=self.pan_var,
            from_=self.pan_values[0],
            to=self.pan_values[-1],
            orient=tk.HORIZONTAL,
            command=self.on_pan_changed,
        )
        self.pan_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)
        self.pan_scale.state(["disabled"])
        ttk.Label(slider_row, textvariable=self.pan_text, width=10, anchor=tk.E).pack(side=tk.LEFT)
        ttk.Label(slider_frame, textvariable=self.segment_var).pack(fill=tk.X, pady=(7, 0))

        controls = ttk.Frame(frame)
        controls.pack(fill=tk.X, pady=(14, 0))
        self.stream_button = ttk.Button(
            controls,
            text="Enable Pose Streaming",
            command=self.toggle_streaming,
            state=tk.DISABLED,
        )
        self.stream_button.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(controls, text="Stop / Hold", command=self.hold).pack(side=tk.LEFT, padx=6)
        ttk.Button(controls, text="Reset Fault / Enable Torque", command=self.reset_fault).pack(
            side=tk.LEFT, padx=6
        )
        tk.Button(
            controls,
            text="EMERGENCY STOP",
            command=self.emergency_stop,
            bg="#c62828",
            fg="white",
            activebackground="#8e0000",
            activeforeground="white",
            font=("TkDefaultFont", 10, "bold"),
        ).pack(side=tk.RIGHT)

    def on_pan_changed(self, value: str) -> None:
        pan = float(value)
        self.pan_text.set(f"{pan:.1f}°")
        self.update_segment_text(pan)

    def update_segment_text(self, pan: float) -> None:
        _, left, fraction = interpolate_keyposes(KEYPOSES, self.pan_values, pan)
        self.segment_var.set(
            f"Interpolating keypose {left + 1} → {left + 2}  |  "
            f"{fraction * 100.0:.0f}% through segment"
        )

    def rpc(self, request: dict[str, Any]) -> dict[str, Any] | None:
        assert self.socket is not None
        try:
            self.socket.send_json(request)
            return self.socket.recv_json()
        except (zmq.Again, zmq.ZMQError):
            self._create_socket()
            return None

    def poll(self) -> None:
        if self.streaming and self.connected:
            pose, _, _ = interpolate_keyposes(KEYPOSES, self.pan_values, self.pan_var.get())
            request: dict[str, Any] = {"type": "set_targets", "positions": pose}
        else:
            request = {"type": "get_state"}

        response = self.rpc(request)
        if response is None:
            self.set_streaming(False)
            self.connected = False
            self.status_var.set("DISCONNECTED — waiting for ZMQ server")
            self.status_label.configure(fg="#b00020")
            self.detail_var.set(f"Server: {self.endpoint}")
            self.pan_scale.state(["disabled"])
            self.stream_button.configure(state=tk.DISABLED)
        else:
            state = response.get("state")
            if isinstance(state, dict):
                self.update_state(state)
            if not response.get("ok", False):
                self.set_streaming(False)
                self.status_var.set(f"COMMAND REJECTED: {response.get('error', 'unknown error')}")
                self.status_label.configure(fg="#b00020")
        self.root.after(self.poll_ms, self.poll)

    def update_state(self, state: dict[str, Any]) -> None:
        self.connected = True
        fault = state.get("fault")
        limits = state.get("limits") or {}
        pan_limits = limits.get("shoulder_pan")

        if isinstance(pan_limits, list) and len(pan_limits) == 2:
            low = max(self.pan_values[0], float(pan_limits[0]))
            high = min(self.pan_values[-1], float(pan_limits[1]))
            if high <= low:
                self.set_streaming(False)
                self.connected = False
                self.status_var.set("KEYPOSE ERROR — pan range does not overlap calibrated limits")
                self.status_label.configure(fg="#b00020")
                self.pan_scale.state(["disabled"])
                self.stream_button.configure(state=tk.DISABLED)
                return
            self.effective_limits = (low, high)
            self.pan_scale.configure(from_=low, to=high)
            bounded = max(low, min(high, self.pan_var.get()))
            self.pan_var.set(bounded)
            self.pan_text.set(f"{bounded:.1f}°")
            self.update_segment_text(bounded)

        if fault:
            self.set_streaming(False)
            self.status_var.set(f"FAULT — TORQUE DISABLED: {fault}")
            self.status_label.configure(fg="#b00020")
            self.pan_scale.state(["disabled"])
            self.stream_button.configure(state=tk.DISABLED)
        else:
            self.pan_scale.state(["!disabled"])
            self.stream_button.configure(state=tk.NORMAL)
            if self.streaming:
                self.status_var.set("STREAMING INTERPOLATED KEYPOSES")
                self.status_label.configure(fg="#087f23")
            elif state.get("watchdog_active"):
                self.status_var.set("CONNECTED — server is holding position")
                self.status_label.configure(fg="#a06000")
            else:
                self.status_var.set("CONNECTED — move slider, then enable streaming")
                self.status_label.configure(fg="#1565c0")

        active_joints = set(state.get("active_joints") or ())
        unavailable = [joint for joint in self.joints if joint not in active_joints]
        if unavailable:
            self.set_streaming(False)
            self.status_var.set(f"KEYPOSE ERROR — inactive joints: {', '.join(unavailable)}")
            self.status_label.configure(fg="#b00020")
            self.stream_button.configure(state=tk.DISABLED)

        warnings = state.get("warnings") or []
        warning_text = f"  |  Warning: {warnings[-1]}" if warnings else ""
        self.detail_var.set(
            f"Server: {self.endpoint}  |  {len(KEYPOSES)} keyposes  |  "
            f"pan {self.effective_limits[0]:.1f}° to {self.effective_limits[1]:.1f}°"
            f"{warning_text}"
        )

        # On first connection, initialize the slider from measured pan without
        # issuing a motion command.
        if not hasattr(self, "_initialized_pan"):
            measured_pan = (state.get("joints") or {}).get("shoulder_pan", {}).get("position")
            if measured_pan is not None:
                low, high = self.effective_limits
                initial = max(low, min(high, float(measured_pan)))
                self.pan_var.set(initial)
                self.pan_text.set(f"{initial:.1f}°")
                self.update_segment_text(initial)
                self._initialized_pan = True

    def set_streaming(self, enabled: bool) -> None:
        self.streaming = enabled
        self.stream_button.configure(
            text="Disable Pose Streaming" if enabled else "Enable Pose Streaming"
        )

    def toggle_streaming(self) -> None:
        if not self.connected:
            messagebox.showwarning("Not connected", "Wait until the ZMQ server is connected.")
            return
        self.set_streaming(not self.streaming)

    def hold(self) -> None:
        self.set_streaming(False)
        response = self.rpc({"type": "hold"})
        if response and isinstance(response.get("state"), dict):
            self.update_state(response["state"])

    def emergency_stop(self) -> None:
        self.set_streaming(False)
        response = self.rpc({"type": "estop"})
        if response and isinstance(response.get("state"), dict):
            self.update_state(response["state"])

    def reset_fault(self) -> None:
        self.set_streaming(False)
        response = self.rpc({"type": "reset_fault"})
        if response is None:
            return
        if not response.get("ok", False):
            messagebox.showerror("Reset rejected", response.get("error", "Unknown error"))
        state = response.get("state")
        if isinstance(state, dict):
            self.update_state(state)

    def close(self) -> None:
        self.set_streaming(False)
        try:
            self.rpc({"type": "hold"})
        except Exception:
            pass
        if self.socket is not None:
            self.socket.close(0)
        self.context.term()
        self.root.destroy()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connect", default="tcp://127.0.0.1:5555", help="ZMQ server endpoint")
    parser.add_argument("--poll-ms", type=int, default=100, help="Command/update period")
    args = parser.parse_args()
    if args.poll_ms < 50:
        parser.error("poll-ms must be at least 50")
    return args


def main() -> None:
    args = parse_args()
    validate_keyposes(KEYPOSES)
    root = tk.Tk()
    HeadingGUI(root, args.connect, args.poll_ms)
    root.mainloop()


if __name__ == "__main__":
    main()
