#!/usr/bin/env python3
"""Tk GUI client for 2_s100_keyboard_joint_control_zmq.py."""

from __future__ import annotations

import argparse
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Any

import zmq

JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


def display(value: Any, digits: int = 2) -> str:
    if value is None:
        return "—"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


class SOFollowerGUI:
    def __init__(self, root: tk.Tk, endpoint: str, poll_ms: int):
        self.root = root
        self.endpoint = endpoint
        self.poll_ms = poll_ms
        self.context = zmq.Context()
        self.socket: zmq.Socket | None = None
        self.streaming = False
        self.damping_var = tk.BooleanVar(value=False)
        self.initialized = False
        self.active_joints = tuple(joint for joint in JOINTS if joint != "gripper")
        self.last_state: dict[str, Any] | None = None
        self.target_vars = {joint: tk.DoubleVar(value=0.0) for joint in JOINTS}
        self.target_labels = {joint: tk.StringVar(value="0.0") for joint in JOINTS}
        self.limit_labels = {joint: tk.StringVar(value="limits: —") for joint in JOINTS}
        self.status_var = tk.StringVar(value="Connecting…")
        self.detail_var = tk.StringVar(value=f"Server: {endpoint}")
        self.warning_var = tk.StringVar(value="")
        self.safety_var = tk.StringVar(value="Waiting for safety configuration…")
        self.scales: dict[str, ttk.Scale] = {}
        self.position_markers: dict[str, tk.Frame] = {}

        self.root.title("SO100/SO101 Safe ZMQ Control")
        self.root.geometry("1320x690")
        self.root.minsize(1050, 570)
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
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill=tk.X)
        self.status_label = tk.Label(
            top,
            textvariable=self.status_var,
            anchor=tk.W,
            font=("TkDefaultFont", 13, "bold"),
            fg="#a06000",
        )
        self.status_label.pack(fill=tk.X)
        ttk.Label(top, textvariable=self.detail_var).pack(fill=tk.X)
        tk.Label(top, textvariable=self.warning_var, fg="#b00020", anchor=tk.W).pack(fill=tk.X)

        controls = ttk.Frame(self.root, padding=(10, 0, 10, 8))
        controls.pack(fill=tk.X)
        self.stream_button = ttk.Button(controls, text="Enable Command Streaming", command=self.toggle_stream)
        self.stream_button.pack(side=tk.LEFT, padx=(0, 6))
        self.damping_check = ttk.Checkbutton(
            controls,
            text="Damping Mode (track measured position)",
            variable=self.damping_var,
            command=self.toggle_damping,
        )
        self.damping_check.pack(side=tk.LEFT, padx=6)
        ttk.Button(controls, text="Hold Current Position", command=self.hold_current).pack(side=tk.LEFT, padx=6)
        ttk.Button(controls, text="Stop Sending", command=self.stop_sending).pack(side=tk.LEFT, padx=6)
        ttk.Button(controls, text="Reset Fault / Enable Torque", command=self.reset_fault).pack(side=tk.LEFT, padx=6)
        tk.Button(
            controls,
            text="EMERGENCY STOP",
            command=self.emergency_stop,
            bg="#c62828",
            fg="white",
            activebackground="#8e0000",
            activeforeground="white",
            font=("TkDefaultFont", 11, "bold"),
            padx=16,
        ).pack(side=tk.RIGHT)

        body = ttk.Panedwindow(self.root, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))
        slider_frame = ttk.LabelFrame(body, text="Desired positions (degrees; gripper in %)", padding=10)
        telemetry_frame = ttk.LabelFrame(body, text="Live motor telemetry", padding=8)
        body.add(slider_frame, weight=2)
        body.add(telemetry_frame, weight=5)

        for row, joint in enumerate(JOINTS):
            ttk.Label(slider_frame, text=joint, width=16).grid(row=row, column=0, sticky=tk.W, pady=8)
            scale = ttk.Scale(
                slider_frame,
                variable=self.target_vars[joint],
                from_=-100.0,
                to=100.0,
                orient=tk.HORIZONTAL,
                command=lambda value, name=joint: self.target_labels[name].set(f"{float(value):.1f}"),
            )
            scale.grid(row=row, column=1, sticky=tk.EW, padx=8)
            ttk.Label(slider_frame, textvariable=self.target_labels[joint], width=8).grid(
                row=row, column=2, sticky=tk.E
            )
            ttk.Label(slider_frame, textvariable=self.limit_labels[joint], width=22).grid(
                row=row, column=3, sticky=tk.E
            )
            self.scales[joint] = scale
            marker = tk.Frame(scale, bg="#9aa8b5", width=2, height=16)
            marker.place_forget()
            self.position_markers[joint] = marker
        slider_frame.columnconfigure(1, weight=1)
        ttk.Label(
            slider_frame,
            text="Faint vertical marker = measured joint position",
            foreground="#6f7c86",
        ).grid(row=len(JOINTS), column=0, columnspan=4, sticky=tk.W, pady=(10, 0))

        columns = (
            "position",
            "target",
            "commanded",
            "traj_velocity",
            "velocity_raw",
            "temperature",
            "current",
            "voltage",
            "load",
            "moving",
            "status",
        )
        self.tree = ttk.Treeview(telemetry_frame, columns=columns, show="tree headings", height=9)
        self.tree.heading("#0", text="Joint")
        self.tree.column("#0", width=115, stretch=False)
        headings = {
            "position": "Position",
            "target": "Target",
            "commanded": "Sent goal",
            "traj_velocity": "Cmd speed",
            "velocity_raw": "Velocity raw",
            "temperature": "Temp °C",
            "current": "Current A/raw",
            "voltage": "Voltage V",
            "load": "Load raw",
            "moving": "Moving",
            "status": "Status",
        }
        widths = {
            "position": 75,
            "target": 70,
            "commanded": 75,
            "traj_velocity": 78,
            "velocity_raw": 86,
            "temperature": 68,
            "current": 105,
            "voltage": 70,
            "load": 72,
            "moving": 58,
            "status": 55,
        }
        for column in columns:
            self.tree.heading(column, text=headings[column])
            self.tree.column(column, width=widths[column], anchor=tk.CENTER, stretch=False)
        for joint in JOINTS:
            self.tree.insert("", tk.END, iid=joint, text=joint, values=("—",) * len(columns))
        self.tree.tag_configure("hot", background="#ffcdd2")
        self.tree.tag_configure("warm", background="#fff3cd")
        self.tree.pack(fill=tk.BOTH, expand=True)

        info = ttk.LabelFrame(self.root, text="Safety behavior", padding=(10, 5))
        info.pack(fill=tk.X, padx=10, pady=(0, 10))
        ttk.Label(
            info,
            text=(
                "Targets are clamped to calibrated limits and filtered by velocity/acceleration limits. "
                "If commands stop, the server watchdog holds the latest smoothed goal. "
                "Over-temperature or sustained over-current disables motor torque and latches a fault."
            ),
            wraplength=1250,
        ).pack(fill=tk.X)
        ttk.Label(info, textvariable=self.safety_var, wraplength=1250).pack(fill=tk.X, pady=(3, 0))

    def update_position_marker(
        self, joint: str, position: Any, low: float, high: float
    ) -> None:
        marker = self.position_markers[joint]
        if position is None or high <= low:
            marker.place_forget()
            return
        scale = self.scales[joint]
        width = scale.winfo_width()
        if width <= 1:
            return
        # Keep the marker aligned with the Scale trough rather than its outer edge.
        thumb_padding = min(12.0, width * 0.08)
        fraction = (float(position) - low) / (high - low)
        fraction = max(0.0, min(1.0, fraction))
        x = thumb_padding + fraction * max(1.0, width - 2.0 * thumb_padding)
        marker.place(x=int(x), rely=0.5, anchor=tk.CENTER)
        marker.lift()

    def rpc(self, request: dict[str, Any]) -> dict[str, Any] | None:
        assert self.socket is not None
        try:
            self.socket.send_json(request)
            return self.socket.recv_json()
        except (zmq.Again, zmq.ZMQError):
            self._create_socket()
            return None

    def poll(self) -> None:
        if self.damping_var.get() and self.initialized and self.last_state:
            joint_state = self.last_state.get("joints") or {}
            measured = {
                joint: joint_state.get(joint, {}).get("position")
                for joint in self.active_joints
            }
            if measured and all(value is not None for value in measured.values()):
                request = {"type": "set_targets", "positions": measured}
            else:
                request = {"type": "get_state"}
        elif self.streaming and self.initialized:
            request = {
                "type": "set_targets",
                "positions": {
                    joint: self.target_vars[joint].get() for joint in self.active_joints
                },
            }
        else:
            request = {"type": "get_state"}

        response = self.rpc(request)
        if response is None:
            self.status_var.set("DISCONNECTED — waiting for ZMQ server")
            self.status_label.configure(fg="#b00020")
            self.detail_var.set(f"Server: {self.endpoint}")
        else:
            if not response.get("ok", False):
                self.warning_var.set(response.get("error", "Server rejected command"))
                self.streaming = False
                self.damping_var.set(False)
                self.stream_button.configure(text="Enable Command Streaming")
            state = response.get("state")
            if isinstance(state, dict):
                self.update_state(state)
        self.root.after(self.poll_ms, self.poll)

    def update_state(self, state: dict[str, Any]) -> None:
        self.last_state = state
        fault = state.get("fault")
        connected = not fault
        if fault:
            self.status_var.set(f"FAULT — TORQUE DISABLED: {fault}")
            self.status_label.configure(fg="#b00020")
            self.streaming = False
            self.damping_var.set(False)
            self.stream_button.configure(text="Enable Command Streaming")
        elif self.damping_var.get():
            self.status_var.set("DAMPING MODE — measured positions are being sent as targets")
            self.status_label.configure(fg="#6a1b9a")
        elif state.get("watchdog_active"):
            self.status_var.set("WATCHDOG HOLD — command stream stopped")
            self.status_label.configure(fg="#a06000")
        elif state.get("control_enabled"):
            self.status_var.set("CONNECTED — safe control active")
            self.status_label.configure(fg="#087f23")
        else:
            self.status_var.set("CONNECTED — no motion commands enabled")
            self.status_label.configure(fg="#1565c0")

        age = state.get("command_age_s")
        active_joints = state.get("active_joints")
        if isinstance(active_joints, list):
            self.active_joints = tuple(joint for joint in active_joints if joint in JOINTS)
        gripper_enabled = bool(state.get("gripper_enabled", False))
        self.detail_var.set(
            f"Server {self.endpoint}  |  Serial {state.get('port', '—')}  |  "
            f"Loop {display(state.get('loop_hz'), 1)} Hz  |  "
            f"Command age {display(age, 3)} s  |  "
            f"Serial errors {state.get('communication_errors', 0)}  |  "
            f"Gripper {'ENABLED' if gripper_enabled else 'DISABLED (torque off)'}"
        )
        warnings = state.get("warnings") or []
        self.warning_var.set(" | ".join(str(item) for item in warnings[-3:]))

        safety = state.get("safety") or {}
        max_speed = state.get("max_speed") or {}
        max_acceleration = state.get("max_acceleration") or {}
        speed_text = ", ".join(
            f"{joint} {display(max_speed.get(joint), 0)}/{display(max_acceleration.get(joint), 0)}"
            for joint in JOINTS
        )
        self.safety_var.set(
            f"Watchdog {display(state.get('watchdog_timeout_s'), 2)} s | "
            f"temperature warn/fault {display(safety.get('warn_temperature_c'), 0)}/"
            f"{display(safety.get('max_temperature_c'), 0)} °C | "
            f"current fault {display(safety.get('max_current_raw'), 0)} raw for "
            f"{display(safety.get('overcurrent_seconds'), 1)} s | "
            f"speed/acceleration limits: {speed_text}"
        )

        limits = state.get("limits") or {}
        commanded = state.get("commanded") or {}
        joints = state.get("joints") or {}
        target = state.get("target") or {}
        trajectory_velocity = state.get("trajectory_velocity") or {}

        if not self.initialized and all(joint in commanded for joint in JOINTS):
            for joint in JOINTS:
                self.target_vars[joint].set(float(commanded[joint]))
                self.target_labels[joint].set(f"{float(commanded[joint]):.1f}")
            self.initialized = True

        for joint in JOINTS:
            if joint in limits and len(limits[joint]) == 2:
                low, high = (float(limits[joint][0]), float(limits[joint][1]))
                self.scales[joint].configure(from_=low, to=high)
                unit = "%" if joint == "gripper" else "°"
                if joint == "gripper" and not gripper_enabled:
                    self.scales[joint].state(["disabled"])
                    self.limit_labels[joint].set("DISABLED — torque off")
                else:
                    self.scales[joint].state(["!disabled"])
                    self.limit_labels[joint].set(f"limits: {low:.1f} to {high:.1f} {unit}")
                if self.initialized:
                    bounded = max(low, min(high, self.target_vars[joint].get()))
                    self.target_vars[joint].set(bounded)

            data = joints.get(joint, {})
            current = "—"
            if data.get("current_a") is not None or data.get("current_raw") is not None:
                current = f"{display(data.get('current_a'), 2)}/{display(data.get('current_raw'), 0)}"
            values = (
                display(data.get("position")),
                display(target.get(joint)),
                display(commanded.get(joint)),
                display(trajectory_velocity.get(joint)),
                display(data.get("velocity_raw"), 0),
                display(data.get("temperature_c"), 0),
                current,
                display(data.get("voltage_v"), 1),
                display(data.get("load_raw"), 0),
                display(data.get("moving"), 0),
                display(data.get("status"), 0),
            )
            if joint in limits and len(limits[joint]) == 2:
                self.update_position_marker(
                    joint,
                    data.get("position"),
                    float(limits[joint][0]),
                    float(limits[joint][1]),
                )
            else:
                self.position_markers[joint].place_forget()

            temperature = data.get("temperature_c")
            tag = ""
            if temperature is not None and temperature >= 65:
                tag = "hot"
            elif temperature is not None and temperature >= 55:
                tag = "warm"
            self.tree.item(joint, values=values, tags=(tag,) if tag else ())

        if self.damping_var.get():
            for joint in self.active_joints:
                position = joints.get(joint, {}).get("position")
                if position is not None:
                    self.target_vars[joint].set(float(position))
                    self.target_labels[joint].set(f"{float(position):.1f}")

        if not connected:
            self.streaming = False
            self.damping_var.set(False)

    def toggle_stream(self) -> None:
        if not self.initialized:
            messagebox.showwarning("Not connected", "Wait until live robot state is received.")
            return
        if self.last_state and self.last_state.get("fault"):
            messagebox.showwarning("Controller fault", "Reset the fault before enabling commands.")
            return
        self.streaming = not self.streaming
        if self.streaming:
            self.damping_var.set(False)
        self.stream_button.configure(
            text="Disable Command Streaming" if self.streaming else "Enable Command Streaming"
        )

    def toggle_damping(self) -> None:
        if not self.damping_var.get():
            return
        if not self.initialized:
            self.damping_var.set(False)
            messagebox.showwarning("Not connected", "Wait until live robot state is received.")
            return
        if self.last_state and self.last_state.get("fault"):
            self.damping_var.set(False)
            messagebox.showwarning("Controller fault", "Reset the fault before enabling damping mode.")
            return
        self.streaming = False
        self.stream_button.configure(text="Enable Command Streaming")

    def hold_current(self) -> None:
        if not self.last_state:
            return
        joints = self.last_state.get("joints") or {}
        for joint in JOINTS:
            position = joints.get(joint, {}).get("position")
            if position is not None:
                self.target_vars[joint].set(float(position))
                self.target_labels[joint].set(f"{float(position):.1f}")
        self.rpc({"type": "hold"})

    def stop_sending(self) -> None:
        self.streaming = False
        self.damping_var.set(False)
        self.stream_button.configure(text="Enable Command Streaming")
        response = self.rpc({"type": "hold"})
        if response and isinstance(response.get("state"), dict):
            self.update_state(response["state"])

    def emergency_stop(self) -> None:
        self.streaming = False
        self.damping_var.set(False)
        self.stream_button.configure(text="Enable Command Streaming")
        response = self.rpc({"type": "estop"})
        if response and isinstance(response.get("state"), dict):
            self.update_state(response["state"])

    def reset_fault(self) -> None:
        self.streaming = False
        self.damping_var.set(False)
        self.stream_button.configure(text="Enable Command Streaming")
        response = self.rpc({"type": "reset_fault"})
        if response is None:
            return
        if not response.get("ok"):
            messagebox.showerror("Reset rejected", response.get("error", "Unknown error"))
        state = response.get("state")
        if isinstance(state, dict):
            self.update_state(state)

    def close(self) -> None:
        self.streaming = False
        self.damping_var.set(False)
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
    parser.add_argument("--poll-ms", type=int, default=100, help="GUI update period")
    args = parser.parse_args()
    if args.poll_ms < 50:
        parser.error("poll-ms must be at least 50")
    return args


def main() -> None:
    args = parse_args()
    root = tk.Tk()
    SOFollowerGUI(root, args.connect, args.poll_ms)
    root.mainloop()


if __name__ == "__main__":
    main()
