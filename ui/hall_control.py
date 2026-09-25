#!/usr/bin/env python3
"""OrbitDrive_HALL_5 — motor control UI (Tkinter + matplotlib).

Waveshare USB-CAN-A @ COM4 (2 M serial baud, 500 kbit/s CAN).
Graphs: HU/HV/HW (mV) and desired vs actual angle (°).
Controls: angle slider, velocity RPM, velocity/angle/torque PID, EKF.

  python ui/hall_control.py
"""
from __future__ import annotations

import json
import math
import os
import struct
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import ttk, messagebox

import numpy as np
import serial
from serial.tools import list_ports

import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# --- Protocol (include/orbit_protocol.h) ------------------------------------
CAN_CMD_BASE = 0x140
CAN_RPT_BASE = 0x240
CAN_NODE_STRIDE = 0x10

CMD_SET_ANGLE = 0x01
CMD_SET_ZERO = 0x02
CMD_SAVE_CFG = 0x03
CMD_RECALIBRATE = 0x05
CMD_SET_PARAM = 0x06
CMD_GET_PARAMS = 0x07
CMD_SET_NODE = 0x08
CMD_SET_VEL = 0x09
CMD_SET_ENABLE = 0x0A
CMD_REBOOT = 0x0B
CMD_MEASURE_STOPS = 0x0C
CMD_HEARTBEAT = 0x0D

RPT_TELEMETRY = 0x01
RPT_PARAM = 0x02
RPT_HALLS = 0x03
RPT_STATUS = 0x04
RPT_LIMITS = 0x05
RPT_NODE = 0x06

P_VEL_P, P_VEL_I, P_VEL_D, P_VEL_RAMP = 0, 1, 2, 3
P_ANGLE_P, P_LPF = 4, 5
P_EKF_EN, P_EKF_QA, P_EKF_QV, P_EKF_R = 6, 7, 8, 9
P_TRQ_P, P_TRQ_I, P_TRQ_D, P_TRQ_LPF = 10, 11, 12, 13
P_SLEW = 14
P_ACCEL = 15
P_DECEL = 16
P_SOFT_MIN = 17
P_SOFT_MAX = 18
P_PRESTOP = 19

# Gains that were running well before the closed-loop nudge. Always available.
REFERENCE_PRESET = {
    "name": "Working reference",
    "locked": True,
    "values": {
        "0": 0.026, "1": 0.30, "2": 0.0012, "3": 70.0,
        "4": 50.0, "5": 0.05,
        "10": 4.0, "11": 10.0, "12": 0.3, "13": 0.005,
        "14": 360.0, "15": 1000.0, "16": 1000.0,
    },
}


def _preset_file() -> str:
    base = os.environ.get("APPDATA") or os.path.expanduser("~")
    folder = os.path.join(base, "OrbitDrive_HALL_5")
    os.makedirs(folder, exist_ok=True)
    return os.path.join(folder, "pid_presets.json")

WS_BAUD = 2_000_000
WINDOW_S = 12.0
DEFAULT_PORT = "COM12"
DEFAULT_NODE = 1
CAN_NODE_MAX = 15


def node_cmd_base(node: int) -> int:
    return CAN_CMD_BASE + int(node) * CAN_NODE_STRIDE


def node_rpt_base(node: int) -> int:
    return CAN_RPT_BASE + int(node) * CAN_NODE_STRIDE


def node_from_can_id(can_id: int) -> int | None:
    """Return node index if can_id is an Orbit RPT/CMD id, else None."""
    for base in (CAN_RPT_BASE, CAN_CMD_BASE):
        off = can_id - base
        if off < 0:
            continue
        node = off // CAN_NODE_STRIDE
        rem = off % CAN_NODE_STRIDE
        if 0 <= node <= CAN_NODE_MAX and rem <= 0x0A:
            return node
    return None


def config_frame():
    f = bytearray(20)
    f[0], f[1], f[2] = 0xAA, 0x55, 0x12
    f[3], f[4] = 0x03, 0x01  # 500 kbit/s
    f[19] = sum(f[2:19]) & 0xFF
    return bytes(f)


def pack_tx(can_id: int, payload: bytes = b"") -> bytes:
    dlc = len(payload) & 0x0F
    frame = bytearray([0xAA, 0xC0 | dlc, can_id & 0xFF, (can_id >> 8) & 0xFF])
    frame.extend(payload)
    frame.append(0x55)
    return bytes(frame)


def parse_frames(buf: bytearray):
    out = []
    while len(buf) >= 2:
        if buf[0] != 0xAA:
            del buf[0]
            continue
        info = buf[1]
        if (info & 0xC0) != 0xC0:
            del buf[0]
            continue
        dlc = info & 0x0F
        id_len = 4 if (info & 0x20) else 2
        total = 2 + id_len + dlc + 1
        if dlc > 8 or len(buf) < total:
            if dlc > 8:
                del buf[0]
            break
        if buf[total - 1] != 0x55:
            del buf[0]
            continue
        can_id = int.from_bytes(bytes(buf[2 : 2 + id_len]), "little")
        payload = bytes(buf[2 + id_len : 2 + id_len + dlc])
        out.append((can_id, payload))
        del buf[:total]
    return out


class HallControlApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("OrbitDrive_HALL_5 — Motor Control")
        root.geometry("1100x820")
        root.minsize(960, 700)

        self.ser: serial.Serial | None = None
        self.reader_stop = threading.Event()
        self.reader_thread: threading.Thread | None = None
        self.lock = threading.Lock()
        self.rx_buf = bytearray()

        self.node = DEFAULT_NODE
        self.board_node: int | None = None  # last RPT_NODE from selected drive
        self._scan_hits: dict[int, float] = {}  # node -> last seen time
        self.soft_lo = -135.0
        self.soft_hi = 135.0

        self.desired = 0.0
        self.actual = 0.0
        self.uq = 0.0
        self.vel_rads = 0.0
        self.hu = self.hv = self.hw = 0.0
        self.params: dict[int, float] = {}
        self.connected = False
        self._slider_send_t = 0.0
        self._sync_params = False  # True only after Get Params until fields filled
        self._params_dirty = set()  # indices user is editing / just applied
        self._focused_entry = None
        self._slider_dragging = False
        self._ignore_slider = False  # skip command= callback during programmatic set
        self._limits_set = False
        self._last_lo = None
        self._last_hi = None
        self._osc_on = False
        self._osc_target = 45.0
        self._osc_sent_t = 0.0
        self.auto_connect = True
        self.auto_oscillate = False  # do not command angles during/after roll home

        t0 = time.time()
        self.t_hist: deque[float] = deque(maxlen=2000)
        self.des_hist: deque[float] = deque(maxlen=2000)
        self.act_hist: deque[float] = deque(maxlen=2000)
        self.hu_hist: deque[float] = deque(maxlen=2000)
        self.hv_hist: deque[float] = deque(maxlen=2000)
        self.hw_hist: deque[float] = deque(maxlen=2000)
        self._hist_t0 = t0

        self._build()
        self.root.after(50, self._ui_tick)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        if self.auto_connect:
            self.root.after(300, self._boot_auto)

    # --- IDs ----------------------------------------------------------------
    def _cmd(self, off: int) -> int:
        return CAN_CMD_BASE + self.node * CAN_NODE_STRIDE + off

    def _rpt(self, off: int) -> int:
        return CAN_RPT_BASE + self.node * CAN_NODE_STRIDE + off

    def _can_id_text(self, node: int) -> str:
        return f"CMD 0x{node_cmd_base(node):03X}  RPT 0x{node_rpt_base(node):03X}"

    def _update_can_id_lbl(self):
        try:
            n = int(self.node_var.get())
        except Exception:
            return
        self.can_id_lbl.set(self._can_id_text(n))

    # --- UI -----------------------------------------------------------------
    def _build(self):
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill=tk.X)

        ttk.Label(top, text="Port").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=10, values=self._ports())
        self.port_box.pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side=tk.LEFT)

        ttk.Label(top, text="CAN node").pack(side=tk.LEFT, padx=(12, 2))
        self.node_var = tk.IntVar(value=DEFAULT_NODE)
        ttk.Spinbox(top, from_=0, to=CAN_NODE_MAX, width=4, textvariable=self.node_var).pack(side=tk.LEFT)
        self.can_id_lbl = tk.StringVar(value=self._can_id_text(DEFAULT_NODE))
        ttk.Label(top, textvariable=self.can_id_lbl, foreground="#666").pack(side=tk.LEFT, padx=6)
        self.node_var.trace_add("write", lambda *_: self._update_can_id_lbl())

        self.btn_conn = ttk.Button(top, text="Connect", command=self._toggle_conn)
        self.btn_conn.pack(side=tk.LEFT, padx=8)
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.LEFT, padx=8)

        # Multi-drive CAN ID assignment
        canf = ttk.LabelFrame(self.root, text="Multi-drive CAN ID (unique per Orbit Drive)", padding=6)
        canf.pack(fill=tk.X, padx=8, pady=(0, 4))
        crow = ttk.Frame(canf)
        crow.pack(fill=tk.X)
        ttk.Label(crow, text="Assign board ID").pack(side=tk.LEFT)
        self.new_node_var = tk.IntVar(value=DEFAULT_NODE)
        ttk.Spinbox(crow, from_=0, to=CAN_NODE_MAX, width=4, textvariable=self.new_node_var).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(crow, text="Write ID to EEPROM + reboot", command=self._assign_can_node).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(crow, text="Scan bus", command=self._scan_nodes).pack(side=tk.LEFT, padx=4)
        self.scan_lbl = tk.StringVar(value="Online nodes: —")
        ttk.Label(canf, textvariable=self.scan_lbl).pack(anchor=tk.W, pady=(4, 0))
        ttk.Label(
            canf,
            text="Connect to the drive's current node, set a new unique ID (0–15), then Write. "
            "After reboot, switch CAN node and Connect again.",
            wraplength=900,
        ).pack(anchor=tk.W)

        body = ttk.Panedwindow(self.root, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        left_host, left = self._scroll_pane(body)
        right_host, right = self._scroll_pane(body)
        body.add(left_host, weight=3)
        body.add(right_host, weight=2)

        # Angle control
        ang = ttk.LabelFrame(left, text="Angle command", padding=8)
        ang.pack(fill=tk.X)
        self.cmd_var = tk.DoubleVar(value=0.0)
        self.des_lbl = tk.StringVar(value="Desired: -- °")
        self.act_lbl = tk.StringVar(value="Actual: -- °")
        self.err_lbl = tk.StringVar(value="Error: -- °")
        ttk.Label(ang, textvariable=self.des_lbl, foreground="#c9a227").pack(anchor=tk.W)
        ttk.Label(ang, textvariable=self.act_lbl, foreground="#3b82f6").pack(anchor=tk.W)
        ttk.Label(ang, textvariable=self.err_lbl).pack(anchor=tk.W)

        self.slider = ttk.Scale(
            ang, from_=-135, to=135, orient=tk.HORIZONTAL, command=self._on_slider
        )
        self.slider.set(0)
        self.slider.pack(fill=tk.X, pady=6)
        # Prevent UI refresh from fighting the user while dragging.
        self.slider.bind("<ButtonPress-1>", lambda _e: self._set_dragging(True))
        self.slider.bind("<ButtonRelease-1>", lambda _e: self._set_dragging(False))
        self.lim_lbl = tk.StringVar(value="Soft limits: [-135, +135] °")
        ttk.Label(ang, textvariable=self.lim_lbl).pack(anchor=tk.W)

        lim_row = ttk.Frame(ang)
        lim_row.pack(fill=tk.X, pady=4)
        ttk.Label(lim_row, text="Soft min").pack(side=tk.LEFT)
        self.soft_min_var = tk.StringVar(value="-135.0")
        ttk.Entry(lim_row, textvariable=self.soft_min_var, width=7).pack(side=tk.LEFT, padx=2)
        ttk.Label(lim_row, text="max").pack(side=tk.LEFT, padx=(6, 0))
        self.soft_max_var = tk.StringVar(value="135.0")
        ttk.Entry(lim_row, textvariable=self.soft_max_var, width=7).pack(side=tk.LEFT, padx=2)
        ttk.Button(lim_row, text="Apply limits", command=self._apply_soft_limits).pack(
            side=tk.LEFT, padx=6
        )
        ttk.Button(lim_row, text="-135/+135", command=self._preset_soft_pitch).pack(side=tk.LEFT, padx=2)

        pre_row = ttk.Frame(ang)
        pre_row.pack(fill=tk.X, pady=2)
        ttk.Label(pre_row, text="Prestop").pack(side=tk.LEFT)
        self.prestop_var = tk.StringVar(value="2.0")
        ttk.Entry(pre_row, textvariable=self.prestop_var, width=6).pack(side=tk.LEFT, padx=4)
        ttk.Label(pre_row, text="° inside each hard stop").pack(side=tk.LEFT)
        ttk.Button(pre_row, text="Apply", command=self._apply_prestop).pack(side=tk.LEFT, padx=6)

        row = ttk.Frame(ang)
        row.pack(fill=tk.X, pady=4)
        self.manual_var = tk.StringVar(value="0.0")
        man = ttk.Entry(row, textvariable=self.manual_var, width=8)
        man.pack(side=tk.LEFT)
        man.bind("<Return>", lambda _e: self._set_manual())
        ttk.Button(row, text="Set °", command=self._set_manual).pack(side=tk.LEFT, padx=4)
        ttk.Button(row, text="Zero", command=self._zero).pack(side=tk.LEFT, padx=2)
        ttk.Button(row, text="Go 0", command=lambda: self._send_angle(0.0)).pack(side=tk.LEFT, padx=2)

        acts = ttk.Frame(ang)
        acts.pack(fill=tk.X, pady=(0, 4))
        ttk.Button(acts, text="Enable", command=self._enable).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(acts, text="Measure stops", command=self._measure_stops).pack(side=tk.LEFT, padx=2)
        ttk.Button(acts, text="Save CFG", command=self._save).pack(side=tk.LEFT, padx=2)

        osc = ttk.Frame(ang)
        osc.pack(fill=tk.X, pady=4)
        self.osc_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            osc, text="Oscillate ±", variable=self.osc_var, command=self._toggle_osc
        ).pack(side=tk.LEFT)
        self.osc_amp_var = tk.DoubleVar(value=30.0)
        amp_box = ttk.Spinbox(
            osc,
            from_=1.0,
            to=135.0,
            increment=1.0,
            width=5,
            textvariable=self.osc_amp_var,
            command=self._on_osc_amp_change,
        )
        amp_box.pack(side=tk.LEFT)
        amp_box.bind("<Return>", lambda _e: self._on_osc_amp_change())
        amp_box.bind("<FocusOut>", lambda _e: self._on_osc_amp_change())
        ttk.Label(osc, text="° (1–135)").pack(side=tk.LEFT, padx=(2, 0))
        ttk.Label(osc, text="dwell").pack(side=tk.LEFT, padx=(8, 2))
        self.osc_dwell_var = tk.DoubleVar(value=0.4)
        ttk.Spinbox(osc, from_=0.2, to=10.0, increment=0.1, width=5, textvariable=self.osc_dwell_var).pack(
            side=tk.LEFT
        )
        ttk.Label(osc, text="s after settle").pack(side=tk.LEFT, padx=2)

        # Velocity
        vel = ttk.LabelFrame(left, text="Velocity command (rpm)", padding=8)
        vel.pack(fill=tk.X, pady=6)
        self.rpm_var = tk.DoubleVar(value=0.0)
        self.rpm_scale = ttk.Scale(vel, from_=-40, to=40, orient=tk.HORIZONTAL, variable=self.rpm_var)
        self.rpm_scale.pack(fill=tk.X)
        vr = ttk.Frame(vel)
        vr.pack(fill=tk.X, pady=4)
        ttk.Button(vr, text="Apply RPM", command=self._apply_rpm).pack(side=tk.LEFT)
        ttk.Button(vr, text="Stop (angle hold)", command=self._stop_vel).pack(side=tk.LEFT, padx=6)
        self.stat_lbl = tk.StringVar(value="Uq=-- V   vel=-- rpm")
        ttk.Label(vel, textvariable=self.stat_lbl).pack(anchor=tk.W)

        # Graphs
        fig = Figure(figsize=(7.2, 5.2), dpi=100)
        self.ax_hall = fig.add_subplot(211)
        self.ax_ang = fig.add_subplot(212)
        self.ax_hall.set_title("Hall HU / HV / HW (mV)")
        self.ax_ang.set_title("Angle desired vs actual (°)")
        self.ax_hall.set_ylabel("mV")
        self.ax_ang.set_ylabel("deg")
        self.ax_ang.set_xlabel("t (s)")
        (self.ln_hu,) = self.ax_hall.plot([], [], label="HU", color="#e74c3c", lw=1.2)
        (self.ln_hv,) = self.ax_hall.plot([], [], label="HV", color="#2ecc71", lw=1.2)
        (self.ln_hw,) = self.ax_hall.plot([], [], label="HW", color="#3498db", lw=1.2)
        (self.ln_des,) = self.ax_ang.plot([], [], label="Desired", color="#f1c40f", lw=1.4)
        (self.ln_act,) = self.ax_ang.plot([], [], label="Actual", color="#3498db", lw=1.4)
        self.ax_hall.legend(loc="upper right", fontsize=8)
        self.ax_ang.legend(loc="upper right", fontsize=8)
        fig.tight_layout()
        canvas = FigureCanvasTkAgg(fig, master=left)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, pady=4)
        self.canvas = canvas
        self.fig = fig

        # Right: live PID and saved presets
        nb = ttk.Notebook(right)
        nb.pack(fill=tk.BOTH, expand=True)
        live = ttk.Frame(nb, padding=4)
        presets = ttk.Frame(nb, padding=4)
        nb.add(live, text="PID")
        nb.add(presets, text="Presets")

        self.entries: dict[int, tk.StringVar] = {}
        self._pid_frame(
            live,
            "Angle loop",
            [
                (P_ANGLE_P, "Angle P"),
                (P_LPF, "LPF Tf"),
                (P_SLEW, "Slew °/s"),
                (P_ACCEL, "Accel °/s²"),
                (P_DECEL, "Decel °/s²"),
            ],
        )
        self._pid_frame(
            live,
            "Velocity PID",
            [(P_VEL_P, "Vel P"), (P_VEL_I, "Vel I"), (P_VEL_D, "Vel D"), (P_VEL_RAMP, "Vel ramp")],
        )
        self._pid_frame(
            live,
            "Torque (current) PID",
            [(P_TRQ_P, "Trq P"), (P_TRQ_I, "Trq I"), (P_TRQ_D, "Trq D"), (P_TRQ_LPF, "Trq LPF")],
        )

        ekf = ttk.LabelFrame(live, text="Hall angle EKF", padding=8)
        ekf.pack(fill=tk.X, pady=4)
        self.ekf_en = tk.BooleanVar(value=False)
        ttk.Checkbutton(ekf, text="Enable EKF", variable=self.ekf_en).pack(anchor=tk.W)
        for idx, lab in [(P_EKF_QA, "q_angle"), (P_EKF_QV, "q_vel"), (P_EKF_R, "r_meas")]:
            r = ttk.Frame(ekf)
            r.pack(fill=tk.X, pady=1)
            ttk.Label(r, text=lab, width=10).pack(side=tk.LEFT)
            v = tk.StringVar(value="")
            self.entries[idx] = v
            e = ttk.Entry(r, textvariable=v, width=10)
            e.pack(side=tk.LEFT)
            e.bind("<FocusIn>", lambda _ev, i=idx: self._on_entry_focus(i))
            e.bind("<FocusOut>", lambda _ev, i=idx: self._on_entry_blur(i))
            e.bind("<Return>", lambda _ev, i=idx: self._apply_one(i))
            ttk.Button(r, text="Set", width=4, command=lambda i=idx: self._apply_one(i)).pack(
                side=tk.LEFT, padx=4
            )

        btns = ttk.Frame(live)
        btns.pack(fill=tk.X, pady=8)
        ttk.Button(btns, text="Apply all PIDs", command=self._apply_pids).pack(fill=tk.X, pady=2)
        ttk.Button(btns, text="Save PIDs to EEPROM", command=self._save_pids_eeprom).pack(fill=tk.X, pady=2)
        ttk.Button(btns, text="Get params from board", command=self._get_params).pack(fill=tk.X, pady=2)
        ttk.Button(btns, text="Recalibrate (reboot)", command=self._recal).pack(fill=tk.X, pady=2)
        ttk.Button(btns, text="Reboot", command=self._reboot).pack(fill=tk.X, pady=2)

        self._build_presets(presets)

        self.log = tk.Text(right, height=8, width=36, font=("Consolas", 9))
        self.log.pack(fill=tk.BOTH, expand=True, pady=4)

    def _scroll_pane(self, parent):
        """Vertical scrollbar so every control stays reachable."""
        host = ttk.Frame(parent)
        bg = ttk.Style().lookup("TFrame", "background") or "#f0f0f0"
        canvas = tk.Canvas(host, highlightthickness=0, borderwidth=0, bg=bg)
        bar = ttk.Scrollbar(host, orient=tk.VERTICAL, command=canvas.yview)
        canvas.configure(yscrollcommand=bar.set)
        inner = ttk.Frame(canvas, padding=4)
        window = canvas.create_window((0, 0), window=inner, anchor="nw")

        def _fit_width(event):
            canvas.itemconfigure(window, width=event.width)

        def _fit_scroll(_event=None):
            canvas.configure(scrollregion=canvas.bbox("all"))

        def _wheel(event):
            canvas.yview_scroll(int(-event.delta / 120), "units")

        canvas.bind("<Configure>", _fit_width)
        inner.bind("<Configure>", _fit_scroll)
        canvas.bind("<Enter>", lambda _e: canvas.bind_all("<MouseWheel>", _wheel))
        canvas.bind("<Leave>", lambda _e: canvas.unbind_all("<MouseWheel>"))
        bar.pack(side=tk.RIGHT, fill=tk.Y)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        return host, inner

    def _pid_frame(self, parent, title, fields):
        fr = ttk.LabelFrame(parent, text=title, padding=8)
        fr.pack(fill=tk.X, pady=4)
        for idx, lab in fields:
            r = ttk.Frame(fr)
            r.pack(fill=tk.X, pady=1)
            ttk.Label(r, text=lab, width=10).pack(side=tk.LEFT)
            v = tk.StringVar(value="")
            self.entries[idx] = v
            e = ttk.Entry(r, textvariable=v, width=10)
            e.pack(side=tk.LEFT)
            e.bind("<FocusIn>", lambda _ev, i=idx: self._on_entry_focus(i))
            e.bind("<FocusOut>", lambda _ev, i=idx: self._on_entry_blur(i))
            e.bind("<Return>", lambda _ev, i=idx: self._apply_one(i))
            ttk.Button(r, text="Set", width=4, command=lambda i=idx: self._apply_one(i)).pack(
                side=tk.LEFT, padx=4
            )

    def _build_presets(self, parent):
        ttk.Label(
            parent,
            text="Save the fields on the PID tab. Working reference is the set that was already running well.",
            wraplength=280,
        ).pack(anchor=tk.W, pady=(0, 6))
        self.preset_list = tk.Listbox(parent, height=8, exportselection=False)
        self.preset_list.pack(fill=tk.BOTH, expand=True)
        self.preset_name = tk.StringVar()
        ttk.Entry(parent, textvariable=self.preset_name).pack(fill=tk.X, pady=4)
        row = ttk.Frame(parent)
        row.pack(fill=tk.X)
        ttk.Button(row, text="Save", command=self._preset_save).pack(side=tk.LEFT, padx=2)
        ttk.Button(row, text="Load", command=self._preset_load).pack(side=tk.LEFT, padx=2)
        ttk.Button(row, text="Apply", command=self._preset_apply).pack(side=tk.LEFT, padx=2)
        ttk.Button(row, text="Delete", command=self._preset_delete).pack(side=tk.LEFT, padx=2)
        self._presets = []
        self._preset_load_file()
        self._preset_refresh()

    def _preset_load_file(self):
        rows = []
        path = _preset_file()
        if os.path.isfile(path):
            try:
                with open(path, "r", encoding="utf-8") as f:
                    rows = json.load(f)
            except Exception:
                rows = []
        if not isinstance(rows, list):
            rows = []
        have_ref = any(isinstance(r, dict) and r.get("name") == REFERENCE_PRESET["name"] for r in rows)
        if not have_ref:
            rows.insert(0, json.loads(json.dumps(REFERENCE_PRESET)))
        self._presets = [r for r in rows if isinstance(r, dict) and r.get("name")]

    def _preset_write(self):
        with open(_preset_file(), "w", encoding="utf-8") as f:
            json.dump(self._presets, f, indent=2)

    def _preset_refresh(self):
        self.preset_list.delete(0, tk.END)
        for row in self._presets:
            self.preset_list.insert(tk.END, row.get("name", ""))

    def _preset_selected(self):
        sel = self.preset_list.curselection()
        if not sel:
            return None
        i = int(sel[0])
        if i < 0 or i >= len(self._presets):
            return None
        return self._presets[i]

    def _capture_pid_fields(self) -> dict:
        out = {}
        for idx, var in self.entries.items():
            s = var.get().strip()
            if not s:
                continue
            try:
                out[str(idx)] = float(s)
            except ValueError:
                continue
        return out

    def _fill_pid_fields(self, values: dict):
        for key, val in values.items():
            try:
                idx = int(key)
            except (TypeError, ValueError):
                continue
            if idx in self.entries:
                self.entries[idx].set(f"{float(val):.5g}")

    def _preset_save(self):
        name = self.preset_name.get().strip()
        if not name:
            messagebox.showinfo("Presets", "Type a name first")
            return
        if name == REFERENCE_PRESET["name"]:
            messagebox.showinfo("Presets", "Working reference stays as it is")
            return
        values = self._capture_pid_fields()
        if len(values) < 4:
            messagebox.showinfo("Presets", "PID fields are empty. Get params or type values first.")
            return
        for row in self._presets:
            if row.get("name") == name and not row.get("locked"):
                row["values"] = values
                break
        else:
            self._presets.append({"name": name, "locked": False, "values": values})
        self._preset_write()
        self._preset_refresh()
        self._log(f"preset saved: {name}")

    def _preset_load(self):
        row = self._preset_selected()
        if row is None:
            messagebox.showinfo("Presets", "Select a preset")
            return
        self._fill_pid_fields(row.get("values") or {})
        self.preset_name.set(row.get("name", ""))
        self._log(f"preset loaded into fields: {row.get('name')}")

    def _preset_apply(self):
        row = self._preset_selected()
        if row is None:
            messagebox.showinfo("Presets", "Select a preset")
            return
        self._fill_pid_fields(row.get("values") or {})
        self.preset_name.set(row.get("name", ""))
        self._log(f"preset loaded into fields: {row.get('name')}")
        self._apply_pids()

    def _preset_delete(self):
        row = self._preset_selected()
        if row is None:
            return
        if row.get("locked") or row.get("name") == REFERENCE_PRESET["name"]:
            messagebox.showinfo("Presets", "Working reference cannot be deleted")
            return
        self._presets = [r for r in self._presets if r.get("name") != row.get("name")]
        self._preset_write()
        self._preset_refresh()
        self._log(f"preset deleted: {row.get('name')}")

    def _log(self, msg: str):
        self.log.insert(tk.END, msg + "\n")
        self.log.see(tk.END)

    def _ports(self):
        return [p.device for p in list_ports.comports()] or [DEFAULT_PORT]

    def _refresh_ports(self):
        self.port_box["values"] = self._ports()

    def _boot_auto(self):
        """Connect, load all params, optionally start oscillation."""
        if not self.connected:
            self._connect()
        if not self.connected:
            return
        self.root.after(800, self._get_params)  # second fetch after telem/limits
        if self.auto_oscillate:
            self.root.after(1500, self._start_osc)

    def _on_osc_amp_change(self):
        """Clamp amp spinbox; if oscillating, retarget to new ±amp."""
        amp = self._osc_amp_clipped()
        try:
            self.osc_amp_var.set(round(amp, 1))
        except Exception:
            pass
        if self._osc_on and self.connected:
            sign = 1.0 if self._osc_target >= 0 else -1.0
            self._osc_target = sign * amp
            self._osc_sent_t = time.time()
            self._send_angle(self._osc_target)
            self._log(f"OSCILLATE amp ±{amp:.1f}° -> {self._osc_target:+.1f}°")

    def _osc_amp_clipped(self) -> float:
        lo, hi = self.soft_lo, self.soft_hi
        try:
            amp = float(self.osc_amp_var.get())
        except Exception:
            amp = 45.0
        soft_cap = 45.0
        if hi > lo:
            soft_cap = min(max(hi, 0.0), max(-lo, 0.0))
        amp = max(1.0, min(soft_cap, abs(amp)))
        return amp

    def _toggle_osc(self):
        if self.osc_var.get():
            self._start_osc()
        else:
            self._stop_osc()

    def _start_osc(self):
        if not self.connected:
            self.osc_var.set(False)
            self._log("Connect first for oscillate")
            return
        self.osc_var.set(True)
        self._osc_on = True
        amp = self._osc_amp_clipped()
        self._osc_target = amp
        self._osc_sent_t = time.time()
        self._send_angle(self._osc_target)
        self._log(f"OSCILLATE start ±{amp:.1f}°")

    def _stop_osc(self):
        self._osc_on = False
        self.osc_var.set(False)
        self._log("OSCILLATE stop")

    def _osc_tick(self, des: float, act: float):
        if not self._osc_on or not self.connected:
            return
        amp = self._osc_amp_clipped()
        # Keep target at ±amp (soft-limit aware)
        tgt = amp if self._osc_target >= 0 else -amp
        self._osc_target = tgt
        dwell = float(self.osc_dwell_var.get())
        err = abs(act - tgt)
        settled = err < 3.0 and abs(des - tgt) < 2.0
        now = time.time()
        # Also flip if we've been commanding this side long enough (profile still moving)
        min_hold = max(dwell, 0.5)
        long_enough = (now - self._osc_sent_t) >= min_hold
        if settled and long_enough:
            self._osc_target = -tgt
            self._osc_sent_t = now
            self._send_angle(self._osc_target)
            self._log(f"OSCILLATE -> {self._osc_target:+.1f}°")
        elif long_enough and (now - self._osc_sent_t) >= max(8.0, 180.0 / max(amp, 1.0)):
            # Safety flip if never settles (limits smaller than 90, etc.)
            self._osc_target = -tgt
            self._osc_sent_t = now
            self._send_angle(self._osc_target)
            self._log(f"OSCILLATE timeout flip -> {self._osc_target:+.1f}°")

    # --- Connect ------------------------------------------------------------
    def _toggle_conn(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        self.node = int(self.node_var.get())
        port = self.port_var.get().strip()
        try:
            self.ser = serial.Serial(port, WS_BAUD, timeout=0.05)
            self.ser.reset_input_buffer()
            self.ser.write(config_frame())
            time.sleep(0.2)
        except Exception as e:
            messagebox.showerror("Connect", str(e))
            return
        self.connected = True
        self.btn_conn.configure(text="Disconnect")
        self.status_var.set(f"Connected {port} node={self.node}")
        self.reader_stop.clear()
        self.reader_thread = threading.Thread(target=self._reader, daemon=True)
        self.reader_thread.start()
        self._sync_params = True
        self._params_dirty.clear()
        self._get_params()
        self._log(f"linked {port} node={self.node}  {self._can_id_text(self.node)}")

    def _disconnect(self):
        self._stop_osc()
        self.reader_stop.set()
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connected = False
        self.btn_conn.configure(text="Connect")
        self.status_var.set("Disconnected")

    def _on_close(self):
        self._disconnect()
        self.root.destroy()

    def _send(self, can_id: int, payload: bytes = b""):
        if not self.ser:
            self._log("TX skipped — not connected")
            return
        try:
            frame = pack_tx(can_id, payload)
            self.ser.write(frame)
            self.ser.flush()
        except Exception as e:
            self._log(f"TX err: {e}")

    def _set_dragging(self, on: bool):
        self._slider_dragging = on
        if not on and self.connected:
            # Final value on release
            try:
                deg = float(self.slider.get())
            except Exception:
                return
            self._send_angle(deg)

    def _slider_set(self, deg: float):
        """Programmatic slider update without spamming SET_ANGLE."""
        self._ignore_slider = True
        try:
            self.slider.set(deg)
        except Exception:
            pass
        finally:
            self._ignore_slider = False

    # --- Commands -----------------------------------------------------------
    def _send_angle(self, deg: float):
        if not self.connected:
            self._log("Connect first to send angle")
            return
        lo, hi = self.soft_lo, self.soft_hi
        if not (hi > lo):
            lo, hi = -40.0, 40.0
        deg_in = float(deg)
        deg = float(np.clip(deg_in, lo, hi))
        self.cmd_var.set(deg)
        if not self._slider_dragging:
            self._slider_set(deg)
        self.manual_var.set(f"{deg:.1f}")
        payload = struct.pack("<f", deg)
        # Send twice — Waveshare adapters occasionally drop a single frame.
        self._send(self._cmd(CMD_SET_ANGLE), payload)
        time.sleep(0.002)
        self._send(self._cmd(CMD_SET_ANGLE), payload)
        self._log(f"SET_ANGLE {deg_in:.2f} -> {deg:.2f}° (id=0x{self._cmd(CMD_SET_ANGLE):03X})")

    def _apply_prestop(self):
        if not self.connected:
            messagebox.showinfo("Prestop", "Connect first")
            return
        try:
            deg = float(self.prestop_var.get().strip())
        except ValueError:
            messagebox.showerror("Prestop", "Enter a number of degrees")
            return
        if deg < 0.0 or deg > 40.0:
            messagebox.showerror("Prestop", "Use 0 to 40°")
            return
        self._send(self._cmd(CMD_SET_PARAM), bytes([P_PRESTOP]) + struct.pack("<f", deg))
        with self.lock:
            self.params[P_PRESTOP] = deg
        self._log(f"SET prestop {deg:.1f}° inside each hard stop (Save CFG to keep)")

    def _preset_soft_pitch(self):
        self.soft_min_var.set("-135.0")
        self.soft_max_var.set("135.0")
        self._apply_soft_limits()

    def _apply_soft_limits(self):
        if not self.connected:
            messagebox.showinfo("Soft limits", "Connect first")
            return
        try:
            lo = float(self.soft_min_var.get().strip())
            hi = float(self.soft_max_var.get().strip())
        except ValueError:
            messagebox.showerror("Soft limits", "Enter numeric min/max degrees")
            return
        if hi < lo + 4.0:
            messagebox.showerror("Soft limits", "max must be at least min + 4°")
            return
        if lo < -135.0 or hi > 135.0:
            messagebox.showerror("Soft limits", "Keep limits within ±135°")
            return
        self._send(self._cmd(CMD_SET_PARAM), bytes([P_SOFT_MIN]) + struct.pack("<f", lo))
        time.sleep(0.02)
        self._send(self._cmd(CMD_SET_PARAM), bytes([P_SOFT_MAX]) + struct.pack("<f", hi))
        with self.lock:
            self.params[P_SOFT_MIN] = lo
            self.params[P_SOFT_MAX] = hi
            self.soft_lo, self.soft_hi = lo, hi
        self._log(f"SET soft limits [{lo:.1f}, {hi:.1f}]° (save CFG to keep)")
        # Pull current command inside new window
        try:
            cur = float(self.slider.get())
        except Exception:
            cur = 0.0
        self._send_angle(cur)

    def _on_slider(self, value=None):
        if self._ignore_slider or not self.connected:
            return
        now = time.time()
        if now - self._slider_send_t < 0.08:
            return
        self._slider_send_t = now
        try:
            deg = float(value) if value is not None else float(self.slider.get())
        except Exception:
            return
        lo, hi = self.soft_lo, self.soft_hi
        if hi > lo:
            deg = float(np.clip(deg, lo, hi))
        self.manual_var.set(f"{deg:.1f}")
        self.cmd_var.set(deg)
        self._send(self._cmd(CMD_SET_ANGLE), struct.pack("<f", deg))

    def _set_manual(self):
        if not self.connected:
            self._log("Connect first")
            return
        try:
            deg = float(self.manual_var.get())
        except ValueError:
            self._log("bad angle text")
            return
        self._send_angle(deg)

    def _zero(self):
        self._send(self._cmd(CMD_SET_ZERO))
        self._log("SET_ZERO")

    def _assign_can_node(self):
        """Program a unique CAN node id onto the currently connected board (EEPROM + reboot)."""
        if not self.connected:
            messagebox.showinfo("CAN ID", "Connect to the drive first (using its current node).")
            return
        try:
            new_id = int(self.new_node_var.get())
        except Exception:
            messagebox.showerror("CAN ID", "Invalid node id")
            return
        if not (0 <= new_id <= CAN_NODE_MAX):
            messagebox.showerror("CAN ID", f"Node must be 0..{CAN_NODE_MAX}")
            return
        if not messagebox.askyesno(
            "Write CAN ID",
            f"Program this Orbit Drive with CAN node {new_id}?\n\n"
            f"Current listen node: {self.node}\n"
            f"New CMD base: 0x{node_cmd_base(new_id):03X}\n"
            f"New RPT base: 0x{node_rpt_base(new_id):03X}\n\n"
            "Board will save EEPROM and reboot. Then set CAN node to the new ID and Connect.",
        ):
            return
        self._stop_osc()
        self._send(self._cmd(CMD_SET_NODE), bytes([new_id & 0xFF]))
        self._log(f"SET_NODE {self.node} -> {new_id} (board rebooting)")
        self.node_var.set(new_id)
        self.new_node_var.set(new_id)
        self._update_can_id_lbl()
        messagebox.showinfo(
            "CAN ID",
            f"Node {new_id} written. Wait for boot home, then Connect with CAN node = {new_id}.",
        )

    def _scan_nodes(self):
        """Listen briefly for Orbit Drive frames from any node on the bus."""
        owned = False
        if self.ser is None:
            port = self.port_var.get().strip()
            try:
                self.ser = serial.Serial(port, WS_BAUD, timeout=0.05)
                self.ser.reset_input_buffer()
                self.ser.write(config_frame())
                time.sleep(0.15)
                owned = True
            except Exception as e:
                messagebox.showerror("Scan", str(e))
                return

        # Clear and collect via shared hit map (also filled by reader thread if running)
        with self.lock:
            self._scan_hits.clear()

        self._log("Scanning CAN bus for Orbit nodes (2.5s)...")
        t0 = time.time()
        local_buf = bytearray()
        while time.time() - t0 < 2.5:
            # If reader thread owns RX, just wait on _scan_hits; else read here
            if owned or not self.connected:
                try:
                    chunk = self.ser.read(512) if self.ser else b""
                except Exception:
                    break
                if chunk:
                    local_buf.extend(chunk)
                for cid, pl in parse_frames(local_buf):
                    n = None
                    if (cid - CAN_RPT_BASE) >= 0 and (cid - CAN_RPT_BASE) % CAN_NODE_STRIDE == RPT_NODE and len(pl) >= 1:
                        n = int(pl[0]) & 0xFF
                    else:
                        n = node_from_can_id(cid)
                    if n is not None and 0 <= n <= CAN_NODE_MAX:
                        with self.lock:
                            self._scan_hits[n] = time.time()
            time.sleep(0.02)

        with self.lock:
            ordered = sorted(self._scan_hits.keys())

        if owned:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

        if ordered:
            detail = ", ".join(f"{n} (RPT 0x{node_rpt_base(n):03X})" for n in ordered)
            self.scan_lbl.set(f"Online nodes: {ordered}")
            self._log(f"Scan found nodes: {detail}")
            messagebox.showinfo("Scan", f"Found Orbit Drive node(s): {ordered}\n{detail}")
        else:
            self.scan_lbl.set("Online nodes: (none heard)")
            self._log("Scan: no Orbit nodes heard")
            messagebox.showinfo("Scan", "No Orbit Drive telemetry heard. Check power / CAN / baud.")

    def _enable(self):
        self._send(self._cmd(CMD_SET_ENABLE), bytes([1]))
        self._log("ENABLE — fault cleared, holding here")

    def _measure_stops(self):
        if not messagebox.askyesno(
            "Measure stops",
            "Sweep both hard stops and replace the saved span?\n\n"
            "The shaft will drive to each end. Keep hands clear.",
        ):
            return
        self._send(self._cmd(CMD_MEASURE_STOPS))
        self._log("MEASURE STOPS")

    def _save(self):
        self._send(self._cmd(CMD_SAVE_CFG))
        self._log("SAVE_CFG")

    def _save_pids_eeprom(self):
        """Apply all PID/profile fields to RAM, then permanently write EEPROM."""
        if not self.connected:
            messagebox.showinfo("EEPROM", "Connect first")
            return
        if not messagebox.askyesno(
            "Save PIDs to EEPROM",
            "Apply current PID / slew / accel values and write them permanently to EEPROM?\n\n"
            "They will load automatically after power-cycle.",
        ):
            return
        self._apply_pids()
        time.sleep(0.05)
        self._send(self._cmd(CMD_SAVE_CFG))
        self._log("SAVE_CFG (PIDs permanently written to EEPROM)")
        messagebox.showinfo("EEPROM", "PIDs saved to EEPROM.")

    def _apply_rpm(self):
        rpm = float(self.rpm_var.get())
        self._send(self._cmd(CMD_SET_VEL), struct.pack("<f", rpm))
        self._log(f"SET_VEL {rpm:.1f} rpm")

    def _stop_vel(self):
        self.rpm_var.set(0.0)
        self._send(self._cmd(CMD_SET_VEL), struct.pack("<f", 0.0))
        self._log("SET_VEL 0 (angle hold)")

    def _on_entry_focus(self, idx: int):
        self._focused_entry = idx
        self._params_dirty.add(idx)

    def _on_entry_blur(self, idx: int):
        if self._focused_entry == idx:
            self._focused_entry = None

    def _apply_one(self, idx: int):
        if not self.connected:
            return
        var = self.entries.get(idx)
        if var is None:
            return
        s = var.get().strip()
        try:
            v = float(s)
        except ValueError:
            self._log(f"bad value for param {idx}")
            return
        self._send(self._cmd(CMD_SET_PARAM), bytes([idx & 0xFF]) + struct.pack("<f", v))
        with self.lock:
            self.params[idx] = v
        self._params_dirty.add(idx)
        self._log(f"SET_PARAM {idx} = {v}")

    def _get_params(self):
        self._sync_params = True
        self._params_dirty.clear()
        self._send(self._cmd(CMD_GET_PARAMS))
        self._log("GET_PARAMS")

    def _recal(self):
        if messagebox.askyesno("Recalibrate", "Invalidate FOC calib and reboot?"):
            self._send(self._cmd(CMD_RECALIBRATE))
            self._log("RECALIBRATE")

    def _reboot(self):
        if messagebox.askyesno("Reboot", "Restart the drive? Homing runs again."):
            self._send(self._cmd(CMD_REBOOT))
            self._log("REBOOT")

    def _apply_pids(self):
        if not self.connected:
            messagebox.showinfo("PID", "Connect first")
            return
        n = 0
        for idx, var in self.entries.items():
            s = var.get().strip()
            if not s:
                continue
            try:
                v = float(s)
            except ValueError:
                self._log(f"skip bad param {idx}: {s!r}")
                continue
            self._send(self._cmd(CMD_SET_PARAM), bytes([idx & 0xFF]) + struct.pack("<f", v))
            with self.lock:
                self.params[idx] = v
            self._params_dirty.add(idx)
            n += 1
            time.sleep(0.015)
        en = 1.0 if self.ekf_en.get() else 0.0
        self._send(self._cmd(CMD_SET_PARAM), bytes([P_EKF_EN]) + struct.pack("<f", en))
        with self.lock:
            self.params[P_EKF_EN] = en
        self._sync_params = False
        self._log(f"applied {n} PID fields + EKF en={en:.0f}")

    # --- Reader -------------------------------------------------------------
    def _reader(self):
        while not self.reader_stop.is_set() and self.ser:
            try:
                n = self.ser.in_waiting
                if n:
                    self.rx_buf.extend(self.ser.read(n))
                frames = parse_frames(self.rx_buf)
                now = time.time()
                for cid, pl in frames:
                    self._ingest(cid, pl, now)
                if not frames:
                    time.sleep(0.001)
            except Exception:
                time.sleep(0.02)

    def _ingest(self, cid: int, pl: bytes, now: float):
        with self.lock:
            n = node_from_can_id(cid)
            if n is not None:
                self._scan_hits[n] = now
            if cid == self._rpt(RPT_TELEMETRY) and len(pl) >= 8:
                d, a = struct.unpack("<ff", pl[:8])
                self.desired, self.actual = d, a
                t = now - self._hist_t0
                self.t_hist.append(t)
                self.des_hist.append(d)
                self.act_hist.append(a)
                # keep halls aligned in time if present
                self.hu_hist.append(self.hu)
                self.hv_hist.append(self.hv)
                self.hw_hist.append(self.hw)
            elif cid == self._rpt(RPT_HALLS) and len(pl) >= 6:
                hu, hv, hw = struct.unpack("<HHH", pl[:6])
                self.hu, self.hv, self.hw = float(hu), float(hv), float(hw)
            elif cid == self._rpt(RPT_STATUS) and len(pl) >= 8:
                self.uq, self.vel_rads = struct.unpack("<ff", pl[:8])
            elif cid == self._rpt(RPT_LIMITS) and len(pl) >= 8:
                lo, hi = struct.unpack("<ff", pl[:8])
                if hi > lo and abs(hi - lo) < 400:
                    self.soft_lo, self.soft_hi = lo, hi
            elif cid == self._rpt(RPT_NODE) and len(pl) >= 1:
                self.board_node = int(pl[0]) & 0xFF
                self._scan_hits[self.board_node] = now
            elif cid == self._rpt(RPT_PARAM) and len(pl) >= 5:
                idx = pl[0]
                (val,) = struct.unpack("<f", pl[1:5])
                self.params[idx] = val
            else:
                # Accept RPT_NODE from any node during multi-drive scan
                if (cid - CAN_RPT_BASE) >= 0 and (cid - CAN_RPT_BASE) % CAN_NODE_STRIDE == RPT_NODE and len(pl) >= 1:
                    bn = int(pl[0]) & 0xFF
                    if 0 <= bn <= CAN_NODE_MAX:
                        self._scan_hits[bn] = now
                        self.board_node = bn if bn == self.node else self.board_node

    # --- UI tick ------------------------------------------------------------
    def _ui_tick(self):
        with self.lock:
            des, act = self.desired, self.actual
            uq, vel = self.uq, self.vel_rads
            lo, hi = self.soft_lo, self.soft_hi
            params = dict(self.params)
            t = list(self.t_hist)
            des_h = list(self.des_hist)
            act_h = list(self.act_hist)
            hu_h = list(self.hu_hist)
            hv_h = list(self.hv_hist)
            hw_h = list(self.hw_hist)
            hu, hv, hw = self.hu, self.hv, self.hw

        self.des_lbl.set(f"Desired: {des:+.2f} °")
        self.act_lbl.set(f"Actual:  {act:+.2f} °")
        self.err_lbl.set(f"Error:   {des - act:+.2f} °")
        rpm = vel * 60.0 / (2.0 * math.pi)
        with self.lock:
            bn = self.board_node
        board_txt = f"  board_id={bn}" if bn is not None else ""
        self.stat_lbl.set(
            f"Uq={uq:+.2f} V   vel={rpm:+.1f} rpm   HU={hu:.0f} HV={hv:.0f} HW={hw:.0f} mV"
            f"   CAN node={self.node}{board_txt}"
        )
        self.lim_lbl.set(f"Soft limits: [{lo:.1f}, {hi:.1f}] °")
        self._osc_tick(des, act)

        # Update slider range only when limits change — reconfigure every tick
        # resets ttk.Scale and makes dragging/Set feel broken.
        if hi > lo and (
            self._last_lo is None
            or abs(lo - self._last_lo) > 0.2
            or abs(hi - self._last_hi) > 0.2
        ):
            self._last_lo, self._last_hi = lo, hi
            try:
                cur = float(self.slider.get())
            except Exception:
                cur = 0.0
            self._ignore_slider = True
            try:
                self.slider.configure(from_=lo, to=hi)
                if not self._slider_dragging:
                    self.slider.set(float(np.clip(cur, lo, hi)))
            finally:
                self._ignore_slider = False
            self._limits_set = True
            self._log(f"slider range -> [{lo:.1f}, {hi:.1f}]")
            try:
                self.soft_min_var.set(f"{lo:.1f}")
                self.soft_max_var.set(f"{hi:.1f}")
            except Exception:
                pass
            # Cap osc amp spinbox to soft half-travel
            soft_cap = min(max(hi, 0.0), max(-lo, 0.0))
            try:
                cur_amp = float(self.osc_amp_var.get())
            except Exception:
                cur_amp = soft_cap
            if cur_amp > soft_cap:
                self.osc_amp_var.set(round(soft_cap, 1))
                self._log(f"osc amp capped to ±{soft_cap:.1f}° (soft limits)")

        # Only pull board values into fields after Get Params, and never while editing.
        if self._sync_params:
            for idx, val in params.items():
                if idx == P_EKF_EN:
                    self.ekf_en.set(val != 0.0)
                elif idx in self.entries and idx != self._focused_entry and idx not in self._params_dirty:
                    self.entries[idx].set(f"{val:.5g}")
            # Stop syncing once we have the main PID set
            if {P_VEL_P, P_VEL_I, P_ANGLE_P, P_LPF, P_SLEW, P_ACCEL, P_DECEL}.issubset(params.keys()):
                self._sync_params = False
                self._log("params loaded from board")
                if P_SOFT_MIN in params:
                    self.soft_min_var.set(f"{params[P_SOFT_MIN]:.1f}")
                if P_SOFT_MAX in params:
                    self.soft_max_var.set(f"{params[P_SOFT_MAX]:.1f}")
                if P_PRESTOP in params:
                    self.prestop_var.set(f"{params[P_PRESTOP]:.1f}")

        if len(t) >= 2:
            t0 = t[-1] - WINDOW_S
            # trim view
            xs = np.array(t)
            mask = xs >= t0
            xs = xs[mask] - (t[-1] - WINDOW_S if t[-1] > WINDOW_S else 0)
            def take(arr):
                a = np.array(arr)[mask]
                return a

            self.ln_hu.set_data(xs, take(hu_h))
            self.ln_hv.set_data(xs, take(hv_h))
            self.ln_hw.set_data(xs, take(hw_h))
            self.ln_des.set_data(xs, take(des_h))
            self.ln_act.set_data(xs, take(act_h))
            self.ax_hall.relim()
            self.ax_hall.autoscale_view(scalex=True, scaley=True)
            self.ax_ang.relim()
            self.ax_ang.autoscale_view(scalex=True, scaley=True)
            self.ax_hall.set_xlim(0, WINDOW_S)
            self.ax_ang.set_xlim(0, WINDOW_S)
            self.canvas.draw_idle()

        if self.connected and (time.monotonic() - getattr(self, "_hb_t", 0.0)) >= 0.25:
            self._hb_t = time.monotonic()
            self._send(self._cmd(CMD_HEARTBEAT))

        self.root.after(50, self._ui_tick)


def main():
    root = tk.Tk()
    try:
        style = ttk.Style()
        if "vista" in style.theme_names():
            style.theme_use("vista")
    except Exception:
        pass
    HallControlApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
