#!/usr/bin/env python3
"""OrbitDrive dual-axis motion simulator (pitch + roll over Waveshare USB-CAN).

  Pitch / tilt : CAN node 1   soft ~[-135, +135]
  Roll         : CAN node 2   soft ~[-45, +45]

  python ui/dual_axis_sim.py
"""
from __future__ import annotations

import math
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

CAN_CMD_BASE = 0x140
CAN_RPT_BASE = 0x240
CAN_NODE_STRIDE = 0x10
CMD_SET_ANGLE = 0x01
CMD_SET_ENABLE = 0x0A
RPT_TELEMETRY = 0x01
RPT_LIMITS = 0x05
RPT_NODE = 0x06

WS_BAUD = 2_000_000
DEFAULT_PORT = "COM12"
PITCH_NODE = 1
ROLL_NODE = 2

# Safe demo amplitudes (inside typical soft windows)
PITCH_LO, PITCH_HI = -35.0, 70.0
ROLL_LO, ROLL_HI = -35.0, 35.0


def cmd_id(node: int, off: int) -> int:
    return CAN_CMD_BASE + int(node) * CAN_NODE_STRIDE + off


def rpt_id(node: int, off: int) -> int:
    return CAN_RPT_BASE + int(node) * CAN_NODE_STRIDE + off


def waveshare_cfg() -> bytes:
    f = bytearray(20)
    f[0], f[1], f[2] = 0xAA, 0x55, 0x12
    f[3], f[4] = 0x03, 0x01  # 500k
    f[19] = sum(f[2:19]) & 0xFF
    return bytes(f)


def tx_frame(can_id: int, payload: bytes = b"") -> bytes:
    dlc = len(payload) & 0x0F
    return bytes([0xAA, 0xC0 | dlc, can_id & 0xFF, (can_id >> 8) & 0xFF]) + payload + bytes([0x55])


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
        cid = int.from_bytes(bytes(buf[2 : 2 + id_len]), "little")
        pl = bytes(buf[2 + id_len : 2 + id_len + dlc])
        out.append((cid, pl))
        del buf[:total]
    return out


class DualAxisSim:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("OrbitDrive — Dual axis sim (pitch + roll)")
        self.root.geometry("980x720")

        self.ser: serial.Serial | None = None
        self.connected = False
        self.lock = threading.Lock()
        self.rx_buf = bytearray()
        self.reader_stop = threading.Event()
        self.reader: threading.Thread | None = None

        self.pitch_des = 0.0
        self.pitch_act = 0.0
        self.roll_des = 0.0
        self.roll_act = 0.0
        self.pitch_lo, self.pitch_hi = PITCH_LO, PITCH_HI
        self.roll_lo, self.roll_hi = ROLL_LO, ROLL_HI

        self.sim_on = False
        self.sim_mode = tk.StringVar(value="lissajous")
        self.sim_t0 = 0.0
        self.period_s = tk.DoubleVar(value=2.5)
        self.pitch_amp = tk.DoubleVar(value=25.0)
        self.roll_amp = tk.DoubleVar(value=20.0)
        self._cmd_hz = 60.0
        self._last_cmd_t = 0.0
        self._last_p_cmd = None
        self._last_r_cmd = None

        t0 = time.time()
        self._hist_t0 = t0
        self.t_hist: deque[float] = deque(maxlen=2500)
        self.pd_hist: deque[float] = deque(maxlen=2500)
        self.pa_hist: deque[float] = deque(maxlen=2500)
        self.rd_hist: deque[float] = deque(maxlen=2500)
        self.ra_hist: deque[float] = deque(maxlen=2500)

        self._build()
        self.root.after(40, self._tick)
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.root.after(250, self._connect)

    def _build(self):
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill=tk.X)
        ttk.Label(top, text="Port").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        ttk.Combobox(
            top, textvariable=self.port_var, width=10, values=self._ports()
        ).pack(side=tk.LEFT, padx=4)
        self.btn = ttk.Button(top, text="Connect", command=self._toggle)
        self.btn.pack(side=tk.LEFT, padx=6)
        self.status = tk.StringVar(value="Disconnected")
        ttk.Label(top, textvariable=self.status).pack(side=tk.LEFT, padx=8)
        ttk.Label(
            top, text=f"Pitch=node {PITCH_NODE}   Roll=node {ROLL_NODE}", foreground="#555"
        ).pack(side=tk.RIGHT)

        live = ttk.LabelFrame(self.root, text="Telemetry", padding=8)
        live.pack(fill=tk.X, padx=8)
        self.pitch_lbl = tk.StringVar(value="Pitch des/act: -- / --")
        self.roll_lbl = tk.StringVar(value="Roll  des/act: -- / --")
        ttk.Label(live, textvariable=self.pitch_lbl, foreground="#c9a227").pack(anchor=tk.W)
        ttk.Label(live, textvariable=self.roll_lbl, foreground="#3b82f6").pack(anchor=tk.W)

        ctrl = ttk.LabelFrame(self.root, text="Coordinated motion", padding=8)
        ctrl.pack(fill=tk.X, padx=8, pady=6)
        row = ttk.Frame(ctrl)
        row.pack(fill=tk.X)
        ttk.Label(row, text="Pattern").pack(side=tk.LEFT)
        ttk.Combobox(
            row,
            textvariable=self.sim_mode,
            width=16,
            values=("lissajous", "opposite", "same", "square", "home"),
            state="readonly",
        ).pack(side=tk.LEFT, padx=6)
        ttk.Label(row, text="Period s").pack(side=tk.LEFT, padx=(10, 2))
        ttk.Spinbox(row, from_=1.5, to=20.0, increment=0.5, width=5, textvariable=self.period_s).pack(
            side=tk.LEFT
        )
        ttk.Label(row, text="Pitch amp").pack(side=tk.LEFT, padx=(10, 2))
        ttk.Spinbox(row, from_=5.0, to=70.0, increment=5.0, width=5, textvariable=self.pitch_amp).pack(
            side=tk.LEFT
        )
        ttk.Label(row, text="Roll amp").pack(side=tk.LEFT, padx=(10, 2))
        ttk.Spinbox(row, from_=5.0, to=40.0, increment=5.0, width=5, textvariable=self.roll_amp).pack(
            side=tk.LEFT
        )

        btns = ttk.Frame(ctrl)
        btns.pack(fill=tk.X, pady=6)
        ttk.Button(btns, text="Go 0 / 0", command=self._go_home).pack(side=tk.LEFT, padx=2)
        ttk.Button(btns, text="Enable both", command=lambda: self._enable_both(True)).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(btns, text="Disable both", command=lambda: self._enable_both(False)).pack(
            side=tk.LEFT, padx=2
        )
        self.sim_btn = ttk.Button(btns, text="Start sim", command=self._toggle_sim)
        self.sim_btn.pack(side=tk.LEFT, padx=12)
        ttk.Label(
            ctrl,
            text="Streams setpoints @ 50 Hz (smooth). Avoid 'square' if you want continuous motion. "
            "Physical lag also comes from 0.25 A current limit + heavy vel LPF on the drives.",
            foreground="#666",
        ).pack(anchor=tk.W)

        man = ttk.Frame(ctrl)
        man.pack(fill=tk.X, pady=4)
        ttk.Label(man, text="Manual pitch").pack(side=tk.LEFT)
        self.man_p = tk.StringVar(value="0")
        ttk.Entry(man, textvariable=self.man_p, width=7).pack(side=tk.LEFT, padx=2)
        ttk.Label(man, text="roll").pack(side=tk.LEFT, padx=(8, 2))
        self.man_r = tk.StringVar(value="0")
        ttk.Entry(man, textvariable=self.man_r, width=7).pack(side=tk.LEFT, padx=2)
        ttk.Button(man, text="Set both", command=self._manual_set).pack(side=tk.LEFT, padx=6)

        fig = Figure(figsize=(9.2, 4.6), dpi=100)
        self.ax = fig.add_subplot(111)
        self.ax.set_title("Pitch & Roll desired vs actual")
        self.ax.set_xlabel("t (s)")
        self.ax.set_ylabel("deg")
        (self.ln_pd,) = self.ax.plot([], [], label="Pitch des", color="#f1c40f", lw=1.3)
        (self.ln_pa,) = self.ax.plot([], [], label="Pitch act", color="#e67e22", lw=1.3)
        (self.ln_rd,) = self.ax.plot([], [], label="Roll des", color="#3498db", lw=1.3)
        (self.ln_ra,) = self.ax.plot([], [], label="Roll act", color="#9b59b6", lw=1.3)
        self.ax.legend(loc="upper right", fontsize=8)
        fig.tight_layout()
        canvas = FigureCanvasTkAgg(fig, master=self.root)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=8, pady=4)
        self.canvas = canvas
        self.fig = fig

        self.log = tk.Text(self.root, height=6, font=("Consolas", 9))
        self.log.pack(fill=tk.X, padx=8, pady=4)

    def _ports(self):
        return [p.device for p in list_ports.comports()] or [DEFAULT_PORT]

    def _log(self, msg: str):
        self.log.insert(tk.END, msg + "\n")
        self.log.see(tk.END)

    def _send(self, can_id: int, payload: bytes = b""):
        if not self.ser:
            return
        try:
            self.ser.write(tx_frame(can_id, payload))
        except Exception as e:
            self._log(f"TX err: {e}")

    def _set_angle(self, node: int, deg: float, *, once: bool = True):
        if node == PITCH_NODE:
            lo, hi = self.pitch_lo, self.pitch_hi
        else:
            lo, hi = self.roll_lo, self.roll_hi
        deg = float(np.clip(deg, lo, hi))
        pl = struct.pack("<f", deg)
        cid = cmd_id(node, CMD_SET_ANGLE)
        self._send(cid, pl)
        if not once:
            self._send(cid, pl)
        return deg

    def _set_both(self, pitch: float, roll: float, *, log: bool = True):
        p = self._set_angle(PITCH_NODE, pitch, once=False)
        r = self._set_angle(ROLL_NODE, roll, once=False)
        self._last_p_cmd, self._last_r_cmd = p, r
        if log:
            self._log(f"SET pitch={p:+.1f}  roll={r:+.1f}")

    def _stream_both(self, pitch: float, roll: float):
        """High-rate path: one frame per axis, skip if unchanged."""
        p = float(np.clip(pitch, self.pitch_lo, self.pitch_hi))
        r = float(np.clip(roll, self.roll_lo, self.roll_hi))
        if self._last_p_cmd is None or abs(p - self._last_p_cmd) > 0.02:
            self._send(cmd_id(PITCH_NODE, CMD_SET_ANGLE), struct.pack("<f", p))
            self._last_p_cmd = p
        if self._last_r_cmd is None or abs(r - self._last_r_cmd) > 0.02:
            self._send(cmd_id(ROLL_NODE, CMD_SET_ANGLE), struct.pack("<f", r))
            self._last_r_cmd = r

    def _go_home(self):
        self.sim_on = False
        self.sim_btn.configure(text="Start sim")
        self._set_both(0.0, 0.0)

    def _enable_both(self, on: bool):
        pl = bytes([1 if on else 0])
        self._send(cmd_id(PITCH_NODE, CMD_SET_ENABLE), pl)
        self._send(cmd_id(ROLL_NODE, CMD_SET_ENABLE), pl)
        self._log("ENABLE both" if on else "DISABLE both")

    def _manual_set(self):
        try:
            p = float(self.man_p.get())
            r = float(self.man_r.get())
        except ValueError:
            messagebox.showerror("Manual", "Enter numeric pitch/roll")
            return
        self.sim_on = False
        self.sim_btn.configure(text="Start sim")
        self._set_both(p, r)

    def _toggle_sim(self):
        if not self.connected:
            messagebox.showinfo("Sim", "Connect first")
            return
        self.sim_on = not self.sim_on
        self.sim_t0 = time.time()
        self.sim_btn.configure(text="Stop sim" if self.sim_on else "Start sim")
        self._log(f"SIM {'ON' if self.sim_on else 'OFF'} mode={self.sim_mode.get()}")
        if not self.sim_on:
            self._set_both(0.0, 0.0)

    def _sim_targets(self, t: float) -> tuple[float, float]:
        mode = self.sim_mode.get()
        T = max(1.5, float(self.period_s.get()))
        ap = float(self.pitch_amp.get())
        ar = float(self.roll_amp.get())
        w = 2.0 * math.pi / T

        if mode == "home":
            return 0.0, 0.0
        if mode == "same":
            return ap * math.sin(w * t), ar * math.sin(w * t)
        if mode == "opposite":
            return ap * math.sin(w * t), -ar * math.sin(w * t)
        if mode == "square":
            # Cosine ease between corners (less brutal than hard steps)
            u = (t % T) / T
            corners = [
                (ap, ar),
                (ap, -ar),
                (-ap * 0.6, -ar),
                (-ap * 0.6, ar),
            ]
            seg = min(3, int(u * 4.0))
            local = (u * 4.0) - seg
            # smoothstep
            s = local * local * (3.0 - 2.0 * local)
            a = corners[seg]
            b = corners[(seg + 1) % 4]
            return a[0] + (b[0] - a[0]) * s, a[1] + (b[1] - a[1]) * s
        # lissajous: pitch 1x, roll 2x
        return ap * math.sin(w * t), ar * math.sin(2.0 * w * t)

    def _toggle(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        if self.connected:
            return
        port = self.port_var.get().strip()
        try:
            self.ser = serial.Serial(port, WS_BAUD, timeout=0.05)
            self.ser.reset_input_buffer()
            self.ser.write(waveshare_cfg())
            time.sleep(0.15)
            self.connected = True
            self.btn.configure(text="Disconnect")
            self.status.set(f"Connected {port}")
            self.reader_stop.clear()
            self.reader = threading.Thread(target=self._reader, daemon=True)
            self.reader.start()
            self._log(f"Connected {port} — commanding pitch@{PITCH_NODE} + roll@{ROLL_NODE}")
            self._enable_both(True)
            # Push fast angle profile on both nodes (RAM) for smoother tracking
            for node in (PITCH_NODE, ROLL_NODE):
                # PARAM_SLEW=14, ACCEL=15, DECEL=16
                for idx, val in ((14, 1500.0), (15, 4000.0), (16, 4000.0)):
                    self._send(
                        cmd_id(node, 0x06),
                        bytes([idx]) + struct.pack("<f", val),
                    )
            self._set_both(0.0, 0.0)
            self._log("Applied slew=1500 accel/decel=4000 on both nodes (RAM)")
        except Exception as e:
            self.status.set("Connect failed")
            messagebox.showerror("CAN", str(e))

    def _disconnect(self):
        self.sim_on = False
        self.sim_btn.configure(text="Start sim")
        self.reader_stop.set()
        if self.reader:
            self.reader.join(timeout=0.5)
            self.reader = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.connected = False
        self.btn.configure(text="Connect")
        self.status.set("Disconnected")

    def _close(self):
        self._disconnect()
        self.root.destroy()

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
            if cid == rpt_id(PITCH_NODE, RPT_TELEMETRY) and len(pl) >= 8:
                d, a = struct.unpack("<ff", pl[:8])
                self.pitch_des, self.pitch_act = d, a
            elif cid == rpt_id(ROLL_NODE, RPT_TELEMETRY) and len(pl) >= 8:
                d, a = struct.unpack("<ff", pl[:8])
                self.roll_des, self.roll_act = d, a
            elif cid == rpt_id(PITCH_NODE, RPT_LIMITS) and len(pl) >= 8:
                lo, hi = struct.unpack("<ff", pl[:8])
                if hi > lo:
                    self.pitch_lo = max(lo, PITCH_LO - 5)
                    self.pitch_hi = min(hi, PITCH_HI + 5)
            elif cid == rpt_id(ROLL_NODE, RPT_LIMITS) and len(pl) >= 8:
                lo, hi = struct.unpack("<ff", pl[:8])
                if hi > lo:
                    self.roll_lo = max(lo, ROLL_LO - 5)
                    self.roll_hi = min(hi, ROLL_HI + 5)
            else:
                return
            t = now - self._hist_t0
            self.t_hist.append(t)
            self.pd_hist.append(self.pitch_des)
            self.pa_hist.append(self.pitch_act)
            self.rd_hist.append(self.roll_des)
            self.ra_hist.append(self.roll_act)

    def _tick(self):
        now = time.time()
        if self.sim_on and self.connected:
            if (now - self._last_cmd_t) >= (1.0 / self._cmd_hz):
                self._last_cmd_t = now
                t = now - self.sim_t0
                p, r = self._sim_targets(t)
                self._stream_both(p, r)

        with self.lock:
            pd, pa = self.pitch_des, self.pitch_act
            rd, ra = self.roll_des, self.roll_act
            t = list(self.t_hist)
            pd_h = list(self.pd_hist)
            pa_h = list(self.pa_hist)
            rd_h = list(self.rd_hist)
            ra_h = list(self.ra_hist)

        self.pitch_lbl.set(f"Pitch des/act: {pd:+.2f} / {pa:+.2f} °   err={pd - pa:+.2f}")
        self.roll_lbl.set(f"Roll  des/act: {rd:+.2f} / {ra:+.2f} °   err={rd - ra:+.2f}")

        if len(t) >= 2:
            t0 = t[-1] - 12.0
            xs = np.array(t)
            m = xs >= t0
            xs = xs[m] - (t[-1] - 12.0 if t[-1] > 12 else 0)

            def take(arr):
                return np.array(arr)[m]

            self.ln_pd.set_data(xs, take(pd_h))
            self.ln_pa.set_data(xs, take(pa_h))
            self.ln_rd.set_data(xs, take(rd_h))
            self.ln_ra.set_data(xs, take(ra_h))
            self.ax.relim()
            self.ax.autoscale_view()
            self.canvas.draw_idle()

        self.root.after(20, self._tick)

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    DualAxisSim().run()
