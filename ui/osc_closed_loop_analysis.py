#!/usr/bin/env python3
"""Closed-loop oscillation sweep analysis (NO firmware changes).

Oscillates ±amp for amp in 15..90 step 15, ≥10 cycles each.
If an end-stop jam is detected, unload torque (hold actual) and abort.

  python ui/osc_closed_loop_analysis.py
"""
from __future__ import annotations

import csv
import math
import struct
import time
from dataclasses import dataclass, field
from pathlib import Path

import serial
from serial.tools import list_ports

PORT = "COM4"
WS_BAUD = 2_000_000
NODE = 1

CAN_CMD_BASE = 0x140
CAN_RPT_BASE = 0x240
CAN_NODE_STRIDE = 0x10
CMD_SET_ANGLE = 0x01
CMD_SET_VEL = 0x09
CMD_SET_ENABLE = 0x0A
RPT_TELEMETRY = 0x01
RPT_STATUS = 0x04
RPT_LIMITS = 0x05

AMPS = [15, 30, 45, 60, 75, 90]
CYCLES_PER_AMP = 10
DWELL_S = 0.35
SETTLE_ERR_DEG = 4.0
SETTLE_DES_ERR = 2.5
# End-stop: stuck short of target with elevated |Uq| and near-zero velocity
JAM_ERR_DEG = 8.0
JAM_VEL_RAD_S = 0.25
JAM_UQ_ABS = 0.35
JAM_HOLD_S = 0.55
MOVE_TIMEOUT_S = 12.0

OUT_DIR = Path(__file__).resolve().parent
CSV_PATH = OUT_DIR / "osc_analysis_raw.csv"
RPT_PATH = OUT_DIR / "osc_analysis_report.txt"


def config_frame():
    f = bytearray(20)
    f[0], f[1], f[2] = 0xAA, 0x55, 0x12
    f[3], f[4] = 0x03, 0x01
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


@dataclass
class Sample:
    t: float
    des: float
    act: float
    uq: float
    vel: float
    amp: float
    phase: str


@dataclass
class AmpSummary:
    amp: float
    cycles_ok: int = 0
    aborted: bool = False
    abort_reason: str = ""
    peak_act_pos: float = float("-inf")
    peak_act_neg: float = float("inf")
    max_abs_err: float = 0.0
    max_abs_uq: float = 0.0
    mean_abs_err_settle: list = field(default_factory=list)
    overshoot_pos: list = field(default_factory=list)
    overshoot_neg: list = field(default_factory=list)
    settle_times: list = field(default_factory=list)
    notes: list = field(default_factory=list)


class Bus:
    def __init__(self, port: str, node: int):
        self.node = node
        self.ser = serial.Serial(port, WS_BAUD, timeout=0.05)
        self.buf = bytearray()
        self.des = 0.0
        self.act = 0.0
        self.uq = 0.0
        self.vel = 0.0
        self.soft_lo = -90.0
        self.soft_hi = 90.0
        self.t0 = time.time()
        self.have_telem = False
        self.cmd_angle = lambda: CAN_CMD_BASE + node * CAN_NODE_STRIDE + CMD_SET_ANGLE
        self.cmd_vel = lambda: CAN_CMD_BASE + node * CAN_NODE_STRIDE + CMD_SET_VEL
        self.cmd_enable = lambda: CAN_CMD_BASE + node * CAN_NODE_STRIDE + CMD_SET_ENABLE
        self.id_telem = CAN_RPT_BASE + node * CAN_NODE_STRIDE + RPT_TELEMETRY
        self.id_status = CAN_RPT_BASE + node * CAN_NODE_STRIDE + RPT_STATUS
        self.id_limits = CAN_RPT_BASE + node * CAN_NODE_STRIDE + RPT_LIMITS
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        self.ser.write(config_frame())
        time.sleep(0.15)

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def send(self, can_id: int, payload: bytes = b""):
        self.ser.write(pack_tx(can_id, payload))

    def set_angle(self, deg: float):
        self.send(self.cmd_angle(), struct.pack("<f", float(deg)))

    def set_vel_rpm(self, rpm: float):
        self.send(self.cmd_vel(), struct.pack("<f", float(rpm)))

    def set_enable(self, on: bool):
        self.send(self.cmd_enable(), bytes([1 if on else 0]))

    def poll(self):
        chunk = self.ser.read(512)
        if chunk:
            self.buf.extend(chunk)
        for cid, pl in parse_frames(self.buf):
            if cid == self.id_telem and len(pl) >= 8:
                self.des, self.act = struct.unpack("<ff", pl[:8])
                self.have_telem = True
            elif cid == self.id_status and len(pl) >= 8:
                self.uq, self.vel = struct.unpack("<ff", pl[:8])
            elif cid == self.id_limits and len(pl) >= 8:
                self.soft_lo, self.soft_hi = struct.unpack("<ff", pl[:8])

    def now(self) -> float:
        return time.time() - self.t0

    def wait_telem(self, timeout=8.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.poll()
            if self.have_telem:
                return True
            time.sleep(0.01)
        return False


def emergency_disable(bus: Bus, reason: str):
    """Disable motor via ORBIT_CMD_SET_ENABLE 0; also hold actual as backup."""
    print(f"\n*** END-STOP / DISABLE: {reason}")
    bus.set_enable(False)
    for _ in range(10):
        bus.poll()
        bus.set_angle(bus.act)
        time.sleep(0.05)
    bus.set_vel_rpm(0.0)


def go_and_monitor(
    bus: Bus,
    target: float,
    amp: float,
    phase: str,
    samples: list[Sample],
    summary: AmpSummary,
) -> tuple[bool, str]:
    """Move to target. Returns (ok, reason). ok=False on jam/timeout."""
    bus.set_angle(target)
    t_cmd = time.time()
    jam_t0 = None
    settled_t0 = None
    last_progress_act = bus.act
    last_progress_t = time.time()

    while True:
        bus.poll()
        t = bus.now()
        samples.append(Sample(t, bus.des, bus.act, bus.uq, bus.vel, amp, phase))
        summary.peak_act_pos = max(summary.peak_act_pos, bus.act)
        summary.peak_act_neg = min(summary.peak_act_neg, bus.act)
        err = abs(bus.act - target)
        summary.max_abs_err = max(summary.max_abs_err, abs(bus.act - bus.des))
        summary.max_abs_uq = max(summary.max_abs_uq, abs(bus.uq))

        # Progress toward target
        if abs(bus.act - target) < abs(last_progress_act - target) - 0.3:
            last_progress_act = bus.act
            last_progress_t = time.time()

        # Settled?
        if err < SETTLE_ERR_DEG and abs(bus.des - target) < SETTLE_DES_ERR:
            if settled_t0 is None:
                settled_t0 = time.time()
            elif time.time() - settled_t0 >= DWELL_S:
                summary.settle_times.append(time.time() - t_cmd)
                if target > 0:
                    summary.overshoot_pos.append(max(0.0, bus.act - target))
                else:
                    summary.overshoot_neg.append(max(0.0, target - bus.act))
                summary.mean_abs_err_settle.append(err)
                return True, "settled"
        else:
            settled_t0 = None

        # Jam / end-stop: stuck short of target, low vel, elevated Uq
        short = err > JAM_ERR_DEG
        slow = abs(bus.vel) < JAM_VEL_RAD_S
        pushed = abs(bus.uq) >= JAM_UQ_ABS
        no_progress = (time.time() - last_progress_t) > JAM_HOLD_S
        if short and slow and pushed and no_progress:
            if jam_t0 is None:
                jam_t0 = time.time()
            elif time.time() - jam_t0 >= JAM_HOLD_S:
                return False, (
                    f"ENDSTOP jam tgt={target:+.1f} act={bus.act:+.1f} "
                    f"des={bus.des:+.1f} uq={bus.uq:+.3f} vel={bus.vel:+.3f}"
                )
        else:
            jam_t0 = None

        if time.time() - t_cmd > MOVE_TIMEOUT_S:
            return False, (
                f"TIMEOUT tgt={target:+.1f} act={bus.act:+.1f} "
                f"des={bus.des:+.1f} uq={bus.uq:+.3f}"
            )
        time.sleep(0.008)


def run():
    ports = [p.device for p in list_ports.comports()]
    print(f"Ports: {ports}")
    if PORT not in ports:
        raise SystemExit(f"{PORT} not found — connect Waveshare USB-CAN")

    bus = Bus(PORT, NODE)
    samples: list[Sample] = []
    summaries: list[AmpSummary] = []
    irregularities: list[str] = []
    aborted_global = False
    amps_run: list[int] = []

    try:
        print("Waiting for telemetry...")
        if not bus.wait_telem(15.0):
            raise SystemExit("No telemetry — is the drive powered and CAN node 1?")
        # Wait for soft-limit report after home
        t_lim = time.time()
        while time.time() - t_lim < 4.0:
            bus.poll()
            time.sleep(0.02)
        soft_cap = min(max(bus.soft_hi, 0.0), max(-bus.soft_lo, 0.0))
        amps_run = [a for a in AMPS if a <= soft_cap + 0.5]
        if not amps_run:
            amps_run = [max(1, int(soft_cap))]
        print(
            f"Live: act={bus.act:+.2f} des={bus.des:+.2f} "
            f"soft=[{bus.soft_lo:.1f},{bus.soft_hi:.1f}] soft_cap=±{soft_cap:.1f} "
            f"uq={bus.uq:+.3f}"
        )
        print(f"Test amps within soft limits: {amps_run}")
        bus.set_enable(True)
        time.sleep(0.3)

        print("Go 0° and settle...")
        ok, reason = go_and_monitor(bus, 0.0, 0.0, "home0", samples, AmpSummary(0.0))
        if not ok:
            emergency_disable(bus, reason)
            irregularities.append(f"Failed initial go-0: {reason}")
            aborted_global = True

        for amp in amps_run:
            if aborted_global:
                break
            summary = AmpSummary(amp=float(amp))
            print(f"\n=== AMP ±{amp}°  ({CYCLES_PER_AMP} cycles) ===")
            # Start toward +amp
            target = float(amp)
            for cyc in range(CYCLES_PER_AMP):
                for sign, label in ((+1, f"c{cyc}+"), (-1, f"c{cyc}-")):
                    target = sign * float(amp)
                    ok, reason = go_and_monitor(
                        bus, target, float(amp), label, samples, summary
                    )
                    if not ok:
                        summary.aborted = True
                        summary.abort_reason = reason
                        irregularities.append(f"±{amp}° cycle {cyc}: {reason}")
                        emergency_disable(bus, reason)
                        aborted_global = True
                        break
                if summary.aborted:
                    break
                summary.cycles_ok += 1
                print(f"  cycle {cyc + 1}/{CYCLES_PER_AMP} OK  "
                      f"peak_act=[{summary.peak_act_neg:+.1f},{summary.peak_act_pos:+.1f}] "
                      f"max|uq|={summary.max_abs_uq:.3f}")

            # Peak reach shortfall vs commanded amp
            if summary.peak_act_pos < float("-inf"):
                short_p = amp - summary.peak_act_pos
                short_n = amp + summary.peak_act_neg
                if short_p > 5.0:
                    note = f"±{amp}: +side peak only {summary.peak_act_pos:+.1f} (short {short_p:.1f}°)"
                    summary.notes.append(note)
                    irregularities.append(note)
                if short_n > 5.0:
                    note = f"±{amp}: -side peak only {summary.peak_act_neg:+.1f} (short {short_n:.1f}°)"
                    summary.notes.append(note)
                    irregularities.append(note)
                asym = abs(summary.peak_act_pos + summary.peak_act_neg)
                if asym > 4.0 and summary.cycles_ok > 0:
                    note = f"±{amp}: CW/CCW peak asymmetry {asym:.1f}°"
                    summary.notes.append(note)
                    irregularities.append(note)
            if summary.max_abs_uq > 1.5:
                note = f"±{amp}: high |Uq| peak {summary.max_abs_uq:.3f}"
                summary.notes.append(note)
                irregularities.append(note)

            summaries.append(summary)
            if summary.aborted:
                break
            # Return toward 0 between amps
            go_and_monitor(bus, 0.0, float(amp), "ret0", samples, summary)

        if not aborted_global:
            bus.set_angle(0.0)
            time.sleep(0.5)

    finally:
        # Write CSV
        with CSV_PATH.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "desired_deg", "actual_deg", "uq", "vel_rads", "amp", "phase"])
            for s in samples:
                w.writerow(
                    [
                        f"{s.t:.4f}",
                        f"{s.des:.3f}",
                        f"{s.act:.3f}",
                        f"{s.uq:.4f}",
                        f"{s.vel:.4f}",
                        f"{s.amp:.1f}",
                        s.phase,
                    ]
                )

        lines = []
        lines.append("OrbitDrive_HALL_5 closed-loop oscillation analysis")
        lines.append(f"time={time.strftime('%Y-%m-%d %H:%M:%S')}")
        lines.append(f"amps_planned={AMPS}  amps_run={amps_run}  cycles_each={CYCLES_PER_AMP}")
        lines.append(f"soft_limits_reported=[{bus.soft_lo:.1f},{bus.soft_hi:.1f}]")
        lines.append(f"raw_csv={CSV_PATH}")
        lines.append("")
        lines.append("--- Per-amplitude summary ---")
        for s in summaries:
            lines.append(f"AMP ±{s.amp:.0f}°")
            lines.append(f"  cycles_completed={s.cycles_ok}/{CYCLES_PER_AMP}")
            lines.append(f"  aborted={s.aborted}  {s.abort_reason}")
            if s.peak_act_pos > float("-inf"):
                lines.append(
                    f"  peak_actual=[{s.peak_act_neg:+.2f}, {s.peak_act_pos:+.2f}]"
                )
            lines.append(f"  max_abs_tracking_err={s.max_abs_err:.2f}°")
            lines.append(f"  max_abs_uq={s.max_abs_uq:.4f}")
            if s.settle_times:
                lines.append(
                    f"  settle_s mean={sum(s.settle_times)/len(s.settle_times):.2f} "
                    f"max={max(s.settle_times):.2f}"
                )
            if s.overshoot_pos:
                lines.append(
                    f"  overshoot_+ mean={sum(s.overshoot_pos)/len(s.overshoot_pos):.2f} "
                    f"max={max(s.overshoot_pos):.2f}"
                )
            if s.overshoot_neg:
                lines.append(
                    f"  overshoot_- mean={sum(s.overshoot_neg)/len(s.overshoot_neg):.2f} "
                    f"max={max(s.overshoot_neg):.2f}"
                )
            for n in s.notes:
                lines.append(f"  NOTE: {n}")
            lines.append("")

        lines.append("--- Irregularities ---")
        if irregularities:
            for i, ir in enumerate(irregularities, 1):
                lines.append(f"{i}. {ir}")
        else:
            lines.append("None flagged by automated checks.")
        lines.append("")
        lines.append(
            "On jam: firmware trips ENDSTOP FAULT + motor.disable(); "
            "script also sends ORBIT_CMD_SET_ENABLE 0."
        )

        text = "\n".join(lines) + "\n"
        RPT_PATH.write_text(text, encoding="utf-8")
        print("\n" + text)
        bus.close()


if __name__ == "__main__":
    run()
