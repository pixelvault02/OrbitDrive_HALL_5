#!/usr/bin/env python3
"""Closed-loop smooth-motion tune for OrbitDrive_HALL_5 (roll) over Waveshare CAN.

Reads desired/actual telemetry, scores overshoot / RMSE / jerk / settle,
searches slew+accel/decel+soft PID, applies best set, optionally SAVE_CFG.

  python ui/smooth_can_tune.py
  python ui/smooth_can_tune.py --port COM12 --node 2 --save
"""
from __future__ import annotations

import argparse
import math
import struct
import time
from pathlib import Path

import serial

WS_BAUD = 2_000_000
CAN_CMD_BASE = 0x140
CAN_RPT_BASE = 0x240
CAN_STRIDE = 0x10

CMD_SET_ANGLE = 0x01
CMD_SAVE_CFG = 0x03
CMD_SET_PARAM = 0x06
CMD_SET_ENABLE = 0x0A

RPT_TELEMETRY = 0x01
RPT_STATUS = 0x04
RPT_LIMITS = 0x05

P_VEL_P, P_VEL_I, P_VEL_D, P_VEL_RAMP = 0, 1, 2, 3
P_ANGLE_P, P_LPF = 4, 5
P_SLEW, P_ACCEL, P_DECEL = 14, 15, 16

OUT = Path(__file__).with_name("smooth_tune_report.txt")


def cfg_frame():
    f = bytearray(20)
    f[0], f[1], f[2] = 0xAA, 0x55, 0x12
    f[3], f[4] = 0x03, 0x01
    f[19] = sum(f[2:19]) & 0xFF
    return bytes(f)


def tx(cid: int, pl: bytes = b""):
    dlc = len(pl) & 0x0F
    return bytes([0xAA, 0xC0 | dlc, cid & 0xFF, (cid >> 8) & 0xFF]) + pl + bytes([0x55])


def parse(buf: bytearray):
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


class Bus:
    def __init__(self, port: str, node: int):
        self.node = node
        self.cmd = CAN_CMD_BASE + node * CAN_STRIDE
        self.rpt = CAN_RPT_BASE + node * CAN_STRIDE
        self.ser = serial.Serial(port, WS_BAUD, timeout=0.02)
        self.buf = bytearray()
        self.ser.reset_input_buffer()
        self.ser.write(cfg_frame())
        time.sleep(0.15)
        self.des = 0.0
        self.act = 0.0
        self.uq = 0.0
        self.vel = 0.0
        self.soft_lo = -40.0
        self.soft_hi = 40.0
        self.rows: list[tuple[float, float, float, float, float]] = []

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def send(self, off: int, pl: bytes = b""):
        self.ser.write(tx(self.cmd + off, pl))

    def set_param(self, idx: int, v: float):
        self.send(CMD_SET_PARAM, bytes([idx & 0xFF]) + struct.pack("<f", float(v)))
        time.sleep(0.012)

    def set_angle(self, deg: float):
        self.send(CMD_SET_ANGLE, struct.pack("<f", float(deg)))
        time.sleep(0.002)
        self.send(CMD_SET_ANGLE, struct.pack("<f", float(deg)))

    def enable(self):
        self.send(CMD_SET_ENABLE, bytes([1]))
        time.sleep(0.05)

    def save(self):
        self.send(CMD_SAVE_CFG)
        time.sleep(0.1)

    def pump(self, dt: float):
        t_end = time.time() + dt
        t0 = time.time()
        while time.time() < t_end:
            n = self.ser.in_waiting
            if n:
                self.buf.extend(self.ser.read(n))
            for cid, pl in parse(self.buf):
                if cid == self.rpt + RPT_TELEMETRY and len(pl) >= 8:
                    d, a = struct.unpack("<ff", pl[:8])
                    self.des, self.act = d, a
                    self.rows.append((time.time() - t0, d, a, self.uq, self.vel))
                elif cid == self.rpt + RPT_STATUS and len(pl) >= 8:
                    self.uq, self.vel = struct.unpack("<ff", pl[:8])
                elif cid == self.rpt + RPT_LIMITS and len(pl) >= 8:
                    lo, hi = struct.unpack("<ff", pl[:8])
                    if hi > lo:
                        self.soft_lo, self.soft_hi = lo, hi
            time.sleep(0.002)

    def clear_rows(self):
        self.rows.clear()


def score_rows(rows, target: float) -> dict:
    if len(rows) < 20:
        return {"score": 1e9, "rmse": 99, "over": 99, "jerk": 99, "settle": 99, "n": len(rows)}
    acts = [r[2] for r in rows]
    ts = [r[0] for r in rows]
    # ignore first 0.15 s of command transit
    use = [(t, a) for t, a in zip(ts, acts) if t >= 0.15]
    if len(use) < 10:
        use = list(zip(ts, acts))
    err = [a - target for _, a in use]
    rmse = math.sqrt(sum(e * e for e in err) / len(err))
    # overshoot beyond target in the commanded direction from start
    start = acts[0]
    direction = 1.0 if target >= start else -1.0
    over = 0.0
    for a in acts:
        signed = (a - target) * direction
        if signed > over:
            over = signed
    # jerk proxy: 2nd difference of actual
    jerks = []
    for i in range(2, len(use)):
        dt1 = max(1e-3, use[i][0] - use[i - 1][0])
        dt0 = max(1e-3, use[i - 1][0] - use[i - 2][0])
        v1 = (use[i][1] - use[i - 1][1]) / dt1
        v0 = (use[i - 1][1] - use[i - 2][1]) / dt0
        jerks.append(abs(v1 - v0) / max(1e-3, 0.5 * (dt0 + dt1)))
    jerk = sorted(jerks)[int(0.9 * (len(jerks) - 1))] if jerks else 99.0
    # settle: first time |err|<1.5 and stays
    settle = use[-1][0]
    for i, (t, a) in enumerate(use):
        if abs(a - target) < 1.5:
            ok = all(abs(use[j][1] - target) < 2.0 for j in range(i, min(i + 25, len(use))))
            if ok:
                settle = t
                break
    # lower is better
    score = 3.0 * rmse + 4.0 * over + 0.002 * jerk + 0.4 * settle
    return {
        "score": score,
        "rmse": rmse,
        "over": over,
        "jerk": jerk,
        "settle": settle,
        "n": len(rows),
    }


def apply_gains(bus: Bus, g: dict):
    order = [
        (P_VEL_P, "vel_p"),
        (P_VEL_I, "vel_i"),
        (P_VEL_D, "vel_d"),
        (P_VEL_RAMP, "vel_ramp"),
        (P_ANGLE_P, "angle_p"),
        (P_LPF, "lpf"),
        (P_SLEW, "slew"),
        (P_ACCEL, "accel"),
        (P_DECEL, "decel"),
    ]
    for idx, key in order:
        bus.set_param(idx, g[key])
    time.sleep(0.05)


def trial(bus: Bus, g: dict, amp: float, dwell: float) -> dict:
    apply_gains(bus, g)
    lo, hi = bus.soft_lo, bus.soft_hi
    # keep well inside soft limits
    amp = min(amp, 0.55 * min(abs(lo), abs(hi), 35.0))
    amp = max(4.0, amp)
    targets = [0.0, +amp, 0.0, -amp, 0.0]
    scores = []
    for tgt in targets:
        bus.clear_rows()
        bus.set_angle(tgt)
        bus.pump(dwell)
        sc = score_rows(bus.rows, tgt)
        scores.append(sc)
        print(
            f"    -> {tgt:+5.1f}°  rmse={sc['rmse']:.2f} over={sc['over']:.2f} "
            f"jerk={sc['jerk']:.0f} settle={sc['settle']:.2f}s  score={sc['score']:.2f}"
        )
    avg = {
        "score": sum(s["score"] for s in scores) / len(scores),
        "rmse": sum(s["rmse"] for s in scores) / len(scores),
        "over": sum(s["over"] for s in scores) / len(scores),
        "jerk": sum(s["jerk"] for s in scores) / len(scores),
        "settle": sum(s["settle"] for s in scores) / len(scores),
    }
    return avg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM12")
    ap.add_argument("--node", type=int, default=2)
    ap.add_argument("--amp", type=float, default=12.0)
    ap.add_argument("--dwell", type=float, default=2.8)
    ap.add_argument("--save", action="store_true")
    args = ap.parse_args()

    bus = Bus(args.port, args.node)
    try:
        bus.enable()
        bus.pump(0.8)
        print(f"node={args.node} soft=[{bus.soft_lo:.1f},{bus.soft_hi:.1f}] act={bus.act:+.2f}")
        if not (bus.soft_hi > bus.soft_lo):
            print("No soft limits heard — is the drive powered / on node 2?")
            return 1

        # Hold current first
        bus.set_angle(bus.act)
        bus.pump(0.6)

        seeds = [
            # smooth profile pack for roll
            dict(
                vel_p=0.010,
                vel_i=0.05,
                vel_d=0.00010,
                vel_ramp=40.0,
                angle_p=18.0,
                lpf=0.85,
                slew=60.0,
                accel=350.0,
                decel=400.0,
            ),
            dict(
                vel_p=0.012,
                vel_i=0.06,
                vel_d=0.00012,
                vel_ramp=50.0,
                angle_p=22.0,
                lpf=0.90,
                slew=45.0,
                accel=250.0,
                decel=300.0,
            ),
            dict(
                vel_p=0.008,
                vel_i=0.04,
                vel_d=0.00008,
                vel_ramp=30.0,
                angle_p=15.0,
                lpf=1.0,
                slew=35.0,
                accel=180.0,
                decel=220.0,
            ),
            dict(
                vel_p=0.013,
                vel_i=0.08,
                vel_d=0.00014,
                vel_ramp=70.0,
                angle_p=25.0,
                lpf=0.75,
                slew=80.0,
                accel=500.0,
                decel=550.0,
            ),
        ]

        results = []
        for i, g in enumerate(seeds):
            print(f"\n=== candidate {i + 1}/{len(seeds)}  slew={g['slew']} accel={g['accel']} angleP={g['angle_p']} ===")
            sc = trial(bus, g, args.amp, args.dwell)
            print(
                f"  AVG score={sc['score']:.2f} rmse={sc['rmse']:.2f} over={sc['over']:.2f} "
                f"jerk={sc['jerk']:.0f} settle={sc['settle']:.2f}"
            )
            results.append((sc["score"], g, sc))

        results.sort(key=lambda x: x[0])
        best_score, best_g, best_sc = results[0]
        print("\n=== refining around best ===")

        refined = []
        for slew in (best_g["slew"] * 0.75, best_g["slew"], best_g["slew"] * 1.25):
            for acc in (best_g["accel"] * 0.7, best_g["accel"], best_g["accel"] * 1.3):
                g = dict(best_g)
                g["slew"] = max(20.0, min(120.0, slew))
                g["accel"] = max(100.0, min(800.0, acc))
                g["decel"] = g["accel"] * 1.15
                print(f"\n--- refine slew={g['slew']:.0f} accel={g['accel']:.0f} ---")
                sc = trial(bus, g, args.amp, args.dwell)
                print(f"  AVG score={sc['score']:.2f}")
                refined.append((sc["score"], g, sc))

        refined.sort(key=lambda x: x[0])
        if refined[0][0] < best_score:
            best_score, best_g, best_sc = refined[0]

        print("\n=== BEST ===")
        for k, v in best_g.items():
            print(f"  {k}={v}")
        print(
            f"  score={best_sc['score']:.2f} rmse={best_sc['rmse']:.2f} "
            f"over={best_sc['over']:.2f} jerk={best_sc['jerk']:.0f}"
        )

        apply_gains(bus, best_g)
        bus.set_angle(0.0)
        bus.pump(2.0)
        if args.save:
            bus.save()
            print("SAVE_CFG sent")

        lines = [
            "OrbitDrive_HALL_5 smooth CAN closed-loop tune",
            f"port={args.port} node={args.node}",
            f"soft=[{bus.soft_lo:.1f},{bus.soft_hi:.1f}]",
            "BEST gains:",
        ]
        for k, v in best_g.items():
            lines.append(f"  {k}={v}")
        lines.append(
            f"metrics: score={best_sc['score']:.3f} rmse={best_sc['rmse']:.3f} "
            f"over={best_sc['over']:.3f} jerk={best_sc['jerk']:.1f} settle={best_sc['settle']:.2f}"
        )
        OUT.write_text("\n".join(lines) + "\n", encoding="utf-8")
        print(f"wrote {OUT}")
        return 0
    finally:
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
