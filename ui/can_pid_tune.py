#!/usr/bin/env python3
"""Closed-loop PID (+ EKF) iterate for OrbitDrive_HALL_5.

Orbit Drive 3.0 cascade on analog halls with HallAngleEKF velocity filter.

Usage:
  python ui/can_pid_tune.py --iterate --save     # fine search around seed
  python ui/can_pid_tune.py --quick --save       # coarse then refine
"""
from __future__ import annotations

import argparse
import math
import struct
import time
from dataclasses import dataclass, field
from pathlib import Path

import serial

PORT = "COM4"
WS_BAUD = 2_000_000
NODE = 1

P_VEL_P, P_VEL_I, P_VEL_D, P_VEL_RAMP, P_ANGLE_P, P_LPF = 0, 1, 2, 3, 4, 5
P_EKF_EN, P_EKF_QA, P_EKF_QV, P_EKF_R = 6, 7, 8, 9

OUT_RPT = Path(__file__).with_name("pid_tune_report.txt")


def config_frame():
    f = bytearray(20)
    f[0], f[1], f[2] = 0xAA, 0x55, 0x12
    f[3], f[4] = 0x03, 0x01
    f[19] = sum(f[2:19]) & 0xFF
    return bytes(f)


def send(ser, can_id, payload=b""):
    dlc = len(payload) & 0x0F
    frame = bytearray([0xAA, 0xC0 | dlc, can_id & 0xFF, (can_id >> 8) & 0xFF])
    frame.extend(payload)
    frame.append(0x55)
    ser.write(frame)


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
class Tel:
    t0: float = field(default_factory=time.time)
    desired: float | None = None
    actual: float | None = None
    rows: list = field(default_factory=list)

    def now(self):
        return time.time() - self.t0

    def ingest(self, cid, pl, telem_id):
        if cid != telem_id or len(pl) < 8:
            return
        d, a = struct.unpack("<ff", pl[:8])
        self.desired, self.actual = d, a
        self.rows.append((self.now(), d, a))


def set_param(ser, cmd_set_param, idx: int, val: float):
    send(ser, cmd_set_param, bytes([idx & 0xFF]) + struct.pack("<f", float(val)))


def set_angle(ser, cmd_set_angle, deg: float):
    send(ser, cmd_set_angle, struct.pack("<f", float(deg)))


def apply_all(ser, cmd_set_param, g: dict):
    order = [
        (P_VEL_P, "vel_p"),
        (P_VEL_I, "vel_i"),
        (P_VEL_D, "vel_d"),
        (P_VEL_RAMP, "vel_ramp"),
        (P_ANGLE_P, "angle_p"),
        (P_LPF, "lpf"),
        (P_EKF_EN, "ekf_en"),
        (P_EKF_QA, "ekf_qa"),
        (P_EKF_QV, "ekf_qv"),
        (P_EKF_R, "ekf_r"),
    ]
    for idx, key in order:
        if key in g:
            set_param(ser, cmd_set_param, idx, g[key])
            time.sleep(0.008)
    time.sleep(0.03)


def pump(ser, tel: Tel, buf: bytearray, duration: float, telem_id: int):
    t_end = time.time() + duration
    while time.time() < t_end:
        n = ser.in_waiting
        if n:
            buf.extend(ser.read(n))
        for cid, pl in parse_frames(buf):
            tel.ingest(cid, pl, telem_id)
        time.sleep(0.0005)


def score_step(rows, cmd: float, band: float = 0.8):
    if len(rows) < 10:
        return {"ok": False, "score": 1e9}
    xs = [(t, a) for t, d, a in rows if abs(d - cmd) < 0.6]
    if len(xs) < 10:
        xs = [(t, a) for t, _, a in rows]
    t0 = xs[0][0]
    angs = [a for _, a in xs]
    errs = [a - cmd for a in angs]

    settle = None
    hold_t0 = None
    for t, a in xs:
        if abs(a - cmd) <= band:
            if hold_t0 is None:
                hold_t0 = t
            elif t - hold_t0 >= 0.18:
                settle = t - t0
                break
        else:
            hold_t0 = None

    start = angs[0]
    direction = 1.0 if cmd >= start else -1.0
    peak = max((a - cmd) * direction for a in angs)
    overshoot = max(0.0, peak)

    late = [(t, a) for t, a in xs if t >= xs[-1][0] - 0.9]
    crossings = 0
    prev = None
    for _, a in late:
        e = a - cmd
        if prev is not None and e * prev < 0:
            crossings += 1
        prev = e

    tail = [abs(a - cmd) for t, a in xs if t >= xs[-1][0] - 0.55]
    ss = sum(tail) / max(1, len(tail))
    rms = math.sqrt(sum(e * e for e in errs) / len(errs))

    # Peak-to-peak in last 0.6 s — vibration proxy
    late_a = [a for t, a in xs if t >= xs[-1][0] - 0.6]
    p2p = (max(late_a) - min(late_a)) if late_a else 99.0

    cost = 0.0
    cost += 0.0 if settle is not None else 35.0
    if settle is not None:
        cost += 10.0 * settle
    cost += 8.0 * overshoot
    cost += 2.0 * crossings
    cost += 18.0 * ss
    cost += 3.0 * rms
    cost += 4.0 * p2p
    if ss > 1.2:
        cost += 25.0
    if overshoot > 5.0:
        cost += 30.0
    if p2p > 3.0:
        cost += 40.0

    ok = (
        settle is not None
        and ss <= 1.0
        and overshoot <= 4.0
        and crossings <= 6
        and p2p <= 2.5
    )
    return {
        "ok": ok,
        "score": cost,
        "settle": settle,
        "os": overshoot,
        "ss": ss,
        "chatter": crossings,
        "p2p": p2p,
        "rms": rms,
        "final": angs[-1],
    }


def trial(ser, buf, ids, gains: dict, targets: list[float], dwell: float):
    cmd_set_angle, cmd_set_param, telem_id = ids
    apply_all(ser, cmd_set_param, gains)
    time.sleep(0.2)
    metrics = []
    for tgt in targets:
        tel = Tel()
        set_angle(ser, cmd_set_angle, tgt)
        pump(ser, tel, buf, dwell, telem_id)
        m = score_step(tel.rows, tgt)
        metrics.append(m)
        print(
            f"  tgt={tgt:+5.1f} settle={m.get('settle')} "
            f"os={m.get('os', 0):.2f} ss={m.get('ss', 0):.2f} "
            f"p2p={m.get('p2p', 0):.2f} ch={m.get('chatter', 0)} "
            f"score={m['score']:.1f}"
        )
    set_angle(ser, cmd_set_angle, 0.0)
    pump(ser, Tel(), buf, 0.9, telem_id)

    n_ok = sum(1 for m in metrics if m.get("ok"))
    total = sum(m["score"] for m in metrics) / max(1, len(metrics))
    total += 0.5 * sum(m.get("chatter", 0) for m in metrics)
    total += 2.0 * sum(m.get("p2p", 0) for m in metrics) / max(1, len(metrics))
    return {"score": total, "ok": n_ok, "n": len(metrics), "metrics": metrics, "gains": dict(gains)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--node", type=int, default=NODE)
    ap.add_argument("--dwell", type=float, default=2.2)
    ap.add_argument("--save", action="store_true")
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--iterate", action="store_true", help="Fine local search around seed")
    args = ap.parse_args()

    can_cmd = 0x140 + args.node * 0x10
    can_rpt = 0x240 + args.node * 0x10
    cmd_set_angle = can_cmd + 0x01
    cmd_save = can_cmd + 0x03
    cmd_set_param = can_cmd + 0x06
    telem_id = can_rpt + 0x01
    ids = (cmd_set_angle, cmd_set_param, telem_id)

    seed = {
        "vel_p": 0.030,
        "vel_i": 0.60,
        "vel_d": 0.0,
        "vel_ramp": 150.0,
        "angle_p": 1.50,
        "lpf": 0.050,
        "ekf_en": 0.0,  # OFF — lag destabilizes angle CL; optional later
        "ekf_qa": 0.001,
        "ekf_qv": 120.0,
        "ekf_r": 0.012,
    }

    targets = [0.0, 4.0, -4.0, 8.0, -8.0, 0.0]

    ser = serial.Serial(args.port, WS_BAUD, timeout=0.05)
    ser.reset_input_buffer()
    ser.write(config_frame())
    time.sleep(0.3)
    buf = bytearray()

    tel = Tel()
    pump(ser, tel, buf, 1.5, telem_id)
    if tel.actual is None:
        print("ERROR: no telemetry — check COM4 / HALL_5 / node")
        ser.close()
        return 1
    print(f"link OK actual={tel.actual:.2f} desired={tel.desired:.2f} node={args.node}")
    print("Apply seed gains (EKF off), settle...")
    apply_all(ser, cmd_set_param, seed)
    set_angle(ser, cmd_set_angle, 0.0)
    pump(ser, Tel(), buf, 2.5, telem_id)

    results = []

    if args.iterate:
        print("\n=== iterate: fine grid around seed (EKF off) ===")
        angle_ps = [1.0, 1.25, 1.5, 1.8, 2.2, 2.8]
        vel_ps = [0.018, 0.024, 0.030, 0.038, 0.048]
        vel_is = [0.30, 0.45, 0.60, 0.80, 1.0]
        lpfs = [0.03, 0.05, 0.07]
        ramps = [80.0, 150.0, 250.0]
        # Optional late EKF probe only if soft PID is already ok
        ekf_modes = [
            {"ekf_en": 0.0, "ekf_qv": 120.0, "ekf_r": 0.012},
            {"ekf_en": 1.0, "ekf_qv": 120.0, "ekf_r": 0.012},
            {"ekf_en": 1.0, "ekf_qv": 200.0, "ekf_r": 0.020},
        ]

        for ap_ in angle_ps:
            for vp in vel_ps:
                g = dict(seed)
                g["angle_p"] = ap_
                g["vel_p"] = vp
                print(f"\ntry A={ap_:.2f} Vp={vp:.3f}")
                r = trial(ser, buf, ids, g, targets, args.dwell)
                results.append(r)
                print(f"  -> avg={r['score']:.1f} ok={r['ok']}/{r['n']}")

        results.sort(key=lambda x: x["score"])
        mid = results[0]["gains"]
        print("\n=== iterate pass2: I / LPF / ramp ===")
        for vi in vel_is:
            for lpf in lpfs:
                for ramp in ramps:
                    g = dict(mid)
                    g["vel_i"] = vi
                    g["lpf"] = lpf
                    g["vel_ramp"] = ramp
                    print(f"\ntry Vi={vi:.2f} LPF={lpf:.3f} R={ramp:.0f}")
                    r = trial(ser, buf, ids, g, targets, args.dwell)
                    results.append(r)
                    print(f"  -> avg={r['score']:.1f} ok={r['ok']}/{r['n']}")

        results.sort(key=lambda x: x["score"])
        mid = results[0]["gains"]
        print("\n=== iterate pass3: EKF on/off probe around best PID ===")
        for em in ekf_modes:
            g = dict(mid)
            g.update(em)
            print(f"\ntry EKF en={em['ekf_en']:.0f} Qv={em['ekf_qv']:.0f} R={em['ekf_r']:.3f}")
            r = trial(ser, buf, ids, g, targets, args.dwell)
            results.append(r)
            print(f"  -> avg={r['score']:.1f} ok={r['ok']}/{r['n']}")
    else:
        # Coarse then refine (EKF off)
        if args.quick:
            angle_ps = [1.2, 1.5, 2.0, 2.8, 4.0]
            vel_ps = [0.02, 0.03, 0.045, 0.06]
            vel_is = [0.3, 0.6, 1.0]
            lpfs = [0.025, 0.04, 0.06]
            ramps = [100.0, 200.0]
        else:
            angle_ps = [1.0, 1.5, 2.0, 3.0, 4.0, 6.0]
            vel_ps = [0.02, 0.03, 0.045, 0.06, 0.09]
            vel_is = [0.25, 0.5, 0.8, 1.2]
            lpfs = [0.02, 0.04, 0.06]
            ramps = [80.0, 150.0, 250.0]

        print("\n=== coarse: angle_p x vel_p (EKF off) ===")
        for ap_ in angle_ps:
            for vp in vel_ps:
                g = dict(seed)
                g["angle_p"] = ap_
                g["vel_p"] = vp
                print(f"\ntry A={ap_:.2f} Vp={vp:.3f}")
                r = trial(ser, buf, ids, g, targets, args.dwell)
                results.append(r)
                print(f"  -> avg={r['score']:.1f} ok={r['ok']}/{r['n']}")

        results.sort(key=lambda x: x["score"])
        mid = results[0]["gains"]
        print("\n=== refine I / LPF / ramp ===")
        for vi in vel_is:
            for lpf in lpfs:
                for ramp in ramps:
                    g = dict(mid)
                    g["vel_i"] = vi
                    g["lpf"] = lpf
                    g["vel_ramp"] = ramp
                    print(f"\ntry Vi={vi:.2f} LPF={lpf:.3f} R={ramp:.0f}")
                    r = trial(ser, buf, ids, g, targets, args.dwell)
                    results.append(r)
                    print(f"  -> avg={r['score']:.1f} ok={r['ok']}/{r['n']}")

    results.sort(key=lambda x: x["score"])
    best = results[0]
    top = results[:8]
    g = best["gains"]

    lines = [
        "OrbitDrive_HALL_5 closed-loop PID+EKF iterate",
        f"node={args.node} dwell={args.dwell}s targets={targets} mode="
        + ("iterate" if args.iterate else ("quick" if args.quick else "full")),
        "",
        "BEST:",
        f"  angle_p={g['angle_p']:.3f}  vel_p={g['vel_p']:.4f}  vel_i={g['vel_i']:.3f}  "
        f"ramp={g['vel_ramp']:.1f}  lpf={g['lpf']:.4f}",
        f"  ekf_en={g.get('ekf_en', 1):.0f}  ekf_qa={g.get('ekf_qa', 0):.4f}  "
        f"ekf_qv={g.get('ekf_qv', 0):.1f}  ekf_r={g.get('ekf_r', 0):.4f}",
        f"  score={best['score']:.2f}  steps_ok={best['ok']}/{best['n']}",
        "",
        "TOP:",
    ]
    for i, r in enumerate(top, 1):
        gg = r["gains"]
        lines.append(
            f"  {i}. score={r['score']:.1f} ok={r['ok']}/{r['n']}  "
            f"A={gg['angle_p']:.2f} Vp={gg['vel_p']:.3f} Vi={gg['vel_i']:.2f} "
            f"R={gg['vel_ramp']:.0f} LPF={gg['lpf']:.3f} "
            f"EKF_R={gg.get('ekf_r', 0):.3f} Qv={gg.get('ekf_qv', 0):.0f}"
        )
    text = "\n".join(lines)
    print("\n" + text)
    OUT_RPT.write_text(text + "\n", encoding="utf-8")
    print(f"\nwrote {OUT_RPT}")

    apply_all(ser, cmd_set_param, best["gains"])
    set_angle(ser, cmd_set_angle, 0.0)
    if args.save:
        send(ser, cmd_save, b"")
        print("SAVE_CFG sent")
    else:
        print("Best applied live (not saved). Use --save to persist.")

    ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
