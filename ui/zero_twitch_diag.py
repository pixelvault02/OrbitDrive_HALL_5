#!/usr/bin/env python3
"""Recalibrate OrbitDrive_HALL_5 over CAN, log zero-hold twitch, then re-command.

  python ui/zero_twitch_diag.py
"""
from __future__ import annotations

import csv
import math
import struct
import statistics
import time
from pathlib import Path

import serial
from serial.tools import list_ports

PORT = "COM4"
WS_BAUD = 2_000_000
NODE = 1

CAN_CMD_BASE = 0x140
CAN_RPT_BASE = 0x240
STRIDE = 0x10
CMD_SET_ANGLE = 0x01
CMD_RECALIBRATE = 0x05
CMD_GET_PARAMS = 0x07
RPT_TELEMETRY = 0x01
RPT_STATUS = 0x04
RPT_PARAM = 0x02

OUT = Path(__file__).resolve().parent / "zero_twitch_diag.csv"
RPT = Path(__file__).resolve().parent / "zero_twitch_report.txt"


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


def cmd(off):
    return CAN_CMD_BASE + NODE * STRIDE + off


def rpt(off):
    return CAN_RPT_BASE + NODE * STRIDE + off


def summarize(rows, label: str) -> str:
    if len(rows) < 5:
        return f"{label}: too few samples ({len(rows)})\n"
    acts = [r[2] for r in rows]
    dess = [r[1] for r in rows]
    uqs = [r[3] for r in rows]
    vels = [r[4] for r in rows]
    errs = [d - a for d, a in zip(dess, acts)]
    # zero-crossing count on actual (twitch proxy)
    zc = 0
    for i in range(1, len(acts)):
        if acts[i - 1] == 0:
            continue
        if acts[i - 1] * acts[i] < 0:
            zc += 1
    dt = rows[-1][0] - rows[0][0]
    zc_hz = (zc / dt) if dt > 0.1 else 0.0
    lines = [
        f"=== {label}  n={len(rows)}  duration={dt:.2f}s ===",
        f"  actual  mean={statistics.mean(acts):+.3f}  std={statistics.pstdev(acts):.3f}  "
        f"pp={max(acts)-min(acts):.3f}  range=[{min(acts):+.3f},{max(acts):+.3f}]",
        f"  desired mean={statistics.mean(dess):+.3f}  std={statistics.pstdev(dess):.3f}  "
        f"pp={max(dess)-min(dess):.3f}",
        f"  error   mean={statistics.mean(errs):+.3f}  std={statistics.pstdev(errs):.3f}  "
        f"pp={max(errs)-min(errs):.3f}",
        f"  Uq      mean={statistics.mean(uqs):+.3f}  std={statistics.pstdev(uqs):.3f}  "
        f"max|Uq|={max(abs(u) for u in uqs):.3f}",
        f"  vel     mean={statistics.mean(vels):+.3f}  std={statistics.pstdev(vels):.3f}  "
        f"max|vel|={max(abs(v) for v in vels):.3f}",
        f"  actual zero-crossings~{zc}  (~{zc_hz:.1f} Hz)",
        "",
    ]
    return "\n".join(lines)


def main():
    if PORT not in [p.device for p in list_ports.comports()]:
        raise SystemExit(f"{PORT} missing")

    ser = serial.Serial(PORT, WS_BAUD, timeout=0.05)
    buf = bytearray()
    des = act = uq = vel = 0.0
    have = False
    params: dict[int, float] = {}
    all_rows: list[tuple] = []

    def poll():
        nonlocal des, act, uq, vel, have
        chunk = ser.read(512)
        if chunk:
            buf.extend(chunk)
        for cid, pl in parse_frames(buf):
            if cid == rpt(RPT_TELEMETRY) and len(pl) >= 8:
                des, act = struct.unpack("<ff", pl[:8])
                have = True
            elif cid == rpt(RPT_STATUS) and len(pl) >= 8:
                uq, vel = struct.unpack("<ff", pl[:8])
            elif cid == rpt(RPT_PARAM) and len(pl) >= 5:
                params[pl[0]] = struct.unpack("<f", pl[1:5])[0]

    def send(can_id, payload=b""):
        ser.write(pack_tx(can_id, payload))

    def capture(phase: str, seconds: float):
        rows = []
        t0 = time.time()
        while time.time() - t0 < seconds:
            poll()
            if have:
                t = time.time() - t0
                row = (t, des, act, uq, vel, phase)
                rows.append(row)
                all_rows.append(row)
            time.sleep(0.008)
        return rows

    try:
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.write(config_frame())
        time.sleep(0.2)

        print("Waiting for live telem before recal...")
        t_wait = time.time()
        while time.time() - t_wait < 8.0:
            poll()
            if have:
                break
            time.sleep(0.02)
        if not have:
            raise SystemExit("No telemetry — is drive powered / CAN node 1?")

        send(cmd(CMD_GET_PARAMS))
        t_p = time.time()
        while time.time() - t_p < 1.5:
            poll()
            time.sleep(0.01)

        print(
            f"Pre-recal: act={act:+.2f} des={des:+.2f} uq={uq:+.3f}  "
            f"AngleP={params.get(4)} LPF={params.get(5)} Slew={params.get(14)}"
        )
        print("Sending RECALIBRATE (invalidate FOC calib + reboot)...")
        send(cmd(CMD_RECALIBRATE))
        have = False
        last_act = None
        updates = 0
        print("Waiting for reboot + FOC align + boot home (up to 120s)...")
        t0 = time.time()
        telem_alive_t = None
        while time.time() - t0 < 120.0:
            poll()
            if have:
                if last_act is None or abs(act - last_act) > 1e-4 or abs(uq) > 1e-4:
                    updates += 1
                    last_act = act
                    telem_alive_t = time.time()
                # Require telem alive for a while near zero after home
                if (
                    updates > 30
                    and abs(des) < 0.5
                    and abs(act) < 5.0
                    and telem_alive_t
                    and (time.time() - telem_alive_t) < 0.5
                    and (time.time() - t0) > 20.0
                ):
                    print(f"  telem live near 0 after {time.time()-t0:.1f}s  act={act:+.2f}")
                    break
            time.sleep(0.02)
        else:
            raise SystemExit("No stable telem after recalibrate timeout")

        print("Settle 4s at post-home zero...")
        time.sleep(4.0)

        print(f"Phase A: log zero-hold 8s (act={act:+.2f} des={des:+.2f})")
        rows_a = capture("post_recal_zero", 8.0)

        print("Phase B: command +8° , wait settle, then 0°, wait settle")
        send(cmd(CMD_SET_ANGLE), struct.pack("<f", 8.0))
        t_m = time.time()
        while time.time() - t_m < 8.0:
            poll()
            if abs(des - 8.0) < 1.0 and abs(act - 8.0) < 2.0:
                time.sleep(1.0)
                break
            time.sleep(0.02)
        send(cmd(CMD_SET_ANGLE), struct.pack("<f", 0.0))
        t_m = time.time()
        while time.time() - t_m < 10.0:
            poll()
            if abs(des) < 0.5 and abs(act) < 2.0:
                time.sleep(2.0)
                break
            time.sleep(0.02)

        print(f"Phase C: log zero-hold after re-command 8s (act={act:+.2f} des={des:+.2f})")
        rows_c = capture("after_angle_cmd_zero", 8.0)

        send(cmd(CMD_GET_PARAMS))
        t_p = time.time()
        while time.time() - t_p < 1.5:
            poll()
            time.sleep(0.01)

        with OUT.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "desired", "actual", "uq", "vel", "phase"])
            for r in all_rows:
                w.writerow([f"{r[0]:.4f}", f"{r[1]:.4f}", f"{r[2]:.4f}", f"{r[3]:.4f}", f"{r[4]:.4f}", r[5]])

        report = []
        report.append("OrbitDrive_HALL_5 zero-twitch diagnostic")
        report.append(f"time={time.strftime('%Y-%m-%d %H:%M:%S')}")
        report.append(
            f"params AngleP={params.get(4)} LPF={params.get(5)} "
            f"VelP={params.get(0)} VelI={params.get(1)} Slew={params.get(14)} "
            f"Accel={params.get(15)} Decel={params.get(16)}"
        )
        report.append(f"csv={OUT}")
        report.append("")
        report.append(summarize(rows_a, "A: after recal — hold at 0 (no extra command)"))
        report.append(summarize(rows_c, "C: after SET_ANGLE +8 then 0 — hold at 0"))

        # Compare twitch severity
        std_a = statistics.pstdev([r[2] for r in rows_a]) if len(rows_a) > 2 else 0
        std_c = statistics.pstdev([r[2] for r in rows_c]) if len(rows_c) > 2 else 0
        uq_a = max(abs(r[3]) for r in rows_a) if rows_a else 0
        uq_c = max(abs(r[3]) for r in rows_c) if rows_c else 0

        report.append("--- Interpretation ---")
        report.append(
            f"actual std: A={std_a:.3f}° → C={std_c:.3f}°  "
            f"({'improved after re-command' if std_c < std_a * 0.7 else 'similar / not clearly better'})"
        )
        report.append(
            f"max|Uq|: A={uq_a:.3f} → C={uq_c:.3f}"
        )
        report.append("")
        report.append("Likely firmware mechanism (code review + this log):")
        report.append(
            "1. After boot home, restoreAngleMode() sets target=angle_goal=current shaft,"
        )
        report.append(
            "   then applyAngleCommand(0) retargets angle_goal to geometric mid (mech_zero)."
        )
        report.append(
            "   If mid_err ≠ 0, aggressive profile (slew 1500, accel/decel 2500) drives into 0."
        )
        report.append(
            "2. Hold uses Angle P=30 with velocity LPF Tf=0.75 (very slow feedback) →"
        )
        report.append(
            "   underdamped hunting / twitch around 0 while desired≈0."
        )
        report.append(
            "3. A later SET_ANGLE rebuilds angle_goal from live shaft and re-runs a clean"
        )
        report.append(
            "   accel/decel into the target; PID state after a real move often settles quieter."
        )
        report.append(
            "4. Not primarily soft-limit asymmetry — this is zero-hold loop / post-home setpoint."
        )

        text = "\n".join(report) + "\n"
        RPT.write_text(text, encoding="utf-8")
        print("\n" + text)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
