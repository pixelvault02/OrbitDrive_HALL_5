import struct
import subprocess
import threading
import time

import serial


def config_frame():
    f = bytearray(20)
    f[0], f[1], f[2] = 0xAA, 0x55, 0x12
    f[3], f[4] = 0x03, 0x01
    f[19] = sum(f[2:19]) & 0xFF
    return bytes(f)


def parse_frames(buf):
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


text = []
can = []


def listen_text(port):
    try:
        ser = serial.Serial(port, 115200, timeout=0.2)
    except Exception as exc:
        text.append(f"{port} FAIL {exc}")
        return
    buf = b""
    t0 = time.time()
    while time.time() - t0 < 36:
        buf += ser.read(400)
    ser.close()
    text.append(f"=== {port} {len(buf)} bytes ===\n{buf.decode('ascii', 'replace')[:2500]}")


def listen_can():
    try:
        ser = serial.Serial("COM13", 2000000, timeout=0.15)
    except Exception as exc:
        can.append(f"COM13 FAIL {exc}")
        return
    ser.write(config_frame())
    buf = bytearray()
    t0 = time.time()
    last = 0.0
    n = 0
    first = None
    lim_n = 0
    while time.time() - t0 < 36:
        buf += ser.read(2048)
        for cid, pl in parse_frames(buf):
            n += 1
            now = time.time() - t0
            if first is None:
                first = now
                can.append(f"first CAN {now:.2f}s id=0x{cid:03X}")
            off = cid & 0xF
            node = (cid - 0x240) // 16 if cid >= 0x240 else -1
            if off == 1 and len(pl) >= 8 and now - last > 0.5:
                des, act = struct.unpack("<ff", pl[:8])
                can.append(f"{now:6.2f}s n{node} des={des:.1f} act={act:.1f}")
                last = now
            elif off == 5 and len(pl) >= 8 and lim_n < 3:
                lo, hi = struct.unpack("<ff", pl[:8])
                can.append(f"{now:6.2f}s LIMITS {lo:.1f}..{hi:.1f}")
                lim_n += 1
            elif off == 4 and len(pl) >= 8 and now - last > 0.45:
                uq, vel = struct.unpack("<ff", pl[:8])
                can.append(f"{now:6.2f}s Uq={uq:.2f} vel={vel:.3f}")
    ser.close()
    can.append(f"frames={n}")


def main():
    threads = [threading.Thread(target=listen_text, args=(p,)) for p in ("COM10", "COM11")]
    threads.append(threading.Thread(target=listen_can))
    for t in threads:
        t.start()
    time.sleep(0.6)
    result = subprocess.run(["ST-LINK_CLI.exe", "-c", "SWD", "UR", "-Rst"], capture_output=True, text=True)
    print("RESET", result.returncode)
    tail = (result.stdout or result.stderr or "")[-500:]
    print(tail)
    for t in threads:
        t.join()
    print("--- UART ---")
    print("\n".join(text) if text else "(none)")
    print("--- CAN ---")
    print("\n".join(can) if can else "(none)")


if __name__ == "__main__":
    main()
