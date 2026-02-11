#!/usr/bin/env python3
import argparse
import csv
import socket
import time


def parse_bool(s: str) -> bool:
    s = (s or "").strip().lower()
    return s in ("1", "true", "yes", "y")


def load_rows(path):
    rows = []
    with open(path, "r", newline="") as f:
        r = csv.reader(f)
        header = next(r, None)
        if not header:
            return rows
        idx = {h.strip().lower(): i for i, h in enumerate(header)}

        def get(row, key):
            i = idx.get(key, -1)
            return row[i] if 0 <= i < len(row) else ""

        for row in r:
            if not row:
                continue
            t = float(get(row, "t") or 0)
            detected = parse_bool(get(row, "detected"))
            conf = get(row, "conf") or get(row, "confidence") or "0"
            cx = get(row, "cx") or "0"
            cy = get(row, "cy") or "0"
            size = get(row, "size") or "0"
            rows.append({
                "t": t,
                "detected": detected,
                "conf": float(conf),
                "cx": float(cx),
                "cy": float(cy),
                "size": float(size),
            })
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", default="iva-log-2.log")
    ap.add_argument("--addr", default="127.0.0.1:9001")
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--speed", type=float, default=1.0, help="1.0 = real-time")
    ap.add_argument("--print-every", type=int, default=1, help="print every Nth send (default: 1)")
    ap.add_argument("--quiet", action="store_true", help="disable printing")
    args = ap.parse_args()

    host, port = args.addr.split(":")
    addr = (host, int(port))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    rows = load_rows(args.log)
    if not rows:
        raise SystemExit("no rows in log")

    while True:
        last_t = rows[0]["t"]
        count = 0
        for row in rows:
            dt = (row["t"] - last_t) / max(args.speed, 1e-6)
            if dt > 0:
                time.sleep(dt)
            last_t = row["t"]

            payload = f"{row['t']},{int(row['detected'])},{row['conf']},{row['cx']},{row['cy']},{row['size']}".encode()
            sock.sendto(payload, addr)
            count += 1
            if not args.quiet and (count % max(args.print_every, 1) == 0):
                print(payload.decode(errors="replace"))

        if not args.loop:
            break


if __name__ == "__main__":
    main()
