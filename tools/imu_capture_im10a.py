"""
IM10A capture for the smartphone-vs-dedicated-IMU comparison (Chapter 3).

Logs the deployed Hiwonder IM10A over serial to CSV, with REAL packet arrival
timestamps (time.perf_counter). This matters: the smartphone log's timestamps
are synthetic/rounded (exactly 20.000 ms apart, zero jitter), so update-rate
claims cannot be made from it. Ours are measured, so ours can be.

Run it TWICE, once per phase:

    python tools/imu_capture_im10a.py --phase stationary --seconds 60
    python tools/imu_capture_im10a.py --phase moving     --seconds 60

Writes to  imu analysis data/im10a_<phase>.csv

Packet format (Hiwonder, 11 bytes, little-endian int16 pairs):
    0x55 0x51  ax ay az T   cs      accel, +/-16 g
    0x55 0x52  wx wy wz T   cs      gyro,  +/-2000 deg/s
    0x55 0x53  roll pitch yaw T cs  euler, +/-180 deg
    cs = sum(bytes 0..9) & 0xFF
"""
import argparse
import os
import struct
import sys
import time

import serial

ACC_SCALE = 16.0 * 9.80665 / 32768.0   # -> m/s^2
GYR_SCALE = 2000.0 / 32768.0           # -> deg/s
ANG_SCALE = 180.0 / 32768.0            # -> deg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM11")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--phase", required=True, choices=["stationary", "moving"])
    ap.add_argument("--outdir", default=os.path.join("imu analysis data"))
    args = ap.parse_args()

    if not os.path.isdir(args.outdir):
        os.makedirs(args.outdir)
    path = os.path.join(args.outdir, "im10a_%s.csv" % args.phase)

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    print("Opened %s at %d baud." % (args.port, args.baud))
    if args.phase == "stationary":
        print(">>> Keep the IMU COMPLETELY STILL on a flat surface for %.0f s." % args.seconds)
    else:
        print(">>> Motion protocol for %.0f s:" % args.seconds)
        print("      rotate ~90 deg, hold ~5 s, rotate back to the EXACT start")
        print("      orientation, hold ~5 s. Repeat. FINISH where you started.")
    print("Recording ...")

    acc = [0.0, 0.0, 0.0]
    gyr = [0.0, 0.0, 0.0]
    rows = []
    buf = bytearray()
    t0 = None
    t_start = time.perf_counter()

    while time.perf_counter() - t_start < args.seconds:
        data = ser.read(256)
        now = time.perf_counter()
        if data:
            buf.extend(data)

        while len(buf) >= 11:
            if buf[0] != 0x55:
                buf.pop(0)
                continue
            pkt = bytes(buf[:11])
            if (sum(pkt[:10]) & 0xFF) != pkt[10]:
                buf.pop(0)
                continue
            del buf[:11]

            a, b, c, _T = struct.unpack("<hhhh", pkt[2:10])
            if pkt[1] == 0x51:
                acc = [a * ACC_SCALE, b * ACC_SCALE, c * ACC_SCALE]
            elif pkt[1] == 0x52:
                gyr = [a * GYR_SCALE, b * GYR_SCALE, c * GYR_SCALE]
            elif pkt[1] == 0x53:
                # Angle packet closes each 3-packet burst -> emit one row.
                if t0 is None:
                    t0 = now
                rows.append((now - t0,
                             acc[0], acc[1], acc[2],
                             gyr[0], gyr[1], gyr[2],
                             a * ANG_SCALE, b * ANG_SCALE, c * ANG_SCALE))

    ser.close()

    with open(path, "w", newline="") as f:
        f.write("t_s,AX,AY,AZ,GX,GY,GZ,Roll,Pitch,Yaw\n")
        for r in rows:
            f.write("%.6f," % r[0] + ",".join("%.5f" % v for v in r[1:]) + "\n")

    n = len(rows)
    dur = rows[-1][0] if n else 0.0
    print("\nWrote %s" % path)
    print("  samples: %d   duration: %.2f s   effective rate: %.1f Hz"
          % (n, dur, (n - 1) / dur if dur > 0 else 0.0))
    if n:
        print("  yaw first/last: %.2f / %.2f deg" % (rows[0][9], rows[-1][9]))


if __name__ == "__main__":
    sys.exit(main())
