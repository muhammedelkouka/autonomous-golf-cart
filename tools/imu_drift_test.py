"""
IMU Drift Test — Hiwonder IM10A on COM11
Reads angle packets for 60 seconds, reports yaw drift.
Run with: python tools/imu_drift_test.py

Pass/Fail criteria:
  PASS  — yaw drift < 0.3 deg over 60 s  →  Fix 2 IMU heading is reliable
  WARN  — 0.3–1.0 deg                    →  marginal; monitor on campus test
  FAIL  — > 1.0 deg                      →  IMU heading has drift problem; investigate
"""

import serial, struct, time, sys

PORT   = "COM11"
BAUD   = 115200
DURATION_S = 60

def parse_stream(port, baud, duration):
    ser = serial.Serial(port, baud, timeout=0.1)
    print(f"Opened {port} at {baud}. Stay COMPLETELY STILL. Recording {duration}s ...")

    buf = bytearray()
    yaw_start = None
    yaw_latest = None
    t_start = time.time()
    t_first_packet = None
    packet_count = 0
    last_print = t_start

    while True:
        elapsed = time.time() - t_start
        if elapsed >= duration:
            break

        data = ser.read(64)
        if data:
            buf.extend(data)

        # Parse all complete 11-byte packets
        while len(buf) >= 11:
            # Find 0x55 header
            if buf[0] != 0x55:
                buf.pop(0)
                continue

            # Need 11 bytes
            if len(buf) < 11:
                break

            pkt = buf[:11]

            # Verify checksum
            expected_cs = sum(pkt[:10]) & 0xFF
            if pkt[10] != expected_cs:
                buf.pop(0)
                continue

            ptype = pkt[1]
            buf = buf[11:]

            # Angle packet 0x53: roll, pitch, yaw as int16 LE, scale = 180/32768
            if ptype == 0x53:
                roll  = struct.unpack_from('<h', pkt, 2)[0] * 180.0 / 32768.0
                pitch = struct.unpack_from('<h', pkt, 4)[0] * 180.0 / 32768.0
                yaw   = struct.unpack_from('<h', pkt, 6)[0] * 180.0 / 32768.0

                if yaw_start is None:
                    yaw_start = yaw
                    t_first_packet = time.time()
                yaw_latest = yaw
                packet_count += 1

        # Print progress every 10 seconds
        now = time.time()
        if now - last_print >= 10 and yaw_latest is not None:
            drift = yaw_latest - yaw_start
            elapsed_real = now - t_start
            print(f"  t={elapsed_real:5.1f}s  yaw={yaw_latest:7.3f}°  drift={drift:+.3f}°  packets={packet_count}")
            last_print = now

    ser.close()
    return yaw_start, yaw_latest, packet_count, t_first_packet

try:
    yaw_start, yaw_final, count, t_first = parse_stream(PORT, BAUD, DURATION_S)
except serial.SerialException as e:
    print(f"\nERROR: Cannot open {PORT}: {e}")
    print("Make sure the IMU USB cable is connected and no other app (Hiwonder app, main stack) is using COM11.")
    sys.exit(1)

if yaw_start is None:
    print("\nERROR: No angle packets received. Check IMU is powered and sending data.")
    sys.exit(1)

drift = yaw_final - yaw_start
rate  = drift / DURATION_S  # deg/s

print(f"\n{'='*55}")
print(f"  Duration   : {DURATION_S}s")
print(f"  Packets rx : {count}")
print(f"  Yaw start  : {yaw_start:.3f}°")
print(f"  Yaw end    : {yaw_final:.3f}°")
print(f"  Total drift: {drift:+.3f}°")
print(f"  Drift rate : {rate:+.4f} deg/s")
print(f"{'='*55}")

if abs(drift) < 0.3:
    verdict = "PASS — IMU heading stable. Fix 2 is reliable."
elif abs(drift) < 1.0:
    verdict = "WARN — marginal drift. Monitor robot_yaw_deg during campus test."
else:
    verdict = "FAIL — drift too high (> 1 deg/60s). Fix 2 may accumulate heading error."

print(f"  Result: {verdict}")
print()
