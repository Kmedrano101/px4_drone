#!/usr/bin/env python3
"""Convierte los /scan de un rosbag (mcap) en la entrada de obstacle_guard_replay.

Cada barrido va con la actitud (vehicle_attitude) y la altura del plano del laser
(dist_bottom + laser_height_above_range) mas cercanas en el tiempo. Lee en modo
streaming, asi que sirve tambien para bags truncados (sin metadata.yaml).

    python3 bag_to_scans.py <bag_0.mcap> [t0_epoch] [laser_height_above_range_m] > scans.csv
Necesita: pip install mcap mcap-ros2-support
"""
import bisect, math, sys
from mcap.stream_reader import StreamReader
from mcap.records import Channel, Message, Schema
from mcap_ros2.decoder import DecoderFactory

path = sys.argv[1]
t_ref = float(sys.argv[2]) if len(sys.argv) > 2 else None
laser_above_range = float(sys.argv[3]) if len(sys.argv) > 3 else 0.134

chan, sch, dec = {}, {}, DecoderFactory()
scans, att, lpos = [], [], []
try:
    with open(path, "rb") as f:
        for r in StreamReader(f).records:
            if isinstance(r, Schema):
                sch[r.id] = r
            elif isinstance(r, Channel):
                chan[r.id] = r
            elif isinstance(r, Message):
                c = chan[r.channel_id]
                if c.topic not in ("/scan", "/fmu/out/vehicle_attitude", "/fmu/out/vehicle_local_position"):
                    continue
                m = dec.decoder_for(c.message_encoding, sch[c.schema_id])(r.data)
                t = r.log_time / 1e9
                if c.topic == "/scan":
                    scans.append((t, m))
                elif c.topic == "/fmu/out/vehicle_attitude":
                    w, x, y, z = m.q
                    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
                    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
                    att.append((t, roll, pitch))
                else:
                    h = (m.dist_bottom + laser_above_range) if m.dist_bottom_valid else float("nan")
                    lpos.append((t, h))
except Exception:
    pass  # bag truncado: se usa lo leido

att_t = [a[0] for a in att]
lpos_t = [p[0] for p in lpos]


def nearest(seq, times, t):
    """Muestra de seq mas cercana en el tiempo a t (seq ordenada por tiempo)."""
    if not seq:
        return None
    i = min(bisect.bisect_left(times, t), len(seq) - 1)
    if i > 0 and abs(seq[i - 1][0] - t) < abs(seq[i][0] - t):
        i -= 1
    return seq[i]


t0 = t_ref if t_ref is not None else (scans[0][0] if scans else 0.0)
print(f"# {len(scans)} scans, {len(att)} actitudes, {len(lpos)} posiciones; t relativo a {t0:.3f}")
for t, m in scans:
    a = nearest(att, att_t, t) or (t, 0.0, 0.0)
    p = nearest(lpos, lpos_t, t) or (t, float("nan"))
    rs = ",".join("nan" if not math.isfinite(v) else f"{v:.3f}" for v in m.ranges)
    print(f"{t - t0:.3f},{a[1]:.5f},{a[2]:.5f},{p[1]:.3f},{m.angle_min:.6f},{m.angle_increment:.6f},"
          f"{m.range_min:.3f},{m.range_max:.3f},{len(m.ranges)},{rs}")
