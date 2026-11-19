"""로봇이 서 있을 때 라이다의 1 m 안 반사점을 로봇(base_footprint) 좌표로 저장 — 몸체 반사 근거(11-19 보고서)."""
import csv, math, sys, time, rclpy, tf2_ros
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
rclpy.init(); n = rclpy.create_node("map_fix_body_capture"); buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
scans = []; n.create_subscription(LaserScan, "/scan", scans.append, qos_profile_sensor_data)
end = time.monotonic() + 20
while time.monotonic() < end and len(scans) < 30: rclpy.spin_once(n, timeout_sec=0.2)
sx = sy = syaw = None
try:
    t = buf.lookup_transform("base_footprint", scans[0].header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=3.0)).transform
    q = t.rotation; sx, sy = t.translation.x, t.translation.y; syaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
except Exception as e:
    print("TF 없음:", e); sx, sy, syaw = 0.031, 0.0, 0.0
print(f"스캔 {len(scans)}장 · 라이다 자리(base 기준) x {sx:.3f} y {sy:.3f} yaw {math.degrees(syaw):.2f}°")
with open(sys.argv[1], "w", newline="") as f:
    w = csv.writer(f); w.writerow(["scan", "angle_deg", "range_m", "x_base_m", "y_base_m"])
    for k, m in enumerate(scans):
        for i, r in enumerate(m.ranges):
            if math.isfinite(r) and m.range_min <= r < 1.0:
                a = m.angle_min + i * m.angle_increment
                w.writerow([k, round(math.degrees(a), 2), round(r, 3), round(sx + r * math.cos(a + syaw), 3), round(sy + r * math.sin(a + syaw), 3)])
n.destroy_node(); rclpy.shutdown()
