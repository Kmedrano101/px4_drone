#!/usr/bin/env python3
"""Comprueba la varianza que ev_odometry_bridge manda al EKF, sin dron ni SLAM.

Publica una TF fija map -> base_link y una /pose con sigma 0.063 m cada 1 s, y
escucha /fmu/in/vehicle_visual_odometry. Lo esperado:
  - justo despues de cada /pose: varianza xy ~ la del SLAM (0.004 m^2)
  - un segundo despues: 0.004 + (1 s * ev_max_speed_m_s)^2  (~2.25 m^2 con 1.5 m/s)
  - z, roll y pitch: 1e4 (finitas, para que EKF2 use las demas)
Correr con el puente lanzado en el mismo ROS_DOMAIN_ID (ver check_variance.sh).
"""
import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from px4_msgs.msg import VehicleOdometry


class Check(Node):
    def __init__(self):
        super().__init__("check_ev_variance")
        self.tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = "map", "base_link"
        t.header.stamp = self.get_clock().now().to_msg()
        t.transform.rotation.w = 1.0
        self.tf.sendTransform(t)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/pose", 10)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(VehicleOdometry, "/fmu/in/vehicle_visual_odometry", self.on_odom, qos)
        self.create_timer(1.0, self.send_pose)
        self.last_pose_t = None
        self.rows = []

    def send_pose(self):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.004
        cov[35] = 0.0001
        m.pose.covariance = cov
        self.pose_pub.publish(m)
        self.last_pose_t = time.time()

    def on_odom(self, m):
        if self.last_pose_t is None:
            self.rows.append(("sin /pose", [float(v) for v in m.position_variance], [float(v) for v in m.orientation_variance]))
            return
        self.rows.append((time.time() - self.last_pose_t, [float(v) for v in m.position_variance],
                          [float(v) for v in m.orientation_variance]))


def main():
    rclpy.init()
    n = Check()
    end = time.time() + 6.5
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    print("  edad pose   var_x(N)   var_y(E)   var_z      var_yaw")
    for age, pv, ov in n.rows[::4]:
        a = age if isinstance(age, str) else f"{age:6.2f} s"
        print(f"  {a:>9}  {pv[0]:8.4f}  {pv[1]:8.4f}  {pv[2]:8.0f}  {ov[2]:8.4f}")
    ok_fin = all(all(math.isfinite(v) for v in pv) for age, pv, ov in n.rows if not isinstance(age, str))
    print(f"\n{len(n.rows)} mensajes; todas las varianzas finitas tras la primera /pose: {ok_fin}")


if __name__ == "__main__":
    main()
