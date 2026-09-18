#!/usr/bin/env python3
"""FC + LiDAR 2D falsos para probar takeoff_position_hold_ev sin dron.

Escenarios (argv[1]):
  preflight  - obstaculo a 0.6 m desde el principio: el nodo NO debe armar
  approach   - arma con todo libre (3 m); en vuelo una pared se acerca a 1.2 m/s por el
               morro: el nodo debe frenar (xy velocidad 0, posicion NaN) y luego pedir LAND
  scanloss   - arma con todo libre; en vuelo el /scan deja de llegar: debe frenar y LAND
  slamlost   - arma con todo libre; en vuelo la sigma de /pose (SLAM) salta de 0.06 a 0.45 m,
               como en el vuelo del 2026-09-17: debe aterrizar
  userhold   - patron en cruz; run_scenario.sh manda HOLD (SIGUSR1) en pleno tramo y despues
               ATERRIZAR (SIGUSR2): debe frenar, quedarse quieto y aterrizar
  cancel     - run_scenario.sh manda HOLD antes de armar: no debe armar
Imprime lo que el nodo comanda. Correr con ROS_DOMAIN_ID aislado.
"""
import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_msgs.msg import (VehicleStatus, VehicleLocalPosition, VehicleAttitude, VehicleCommand,
                          TrajectorySetpoint, OffboardControlMode)

SCEN = sys.argv[1]


class MockFC(Node):
    def __init__(self):
        super().__init__("mock_fc")
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        be_tl = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           history=HistoryPolicy.KEEP_LAST, depth=1)
        self.st_pub = self.create_publisher(VehicleStatus, "/fmu/out/vehicle_status", be)
        self.lp_pub = self.create_publisher(VehicleLocalPosition, "/fmu/out/vehicle_local_position", be)
        self.att_pub = self.create_publisher(VehicleAttitude, "/fmu/out/vehicle_attitude", be)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/pose", 10)
        self.create_subscription(VehicleCommand, "/fmu/in/vehicle_command", self.on_cmd, be_tl)
        self.create_subscription(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", self.on_sp, be_tl)
        self.create_subscription(OffboardControlMode, "/fmu/in/offboard_control_mode", lambda m: None, be_tl)
        self.nav = VehicleStatus.NAVIGATION_STATE_POSCTL
        self.armed = False
        self.t0 = time.time()
        self.t_arm = None
        self.z = 0.0
        self.last_sp = None
        self.braking_seen = False
        self.pattern_seen = False
        self.hold_logged = False
        self.land_seen = False
        self.create_timer(0.02, self.tick_fc)     # 50 Hz
        self.create_timer(0.1, self.tick_scan)    # 10 Hz
        self.create_timer(0.5, self.tick_pose)    # 2 Hz, como slam_toolbox en la Pi

    def log(self, s):
        print(f"[mock t={time.time() - self.t0:5.1f}s] {s}", flush=True)

    def on_cmd(self, m):
        if m.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
            self.nav = VehicleStatus.NAVIGATION_STATE_OFFBOARD
            self.log("recibe DO_SET_MODE -> OFFBOARD")
        elif m.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            self.armed = m.param1 > 0.5
            self.t_arm = time.time() if self.armed else self.t_arm
            self.log(f"recibe ARM_DISARM param1={m.param1:.0f} -> {'ARMADO' if self.armed else 'desarmado'}")
        elif m.command == VehicleCommand.VEHICLE_CMD_NAV_LAND:
            self.nav = VehicleStatus.NAVIGATION_STATE_AUTO_LAND
            self.land_seen = True
            self.log("recibe NAV_LAND -> AUTO_LAND")

    def on_sp(self, m):
        braking = math.isnan(m.position[0]) and m.velocity[0] == 0.0 and m.velocity[1] == 0.0
        if braking and not self.braking_seen:
            self.braking_seen = True
            self.log(f"SETPOINT DE FRENADO: position={list(m.position)} velocity={list(m.velocity)}")
        if not math.isnan(m.position[0]):
            far = abs(float(m.position[0])) > 0.5 or abs(float(m.position[1])) > 0.5
            if far and not self.pattern_seen:
                self.pattern_seen = True
                self.log(f"el nodo manda un tramo del patron: position={[round(float(v), 2) for v in m.position]}")
            if self.braking_seen and not far and not self.hold_logged:
                self.hold_logged = True
                self.log(f"SETPOINT DE HOLD (tras frenar): position={[round(float(v), 2) for v in m.position]}")
        self.last_sp = m

    def flight_time(self):
        return None if self.t_arm is None else time.time() - self.t_arm

    def tick_fc(self):
        now_us = int(self.get_clock().now().nanoseconds / 1000)
        st = VehicleStatus()
        st.timestamp = now_us
        st.nav_state = self.nav
        st.nav_state_user_intention = VehicleStatus.NAVIGATION_STATE_OFFBOARD  # switch del RC en OFFBOARD
        st.arming_state = VehicleStatus.ARMING_STATE_ARMED if self.armed else VehicleStatus.ARMING_STATE_DISARMED
        self.st_pub.publish(st)
        if self.armed and self.nav == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.last_sp is not None:
            tz = float(self.last_sp.position[2])
            if not math.isnan(tz):
                self.z += max(-0.02, min(0.02, tz - self.z))  # sube a ~1 m/s
        if self.nav == VehicleStatus.NAVIGATION_STATE_AUTO_LAND and self.armed:
            self.z = min(0.0, self.z + 0.02)
            if self.z >= -0.01:
                self.armed = False
                self.log("aterrizado y desarmado")
        lp = VehicleLocalPosition()
        lp.timestamp = now_us
        lp.x = lp.y = 0.0
        lp.z = float(self.z)
        lp.heading = 0.0
        lp.xy_valid = lp.z_valid = lp.v_xy_valid = True
        lp.heading_good_for_control = True
        lp.dead_reckoning = False
        lp.eph = 0.05
        lp.dist_bottom = float(0.17 - self.z)
        lp.dist_bottom_valid = bool(self.z < -0.1)
        self.lp_pub.publish(lp)
        att = VehicleAttitude()
        att.timestamp = now_us
        att.q = [1.0, 0.0, 0.0, 0.0]
        self.att_pub.publish(att)

    def tick_pose(self):
        ft = self.flight_time()
        var = 0.004  # sigma 0.063 m, lo normal en hover
        if SCEN == "slamlost" and ft is not None and ft > 2.0:
            var = 0.2  # sigma 0.447 m
            if not getattr(self, "_lost_logged", False):
                self._lost_logged = True
                self.log("SLAM PIERDE EL TRACKING: sigma 0.45 m")
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        cov[0] = cov[7] = var
        cov[35] = 0.0001
        m.pose.covariance = cov
        self.pose_pub.publish(m)

    def tick_scan(self):
        ft = self.flight_time()
        if SCEN == "scanloss" and ft is not None and ft > 2.0:
            if not getattr(self, "_scan_off_logged", False):
                self._scan_off_logged = True
                self.log("CORTO /scan")
            return
        n = 500
        ranges = [4.0] * n
        near = None
        if SCEN == "preflight":
            near = 0.6
        elif SCEN == "approach" and ft is not None and ft > 2.0:
            near = max(0.3, 3.0 - 1.2 * (ft - 2.0))
        if near is not None:
            for i in list(range(0, 8)) + list(range(n - 8, n)):  # +-5.8 grados alrededor del morro
                ranges[i] = near
        m = LaserScan()
        m.header.frame_id = "base_laser"
        m.header.stamp = self.get_clock().now().to_msg()
        m.angle_min = 0.0
        m.angle_increment = 2 * math.pi / n
        m.angle_max = m.angle_increment * (n - 1)
        m.range_min, m.range_max = 0.02, 12.0
        m.ranges = ranges
        self.scan_pub.publish(m)
        if near is not None and SCEN == "approach" and int(ft * 10) % 5 == 0:
            self.log(f"pared a {near:.2f} m")


def main():
    rclpy.init()
    n = MockFC()
    end = time.time() + 45
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
        if SCEN != "preflight" and n.land_seen and not n.armed:
            # Seguir 1 s publicando DESARMADO: el nodo espera verlo para terminar.
            t_end = time.time() + 1.0
            while time.time() < t_end:
                rclpy.spin_once(n, timeout_sec=0.05)
            break
    n.log(f"FIN: frenado visto={n.braking_seen}, LAND visto={n.land_seen}, armado alguna vez={n.t_arm is not None}")


if __name__ == "__main__":
    main()
