#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint

class OffboardStreamer(Node):
    def __init__(self):
        super().__init__('offboard_streamer')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.ocm_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.sp_pub  = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.create_timer(0.1, self.on_timer)   # 10 Hz (> 2 Hz requerido)
        self.get_logger().info('Publicando setpoints Offboard a 10 Hz (props FUERA)...')

    def on_timer(self):
        now = int(self.get_clock().now().nanoseconds / 1000)  # microsegundos

        ocm = OffboardControlMode()
        ocm.timestamp = now
        ocm.position = True
        ocm.velocity = ocm.acceleration = ocm.attitude = ocm.body_rate = False
        self.ocm_pub.publish(ocm)

        sp = TrajectorySetpoint()
        sp.timestamp = now
        sp.position = [0.0, 0.0, -1.0]                 # NED: 1 m arriba
        sp.velocity = [float('nan')]*3
        sp.acceleration = [float('nan')]*3
        sp.yaw = 0.0
        self.sp_pub.publish(sp)

def main():
    rclpy.init()
    node = OffboardStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
