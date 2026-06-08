#!/usr/bin/env python3
import math
import statistics
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
import tf2_ros

class MocapPressurePub(Node):
    def __init__(self):
        super().__init__('mocap_pressure_pub')

        self.pressure_pub = self.create_publisher(FluidPressure, "gt_pressure", 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pressure_pub_timer = self.create_timer(0.04, self.pressure_timer_callback)

        self.history_size = 200
        self.sigma_threshold = 3.0
        self.pressure_history = deque(maxlen=self.history_size)

    def pressure_timer_callback(self):
        # lookup transform
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                'sam_mocap/pressure_link',
                rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"Could not lookup transform: {e}")
            return

        # compute pressure
        depth = -trans.transform.translation.z + 0.10
        pressure = (depth * 9.818 * 998) + 100800

        # add to history and check for outlier
        self.pressure_history.append(pressure)
        if len(self.pressure_history) >= 10:
            mean_p = statistics.mean(self.pressure_history)
            stdev_p = statistics.stdev(self.pressure_history)

            # avoid div-by-zero
            if stdev_p > 0 and abs(pressure - mean_p) > self.sigma_threshold * stdev_p:
                self.get_logger().warn(
                    f"Dropping outlier pressure {pressure:.1f} Pa "
                    f"(mean={mean_p:.1f}, σ={stdev_p:.1f})"
                )
                return

        msg = FluidPressure()
        msg.fluid_pressure = pressure
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pressure_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MocapPressurePub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
