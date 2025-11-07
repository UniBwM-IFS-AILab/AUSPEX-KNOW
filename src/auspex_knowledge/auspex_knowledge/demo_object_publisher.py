#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from auspex_msgs.msg import ObjectKnowledge
from geometry_msgs.msg import Point, Vector3

class DemoObjectPublisher(Node):
    def __init__(self):
        super().__init__('demo_object_publisher')

        self.publisher_ = self.create_publisher(ObjectKnowledge, '/detections', 10)

        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.msg_counter = 0

        self.get_logger().info('demo object publisher started, publishing to /detections')

    def timer_callback(self):
        msg = ObjectKnowledge()

        msg.id = f"object_{self.msg_counter:03d}"
        msg.detection_class = "person"
        msg.priority = 1

        current_time = self.get_clock().now().to_msg()
        msg.time_stamp = current_time

        msg.position = Point()
        msg.position.x = float(48.07613108932041)
        msg.position.y = float(11.63815718815823)
        msg.position.z = float(0.0)

        msg.velocity = Vector3()
        msg.velocity.x = 0.0
        msg.velocity.y = 0.0
        msg.velocity.z = 0.0

        msg.confidence = 0.85
        msg.state = "tracking"

        self.publisher_.publish(msg)

        self.get_logger().info(f'published object: {msg.id} at position ({msg.position.x:.1f}, {msg.position.y:.1f}, {msg.position.z:.1f})')

        self.msg_counter += 1


def main(args=None):
    rclpy.init(args=args)

    demo_publisher = DemoObjectPublisher()

    try:
        rclpy.spin(demo_publisher)
    except KeyboardInterrupt:
        pass

    demo_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
