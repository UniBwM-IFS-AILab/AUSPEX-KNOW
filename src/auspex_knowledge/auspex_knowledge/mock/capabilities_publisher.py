#!/usr/bin/env python3
import argparse
import random
import rclpy
import time
from rclpy.node import Node

from auspex_msgs.msg import PlatformCapabilities, SensorCapabilities


class CapabilitiesPublisher(Node):

    def __init__(self, number_of_uavs: int):
        super().__init__('capabilities_mock_publisher')

        self._platform_pub = self.create_publisher(PlatformCapabilities, '/platform_capabilities', 1000)
        self._sensor_pub = self.create_publisher(SensorCapabilities, '/sensor_capabilities', 1000)

        self._number_of_uavs = number_of_uavs
        self._sensor_modes = ["eo_camera", "ir_camera"]

        self._sensor_counter = 1

    def publish_once(self):

        for i in range(1, self._number_of_uavs + 1):
            platform_id = f"drone{i}"
            if i == 1:
                platform_id = "hawk1"
            elif i == 2:
                platform_id = "hawk2"
            elif i == 3:
                platform_id = "magpie1"

            platform_msg = PlatformCapabilities()
            platform_msg.platform_id = platform_id
            platform_msg.model_info = f"Mock UAV Model {random.choice(['A', 'B', 'C'])}"
            platform_msg.platform_class = "multicopter"

            platform_msg.max_flight_duration = float(random.randint(20, 45))     # min
            platform_msg.max_flight_height = float(random.randint(80, 200))      # m AGL
            platform_msg.max_velocity = float(random.randint(10, 25))            # m/s
            platform_msg.turning_radius = float(random.randint(20, 60))          # m

            #platform_msg.payload = "first_aid_kit" if random.random() < 0.5 else ""

            if platform_id == "hawk2":
                platform_msg.payload = "first_aid_kit"
            else:
                platform_msg.payload = ""

            self._platform_pub.publish(platform_msg)

            #if i % 2 == 0:
            if platform_id == "hawk1" or platform_id == "magpie1":
                sensor_msg = SensorCapabilities()
                sensor_msg.platform_id = platform_id
                sensor_msg.sensor_id = f"sensor_{self._sensor_counter}"
                self._sensor_counter += 1

                sensor_msg.sensor_mode = random.choice(self._sensor_modes)

                if sensor_msg.sensor_mode == "eo_camera":
                    sensor_msg.fov_hor_min = 20.0
                    sensor_msg.fov_hor_max = 60.0
                    sensor_msg.fov_vert_min = 15.0
                    sensor_msg.fov_vert_max = 45.0
                    sensor_msg.image_width = 1920
                    sensor_msg.image_height = 1080
                else:
                    sensor_msg.fov_hor_min = 10.0
                    sensor_msg.fov_hor_max = 30.0
                    sensor_msg.fov_vert_min = 8.0
                    sensor_msg.fov_vert_max = 24.0
                    sensor_msg.image_width = 640
                    sensor_msg.image_height = 512

                self._sensor_pub.publish(sensor_msg)

        print(f"published platform caps for {self._number_of_uavs} uavs")
        print(f"published sensor caps for {self._number_of_uavs // 2} uavs")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--num", type=int, default=10)
    args = parser.parse_args()

    rclpy.init()
    node = CapabilitiesPublisher(args.num)

    node.publish_once()
    rclpy.spin_once(node, timeout_sec=1.0)
    time.sleep(1.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
