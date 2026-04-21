#!/usr/bin/env python3
import argparse
import random
import time
import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from auspex_msgs.msg import Waypoint


class WaypointPublisher(Node):

    def __init__(self, number_of_waypoints: int):
        super().__init__('waypoint_mock_publisher')

        self._publisher = self.create_publisher(Waypoint, '/waypoints', 1000)
        self._number_of_waypoints = number_of_waypoints

        self._min_lon = 11.62789123072637
        self._max_lon = 11.65043064223953
        self._min_lat = 48.07104825713636
        self._max_lat = 48.07954921425252

    def publish_once(self):

        for i in range(1, self._number_of_waypoints + 1):

            lat = random.uniform(self._min_lat, self._max_lat)
            lon = random.uniform(self._min_lon, self._max_lon)

            msg = Waypoint()
            msg.waypoint_id = f"waypoint{i}"
            msg.gps_position = GeoPoint(
                latitude=lat,
                longitude=lon,
                altitude=0.0
            )

            self._publisher.publish(msg)

        print(f"published {self._number_of_waypoints} waypoints")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--num", type=int, default=1)
    args = parser.parse_args()

    rclpy.init()
    node = WaypointPublisher(args.num)

    node.publish_once()
    rclpy.spin_once(node, timeout_sec=1.0)
    time.sleep(1.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
