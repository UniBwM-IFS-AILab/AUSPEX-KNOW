#!/usr/bin/env python3
import math
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion
from geographic_msgs.msg import GeoPoint
from tf_transformations import quaternion_from_euler
from auspex_msgs.msg import PlatformState, SensorMode


class PlatformStatePublisher(Node):

    def __init__(self, num_of_uavs: int):
        super().__init__('platform_state_publisher')

        self._publisher = self.create_publisher(PlatformState, '/platform_state', 1000)
        self._number_of_uavs = num_of_uavs
        self._message_counter = 0

        self._goal_latitude = 48.075345
        self._goal_longitude = 11.639462

        self._uavs = []

        base_latitude = 48.080934
        base_longitude = 11.634175

        meters_per_degree_lon = 111320.0 * math.cos(math.radians(base_latitude))
        one_meter_in_lon = 1.0 / meters_per_degree_lon

        for i in range(self._number_of_uavs):
            if i == 0:
                platform_id = "hawk1"
            elif i == 1:
                platform_id = "hawk2"
            elif i == 2:
                platform_id = "magpie1"
            platform_id
            self._uavs.append({
                #"platform_id": f"drone{i+1}",
                "platform_id": platform_id,
                "latitude": base_latitude,
                "longitude": base_longitude + i * one_meter_in_lon,
                "status": "landed"
            })

    def start(self):
        self.create_timer(0.1, self._publish_states)

    def _publish_states(self):

        for uav in self._uavs:

            #if abs(uav["latitude"] - self._goal_latitude) < 0.001:
            #    uav["status"] = "landed"
            #else:
            #    uav["latitude"] -= 0.000010
            #    uav["longitude"] += 0.000010
            #    uav["status"] = "flying"

            if uav["platform_id"] == "hawk1":
                uav["latitude"] -= 0.000010
                uav["longitude"] += 0.000010
                uav["altitude"] = 25.0
                uav["status"] = "flying"
            elif uav["platform_id"] == "hawk2":
                uav["altitude"] = 0.0
                uav["status"] = "landed"
            elif uav["platform_id"] == "magpie1":
                uav["latitude"] -= 0.000010
                uav["longitude"] += 0.000010
                uav["altitude"] = 31.0
                uav["status"] = "flying"
            msg = PlatformState()
            msg.platform_id = uav["platform_id"]
            msg.team_id = "team1"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = str(self._message_counter)

            msg.platform_gps_position = GeoPoint(
                latitude=uav["latitude"],
                longitude=uav["longitude"],
                altitude=uav["altitude"]
            )

            yaw_rad = math.radians(135.0)
            q = quaternion_from_euler(0, 0, yaw_rad)

            msg.platform_pose.orientation = Quaternion(
                x=q[0], y=q[1], z=q[2], w=q[3]
            )
            msg.platform_pose.position = Point(x=0.0, y=0.0, z=0.0)

            msg.platform_status = uav["status"]

            self._publisher.publish(msg)

        print(f"published {self._number_of_uavs} uavs | msg #{self._message_counter}")
        self._message_counter += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--num", type=int, default=1)
    args = parser.parse_args()

    rclpy.init()

    node = PlatformStatePublisher(args.num)
    node.start()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
