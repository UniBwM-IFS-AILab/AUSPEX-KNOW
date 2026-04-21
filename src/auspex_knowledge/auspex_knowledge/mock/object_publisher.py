#!/usr/bin/env python3
import argparse
import random
import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from auspex_msgs.msg import ObjectObservation


class ObjectObservationPublisher(Node):

    def __init__(self, number_of_objects: int, number_of_uavs: int):
        super().__init__('object_observation_mock_publisher')

        self._publisher = self.create_publisher(ObjectObservation, '/object_observation', 10)

        self._number_of_objects = number_of_objects
        self._number_of_uavs = number_of_uavs
        self._message_counter = 0

        self._min_lon = 11.62789123072637
        self._max_lon = 11.65043064223953
        self._min_lat = 48.07104825713636
        self._max_lat = 48.07954921425252

        self._classes = ["pedestrian", "cat", "bicycle"]

        self._drones = ["hawk1", "magpie1"]

        self.create_timer(1.0, self._publish_random_object)

    def _random_position(self):
        latitude = random.uniform(self._min_lat, self._max_lat)
        longitude = random.uniform(self._min_lon, self._max_lon)
        return latitude, longitude

    def _publish_random_object(self):

        object_id = f"object{random.randint(1, self._number_of_objects)}"
        platform_id = random.choice(self._drones)
        detection_class = random.choice(self._classes)
        confidence = random.uniform(0.0, 1.0)

        lat, lon = self._random_position()

        msg = ObjectObservation()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self._message_counter)

        msg.object_id = object_id
        msg.platform_id = platform_id
        msg.sensor_id = ""  # leer
        msg.detection_class = detection_class
        msg.confidence = confidence
        msg.estimated_position = GeoPoint(
            latitude=lat,
            longitude=lon,
            altitude=0.0
        )

        if object_id == "object3":
            msg.platform_id = "hawk1"
            msg.detection_class = "pedestrian"
            msg.confidence = 0.79

        self._publisher.publish(msg)

        print(f"published {object_id} from {platform_id} class={detection_class} conf={confidence:.1f}")
        self._message_counter += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_obj", type=int, default=1)
    parser.add_argument("--num_uav", type=int, default=1)
    args = parser.parse_args()

    rclpy.init()
    node = ObjectObservationPublisher(args.num_obj, args.num_uav)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
