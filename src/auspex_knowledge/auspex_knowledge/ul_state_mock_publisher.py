#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Point, Quaternion
from geographic_msgs.msg import GeoPoint
from tf_transformations import quaternion_from_euler
from auspex_msgs.srv import InsertKnowledge

from msg_context.loader import DroneState, PlatformCapabilities, SensorCapabilities, SensorMode


class UlStateMockPublisher(Node):
    def __init__(self):
        super().__init__('ul_state_mock_publisher')

        self._state_pub = self.create_publisher(DroneState, '/drone_state', 10)
        self._caps_pub = self.create_publisher(PlatformCapabilities, '/platform_capabilities', 10)
        self._insert_client = self.create_client(InsertKnowledge, '/insert_knowledge')

        # starting position
        self.lat_ul = 48.080934
        self.lon_ul = 11.634175

        # fake goal
        self.lat_goal = 48.075345
        self.lon_goal = 11.639462

        self._platform_id = 'ul_hummingbird'

        self._platform_status_ul = 'starting'

        self._msg_counter = 0

    def register_platform(self):
        self._executor = SingleThreadedExecutor()
        if not self._insert_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('error: no connection to knowledge service')
            return False
        insert_request = InsertKnowledge.Request()
        insert_request.collection = 'platform'
        insert_request.path = '$'
        insert_request.entity = '{"platform_id":"'+ self._platform_id + '"}'
        future = self._insert_client.call_async(insert_request)
        rclpy.spin_until_future_complete(self, future, self._executor)
        return future.result().success

    def publish_caps(self):
        caps_msg = PlatformCapabilities()
        caps_msg.platform_id = self._platform_id
        caps_msg.model_info = 'some kind of UL'
        caps_msg.max_flight_duration  = 60.0
        caps_msg.max_flight_height = 1000.0
        caps_msg.max_velocity = 100.0
        caps_msg.turning_radius = 100.0

        sensor_caps_msg = SensorCapabilities()
        sensor_caps_msg.sensor_id = 'ul_camera'
        sensor_caps_msg.sensor_mode.value = SensorMode.SENSOR_MODE_EO
        sensor_caps_msg.fov_hor_min = 30.0
        sensor_caps_msg.fov_hor_max = 30.0
        sensor_caps_msg.fov_vert_min = 30.0
        sensor_caps_msg.fov_vert_max = 30.0
        sensor_caps_msg.image_width = 600
        sensor_caps_msg.image_height = 600

        caps_msg.sensor_caps.append(sensor_caps_msg)

        self._caps_pub.publish(caps_msg)

    def start_state_pub(self):
        self.publish_timer = self.create_timer(0.1, self.publish_ul_state)

    def publish_ul_state(self):
        if abs(self.lat_ul-self.lat_goal) < 0.001 and abs(self.lon_ul-self.lon_goal) < 0.001:
            self._platform_status_ul = 'landing'
        else:
            self.lat_ul -= 0.000010
            self.lon_ul += 0.000010
            self._platform_status_ul = 'flying'

        msg0 = DroneState()
        msg0.platform_id = self._platform_id
        msg0.team_id = 'ul_team'
        msg0.header.stamp = self.get_clock().now().to_msg()
        msg0.header.frame_id = str(self._msg_counter)
        msg0.platform_gps_position = GeoPoint(latitude=self.lat_ul, longitude=self.lon_ul, altitude=80.0)

        angle_in_degrees = 90.0 + 55.0
        angle_in_radians = math.radians(angle_in_degrees)
        quaternion = quaternion_from_euler(0, 0, angle_in_radians)
        msg0.platform_pose.orientation = Quaternion()
        msg0.platform_pose.orientation.x = quaternion[0]
        msg0.platform_pose.orientation.y = quaternion[1]
        msg0.platform_pose.orientation.z = quaternion[2]
        msg0.platform_pose.orientation.w = quaternion[3]

        msg0.platform_pose.position = Point(x=0.0, y=0.0, z=0.0)

        msg0.platform_status = self._platform_status_ul

        msg0.sensor_mode.value = SensorMode.SENSOR_MODE_EO

        self._state_pub.publish(msg0)

        print('publishing: ' + str(self._msg_counter))
        self._msg_counter += 1

def main(args=None):
    rclpy.init(args=args)

    ulStateMockPublisher = UlStateMockPublisher()

    if ulStateMockPublisher.register_platform():
        print('register platform successful...')
        ulStateMockPublisher.publish_caps()
        ulStateMockPublisher.start_state_pub()
        rclpy.spin(ulStateMockPublisher)
    else:
        print('register platform not successful...')

    ulStateMockPublisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
