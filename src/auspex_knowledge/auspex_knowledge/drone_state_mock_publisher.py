#!/usr/bin/env python3
import math
import os
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion
from geographic_msgs.msg import GeoPoint
from tf_transformations import quaternion_from_euler
from auspex_msgs.srv import InsertKnowledge

from msg_context.loader import DroneState, SensorMode


class DroneStateMockPublisher(Node):
    def __init__(self):
        super().__init__('drone_state_mock_publisher')

        self._state_pub = self.create_publisher(DroneState, '/drone_state', 10)
        self._insert_client = self.create_client(InsertKnowledge, '/insert_knowledge')

        self._platform_id1 = 'drone1'
        self.lat_drone1 = 48.080934
        self.lon_drone1 = 11.634175

        self._platform_id2 = 'drone2'
        self.lat_drone2 = 48.080934
        self.lon_drone2 = 11.634175

        self.lat_goal = 48.075345
        self.lon_goal = 11.639462

        self.platform_status_drone1 = 'hover'
        self.platform_status_drone2 = 'hover'

        self._msg_counter = 0

    def register_platforms(self):
        self._executor = SingleThreadedExecutor()
        if not self._insert_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('error: no connection to knowledge service')
            return False

        insert_request = InsertKnowledge.Request()
        insert_request.collection = 'platform'
        insert_request.path = '$'
        insert_request.entity = '{"platform_id":"'+ self._platform_id1 + '"}'
        future = self._insert_client.call_async(insert_request)
        rclpy.spin_until_future_complete(self, future, self._executor)
        if not future.result().success:
            return False

        insert_request = InsertKnowledge.Request()
        insert_request.collection = 'platform'
        insert_request.path = '$'
        insert_request.entity = '{"platform_id":"'+ self._platform_id2 + '"}'
        future = self._insert_client.call_async(insert_request)
        rclpy.spin_until_future_complete(self, future, self._executor)
        return future.result().success

    def publish_caps(self):
        #TODO
        return

    def start_state_pub(self):
        self.publish_timer = self.create_timer(0.1, self.publish_drone_state)

    def publish_drone_state(self):
        if abs(self.lat_drone1-self.lat_goal) < 0.001 and abs(self.lon_drone1-self.lon_goal) < 0.001:
            self.platform_status_drone1 = 'landing'
        else:
            self.lat_drone1 -= 0.000010
            self.lon_drone1 += 0.000010
            self.platform_status_drone1 = 'flying'

        if abs(self.lon_drone2-self.lon_goal) < 0.001:
            self.platform_status_drone2 = 'landed'
        else:
            self.lat_drone2 -= 0.000010
            self.lon_drone2 -= 0.000010
            self.platform_status_drone2 = 'flying'

        msg0 = DroneState()
        msg0.platform_id = self._platform_id1
        msg0.team_id = 'team1'
        msg0.header.stamp = self.get_clock().now().to_msg()
        msg0.header.frame_id = str(self._msg_counter)
        msg0.platform_gps_position = GeoPoint(latitude=self.lat_drone1, longitude=self.lon_drone1, altitude=80.0)

        angle_in_degrees = 90.0 + 55.0
        angle_in_radians = math.radians(angle_in_degrees)
        quaternion = quaternion_from_euler(0, 0, angle_in_radians)
        msg0.platform_pose.orientation = Quaternion()
        msg0.platform_pose.orientation.x = quaternion[0]
        msg0.platform_pose.orientation.y = quaternion[1]
        msg0.platform_pose.orientation.z = quaternion[2]
        msg0.platform_pose.orientation.w = quaternion[3]

        msg0.platform_pose.position = Point(x=0.0, y=0.0, z=0.0)

        msg0.platform_status = self.platform_status_drone1

        msg0.sensor_mode.value = SensorMode.SENSOR_MODE_EO

        self._state_pub.publish(msg0)

        msg1 = DroneState()
        msg1.platform_id = self._platform_id2
        msg1.team_id = 'team1'
        msg1.header.stamp = self.get_clock().now().to_msg()
        msg1.header.frame_id = str(self._msg_counter)
        msg1.platform_gps_position = GeoPoint(latitude=self.lat_drone2, longitude=self.lon_drone2, altitude=80.0)

        angle_in_degrees = 180.0 + 45.0
        angle_in_radians = math.radians(angle_in_degrees)
        quaternion = quaternion_from_euler(0, 0, angle_in_radians)
        msg1.platform_pose.orientation = Quaternion()
        msg1.platform_pose.orientation.x = quaternion[0]
        msg1.platform_pose.orientation.y = quaternion[1]
        msg1.platform_pose.orientation.z = quaternion[2]
        msg1.platform_pose.orientation.w = quaternion[3]

        msg1.platform_pose.position = Point(x=0.0, y=0.0, z=0.0)

        msg1.platform_status = self.platform_status_drone2

        msg1.sensor_mode.value = SensorMode.SENSOR_MODE_EO

        self._state_pub.publish(msg1)

        print('publishing: ' + str(self._msg_counter))
        self._msg_counter += 1

def main(args=None):
    rclpy.init(args=args)

    droneStateMockPublisher = DroneStateMockPublisher()

    if droneStateMockPublisher.register_platforms():
        print('register platform successful...')
        #droneStateMockPublisher.publish_caps()
        droneStateMockPublisher.start_state_pub()
        rclpy.spin(droneStateMockPublisher)
    else:
        print('register platform not successful...')
    droneStateMockPublisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
