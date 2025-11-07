#!/usr/bin/env python3
import os
import time
import json
import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint

from auspex_msgs.msg import Area, SearchMission


class SearchMissionMockPublisher(Node):
    def __init__(self):
        super().__init__('search_mission_mock_publisher')
        self.publisher = self.create_publisher(SearchMission, 'search_mission', 10)
        self._mission_dict = {}
        self._counter = 0

    def load_mission_file(self, filename):
        with open(filename) as json_file:
            self._mission_dict = json.load(json_file)
            print(self._mission_dict)

        self._msg = SearchMission()
        self._msg.team_id = self._mission_dict['team_id']
        self._msg.platform_class.value = int(self._mission_dict['platform_class']['value'])
        self._msg.mission_status = "UNPROCESSED"

        self._msg.search_areas = []

        for search_area in self._mission_dict['search_areas']:
            area = Area()
            area.type = int(search_area['type'])
            area.description = search_area['description']
            area.points = []

            search_area_points = search_area['points']
            for point in search_area_points:
                area.points.append(GeoPoint(latitude=float(point['latitude']), longitude=float(point['longitude']), altitude=float(point['altitude'])))

            self._msg.search_areas.append(area)

        self._msg.no_fly_zones = []

        for index, no_fly_zone in enumerate(self._mission_dict['no_fly_zones']):
            no_fly_zone = Area()
            no_fly_zone.type = int(self._mission_dict['no_fly_zones'][index]['type'])
            no_fly_zone.description = self._mission_dict['no_fly_zones'][index]['description']
            no_fly_zone_points = self._mission_dict['no_fly_zones'][index]['points']
            no_fly_zone.points = []
            for point in no_fly_zone_points:
                no_fly_zone.points.append(GeoPoint(latitude=float(point['latitude']), longitude=float(point['longitude']), altitude=float(point['altitude'])))
            self._msg.no_fly_zones.append(no_fly_zone)

        self._msg.max_height = int(self._mission_dict['max_height'])
        self._msg.min_height = int(self._mission_dict['min_height'])
        self._msg.desired_ground_dist = int(self._mission_dict['desired_ground_dist'])
        starting_point = self._mission_dict['starting_point']
        self._msg.starting_point = GeoPoint(latitude=float(starting_point['latitude']), longitude=float(starting_point['longitude']), altitude=float(starting_point['altitude']))

        self._msg.mission_goal = self._mission_dict['mission_goal']
        self._msg.target_objects = self._mission_dict['target_objects']

        self._msg.pois = []
        self._msg.prio_areas = []

        self._msg.sensor_mode.value = int(self._mission_dict['sensor_mode']['value'])

        self._msg.danger_zones = []

        print(self._msg)

    def publish(self):
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._msg.header.frame_id = str(self._counter)
        self.publisher.publish(self._msg)
        self._counter += 1
        self.get_logger().info('published new search mission')

def main(args=None):
    rclpy.init(args=args)

    params_dir =  os.getenv('AUSPEX_PARAMS_PATH')
    file_path = os.path.join(params_dir, 'mission', 'example_mission.json')

    searchMissionMockPublisher = SearchMissionMockPublisher()
    searchMissionMockPublisher.load_mission_file(file_path)
    time.sleep(2)  # wait for subscribers to connect
    searchMissionMockPublisher.publish()
    searchMissionMockPublisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
