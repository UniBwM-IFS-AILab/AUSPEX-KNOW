import rclpy
from rclpy.node import Node
from oauthlib.oauth2 import BackendApplicationClient
from requests_oauthlib import OAuth2Session
import rasterio
import numpy as np
import os
import io

from auspex_msgs.srv import GetAltitude
from auspex_msgs.srv import GetHeighestPoint
from auspex_msgs.srv import GetLocationsROI


class CopernicusServer(Node):
    def __init__(self):
        super().__init__('copernicus_server')
        self.client_id = os.getenv('COPERNICUS_CLIENT_ID')
        self.client_secret = os.getenv('COPERNICUS_CLIENT_SK') 

        if not self.client_id or not self.client_secret:
            print('error: copernicus client id and secret key not defined in env variables.')
            return

        client = BackendApplicationClient(client_id=self.client_id)
        self.oauth = OAuth2Session(client=client)

        self.token = self.oauth.fetch_token(token_url='https://identity.dataspace.copernicus.eu/auth/realms/CDSE/protocol/openid-connect/token', client_secret=self.client_secret, include_client_id=True)

        self.oauth.get("https://sh.dataspace.copernicus.eu/configuration/v1/wms/instances")

        self.get_altitude_server = self.create_service(GetAltitude, '/auspex_get_altitude', self.get_altitude_server_callback)
        self.get_heighestPoint_server = self.create_service(GetHeighestPoint, '/auspex_get_heighest_point', self.get_heighest_point_server_callback)
        self.get_location_roi = self.create_service(GetLocationsROI, '/auspex_get_locations_roi', self.get_locations_roi_server_callback)

        timer_period = 50  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        print('copernicus server up and runnning...')

    def timer_callback(self):
        print('fetching new token...')
        self.token = self.oauth.fetch_token(token_url='https://identity.dataspace.copernicus.eu/auth/realms/CDSE/protocol/openid-connect/token', client_secret=self.client_secret, include_client_id=True)

    def get_altitude_server_callback(self, request, response):
        print('got a request...')
        latitude = request.gps_position.latitude
        longitude = request.gps_position.longitude
        resolution =  request.resolution

        # define a very small bounding box around the point
        delta = 0.000001
        bounding_box = [longitude - delta, latitude - delta, longitude + delta, latitude + delta]

        print(f'computing the altitude amsl for resolution: {resolution}')
        elevation = self._get_mean_elevation(self.oauth, bounding_box, resolution)
        print(f'computed elevation = {elevation}')
        if elevation != -1:
            response.success = True
            response.altitude_amsl = float(elevation)
        print('finished.')
        return response

    def get_heighest_point_server_callback(self, request, response):
        if len(request.region_bb) != 2:
            print('for the bounding box two points have to be defined')
            return response

        bounding_box = [request.region_bb[0].longitude,  request.region_bb[0].latitude, request.region_bb[1].longitude, request.region_bb[1].latitude]  # [minLon, minLat, maxLon, maxLat]

        resolution = request.resolution # meters
        tile_size = request.tile_size   # square meters

        print(f'computing the heighest point for resolution: {resolution} and tile_size {tile_size}')

        if tile_size == 0:
            tile_size = 500

        if tile_size >1450:
            tile_size = 1400

        tiles = self._split_bbox(bounding_box, tile_size)
        if len(tiles) < 0:
            print('region too small..')

        max_elevation_overall = -np.inf

        for tile_bbox in tiles:
            max_elevation = self._get_max_elevation_from_tile(self.oauth, tile_bbox, resolution)

            if max_elevation > max_elevation_overall:
                max_elevation_overall = max_elevation


        if max_elevation_overall != -np.inf:
            response.success = True
            response.altitude_amsl = float(max_elevation_overall)

        return response


    def get_locations_roi_server_callback(self, request, response):
        return response


    def _get_mean_elevation(self, oauth, bounding_box, resolution=1):
        evalscript = '''
        // VERSION=3
        function setup() {
        return {
            input: ["DEM"],
            output: { id: "default", bands: 1, sampleType: "FLOAT32" }
        };
        }

        function evaluatePixel(sample) {
            return [sample.DEM];
        }
        '''

        request = {
            "input": {
                "bounds": {
                    "bbox": bounding_box
                },
                "data": [
                    {
                        "type": "DEM",
                        "dataFilter": {
                            "timeRange": {
                                "from": "2023-01-01T00:00:00Z",
                                "to": "2023-01-31T23:59:59Z"
                            }
                        }
                    }
                ]
            },
            "output": {
                "resx": resolution,  
                "resy": resolution,  
                "responses": [
                    {
                        "identifier": "default",
                        "format": {
                            "type": "image/tiff"
                        }
                    }
                ]
            },
            "evalscript": evalscript
        }

        url = 'https://sh.dataspace.copernicus.eu/api/v1/process'
        response = oauth.post(url, json=request)
        if response.status_code == 200:
            with io.BytesIO(response.content) as tiff_file:
                with rasterio.open(tiff_file) as src:
                    band = src.read(1)
                    band = np.ma.masked_invalid(band)
                    elevation = band[0, 0]
                    return elevation
        else:
            print(f'request failed with status code {response.status_code}: {response.text}')
        return -1

    def _get_max_elevation_from_tile(self, oauth, tile_bbox, resolution):
        evalscript = '''
            // VERSION=3
            function setup() {
            return {
                input: ["DEM"],
                output: { id: "default", bands: 1, sampleType: "FLOAT32" }
            };
            }

            function evaluatePixel(sample) {
            return [sample.DEM];
            }
        '''

        request = {
            "input": {
                "bounds": {
                    "bbox": tile_bbox
                },
                "data": [
                    {
                        "type": "DEM",
                        "dataFilter": {
                            "timeRange": {
                                "from": "2023-01-01T00:00:00Z",
                                "to": "2023-01-31T23:59:59Z"
                            }
                        }
                    }
                ]
            },
            "output": {
                "resx": resolution,  # Resolution in meters
                "resy": resolution,  # Resolution in meters
                "responses": [
                    {
                        "identifier": "default",
                        "format": {
                            "type": "image/tiff"
                        }
                    }
                ]
            },
            "evalscript": evalscript
        }

        url = 'https://sh.dataspace.copernicus.eu/api/v1/process'
        response = oauth.post(url, json=request)

        if response.status_code == 200:
            with io.BytesIO(response.content) as tiff_file:
                with rasterio.open(tiff_file) as src:
                    band = src.read(1)
                    band = np.ma.masked_invalid(band)
                    return band.max()
        else:
            print(f'request failed for tile {tile_bbox} with status code {response.status_code}: {response.text}')
            return -np.inf

    def _split_bbox(self, bbox, tile_size):
        min_lon, min_lat, max_lon, max_lat = bbox

        tiles = []

        min_lat_rad = np.radians(min_lat)
        max_lat_rad = np.radians(max_lat)

        lon_step = tile_size / (111320 * np.cos((min_lat_rad + max_lat_rad) / 2))
        lat_step = tile_size / 111320

        lon = min_lon
        while lon < max_lon:
            lat = min_lat
            while lat < max_lat:
                tiles.append([lon, lat, min(lon + lon_step, max_lon), min(lat + lat_step, max_lat)])
                lat += lat_step
            lon += lon_step

        return tiles


def main(args=None):
    rclpy.init(args=args)
    copernicus_server = CopernicusServer()
    rclpy.spin(copernicus_server) 
    copernicus_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
