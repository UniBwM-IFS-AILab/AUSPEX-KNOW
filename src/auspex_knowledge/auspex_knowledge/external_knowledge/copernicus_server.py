import rclpy
from rclpy.node import Node
from oauthlib.oauth2 import BackendApplicationClient
from requests_oauthlib import OAuth2Session
import rasterio
import numpy as np
import os
import io
import requests
from requests.exceptions import ConnectionError, Timeout, RequestException

from auspex_msgs.srv import GetAltitude
from auspex_msgs.srv import GetHighestPoint
from auspex_msgs.srv import GetLocationsROI


class CopernicusServer(Node):
    def __init__(self):
        super().__init__('copernicus_server')
        self.client_id = os.getenv('COPERNICUS_CLIENT_ID')
        self.client_secret = os.getenv('COPERNICUS_CLIENT_SK')

        if not self.client_id or not self.client_secret:
            self.get_logger().error('Copernicus client id and secret key not defined in env variables.')
            return

        client = BackendApplicationClient(client_id=self.client_id)
        self.oauth = OAuth2Session(client=client)

        # Initialize token with error handling
        self.token = None
        self._fetch_token_with_retry()

        # Test connection with error handling
        try:
            self.oauth.get("https://sh.dataspace.copernicus.eu/configuration/v1/wms/instances")
        except Exception as e:
            self.get_logger().warning(f'Initial connection test failed: {e}')

        self.get_altitude_server = self.create_service(GetAltitude, '/auspex_get_altitude', self.get_altitude_server_callback)
        self.get_highest_point_server = self.create_service(GetHighestPoint, '/auspex_get_highest_point', self.get_highest_point_server_callback)
        self.get_location_roi = self.create_service(GetLocationsROI, '/auspex_get_locations_roi', self.get_locations_roi_server_callback)

        timer_period = 50  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Copernicus server up and running...')

    def timer_callback(self):
        self.get_logger().info('Refreshing token...')
        self._fetch_token_with_retry()

    def _fetch_token_with_retry(self, max_retries=3):
        """Fetch token with retry logic and error handling"""
        for attempt in range(max_retries):
            try:
                self.token = self.oauth.fetch_token(
                    token_url='https://identity.dataspace.copernicus.eu/auth/realms/CDSE/protocol/openid-connect/token',
                    client_secret=self.client_secret,
                    include_client_id=True,
                    timeout=30  # Add timeout
                )
                self.get_logger().info('Token refreshed successfully')
                return True
            except (ConnectionError, Timeout, RequestException) as e:
                self.get_logger().warning(f'Token refresh attempt {attempt + 1}/{max_retries} failed: {e}')
                if attempt < max_retries - 1:
                    # Wait before retry (exponential backoff)
                    import time
                    time.sleep(2 ** attempt)
                else:
                    self.get_logger().error('All token refresh attempts failed. Services may be unavailable.')
            except Exception as e:
                self.get_logger().error(f'Unexpected error during token refresh: {e}')
                break
        return False

    def get_altitude_server_callback(self, request, response):
        self.get_logger().info('Received altitude request')
        try:
            if not self.token:
                self.get_logger().warning('No valid token available')
                response.success = False
                return response

            latitude = request.gps_position.latitude
            longitude = request.gps_position.longitude
            resolution = request.resolution

            # define a very small bounding box around the point
            delta = 0.000001
            bounding_box = [longitude - delta, latitude - delta, longitude + delta, latitude + delta]

            self.get_logger().info(f'Computing altitude AMSL for resolution: {resolution}')
            elevation = self._get_mean_elevation(self.oauth, bounding_box, resolution)
            self.get_logger().info(f'Computed elevation = {elevation}')

            if elevation != -1:
                response.success = True
                response.altitude_amsl = float(elevation)
            else:
                response.success = False
                response.altitude_amsl = float(elevation)
                self.get_logger().warning('Failed to compute elevation')
        except Exception as e:
            self.get_logger().error(f'Error in altitude service: {e}')
            response.success = False

        self.get_logger().info('Altitude request finished')
        return response

    def get_highest_point_server_callback(self, request, response):
        try:
            if not self.token:
                self.get_logger().warning('No valid token available')
                response.success = False
                return response

            if len(request.region_bb) != 2:
                self.get_logger().warning('For the bounding box two points have to be defined')
                response.success = False
                return response

            bounding_box = [request.region_bb[0].longitude,  request.region_bb[0].latitude, request.region_bb[1].longitude, request.region_bb[1].latitude]  # [minLon, minLat, maxLon, maxLat]

            resolution = request.resolution # meters
            tile_size = request.tile_size   # square meters

            self.get_logger().info(f'Computing highest point for resolution: {resolution} and tile_size {tile_size}')

            if tile_size == 0:
                tile_size = 500

            if tile_size > 1450:
                tile_size = 1400

            tiles = self._split_bbox(bounding_box, tile_size)
            if len(tiles) < 1:
                self.get_logger().warning('Region too small or no tiles generated')
                response.success = False
                return response

            max_elevation_overall = -np.inf

            for tile_bbox in tiles:
                max_elevation = self._get_max_elevation_from_tile(self.oauth, tile_bbox, resolution)
                if max_elevation > max_elevation_overall:
                    max_elevation_overall = max_elevation

            if max_elevation_overall != -np.inf:
                response.success = True
                response.altitude_amsl = float(max_elevation_overall)
            else:
                response.success = False
                self.get_logger().warning('Failed to compute highest point')

        except Exception as e:
            self.get_logger().error(f'Error in highest point service: {e}')
            response.success = False

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
        try:
            response = oauth.post(url, json=request, timeout=60)
            if response.status_code == 200:
                with io.BytesIO(response.content) as tiff_file:
                    with rasterio.open(tiff_file) as src:
                        band = src.read(1)
                        band = np.ma.masked_invalid(band)
                        elevation = band[0, 0]
                        return elevation
            else:
                print(f'Request failed with status code {response.status_code}: {response.text}')
        except (ConnectionError, Timeout, RequestException) as e:
            print(f'Network error during elevation request: {e}')
        except Exception as e:
            print(f'Unexpected error during elevation request: {e}')
        return -1.0

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
        try:
            response = oauth.post(url, json=request, timeout=60)
            if response.status_code == 200:
                with io.BytesIO(response.content) as tiff_file:
                    with rasterio.open(tiff_file) as src:
                        band = src.read(1)
                        band = np.ma.masked_invalid(band)
                        return band.max()
            else:
                print(f'Request failed for tile {tile_bbox} with status code {response.status_code}: {response.text}')
        except (ConnectionError, Timeout, RequestException) as e:
            print(f'Network error for tile {tile_bbox}: {e}')
        except Exception as e:
            print(f'Unexpected error for tile {tile_bbox}: {e}')
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
    try:
        copernicus_server = CopernicusServer()
        if copernicus_server.client_id and copernicus_server.client_secret:
            print('Copernicus server initialized successfully')
            rclpy.spin(copernicus_server)
        else:
            print('Failed to initialize Copernicus server due to missing credentials')
    except KeyboardInterrupt:
        print('Shutting down Copernicus server...')
    except Exception as e:
        print(f'Unexpected error in main: {e}')
    finally:
        try:
            copernicus_server.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
