import os
import io
import time
import numpy as np
import rasterio

from oauthlib.oauth2 import BackendApplicationClient
from requests_oauthlib import OAuth2Session
from requests.exceptions import ConnectionError, Timeout, RequestException


class CopernicusProvider:
    """
    Extracted Copernicus / OAuth / Process API DEM logic from HeightServer.

    Keeps original behavior:
    - uses client credentials from env vars
    - fetches token with retry
    - provides point elevation (returns band[0,0])
    - provides tile max elevation (returns band.max())
    """

    TOKEN_URL = 'https://identity.dataspace.copernicus.eu/auth/realms/CDSE/protocol/openid-connect/token'
    PROCESS_URL = 'https://sh.dataspace.copernicus.eu/api/v1/process'
    CONFIG_TEST_URL = "https://sh.dataspace.copernicus.eu/configuration/v1/wms/instances"

    def __init__(self, logger=None):
        """
        logger: optional object that supports .info/.warning/.error (e.g., rclpy logger).
        """
        self._logger = logger

        self.client_id = os.getenv('COPERNICUS_CLIENT_ID')
        self.client_secret = os.getenv('COPERNICUS_CLIENT_SK')

        self.oauth = None
        self.token = None

        if self.client_id and self.client_secret:
            client = BackendApplicationClient(client_id=self.client_id)
            self.oauth = OAuth2Session(client=client)

    def _log_info(self, msg: str):
        if self._logger:
            self._logger.info(msg)

    def _log_warn(self, msg: str):
        if self._logger:
            self._logger.warning(msg)

    def _log_error(self, msg: str):
        if self._logger:
            self._logger.error(msg)

    def has_credentials(self) -> bool:
        return bool(self.client_id and self.client_secret)

    def initialize(self) -> bool:
        """
        Initialize Copernicus access:
        - fetch token with retry
        - run a lightweight connection test (same as your original)
        """
        if not self.has_credentials():
            self._log_error('Copernicus client id and secret key not defined in env variables.')
            return False

        ok = self.fetch_token_with_retry()
        if not ok:
            # keep going but mark as unavailable via token==None
            return False

        # Test connection (same behavior as original: warn on failure, do not throw)
        try:
            self.oauth.get(self.CONFIG_TEST_URL)
        except Exception as e:
            self._log_warn(f'Initial connection test failed: {e}')

        return True
    
    def check_available(self) -> bool:
        """
        Stronger than just "has credentials":
        - ensures we can fetch token
        - ensures we can reach Copernicus endpoint
        """
        if not self.has_credentials():
            return False

        if not self.token:
            if not self.fetch_token_with_retry():
                return False

        try:
            r = self.oauth.get(self.CONFIG_TEST_URL, timeout=20)
            # Any HTTP response means networking + auth header path is basically working.
            # (Even if endpoint changes status codes, connection matters here.)
            _ = r.status_code
        except Exception as e:
            self._log_warn(f'Copernicus availability check failed: {e}')
            return False

        return self.is_ready()

    def fetch_token_with_retry(self, max_retries=3) -> bool:
        """
        Same as original _fetch_token_with_retry, moved here.
        """
        if not self.oauth:
            self._log_error("OAuth session not initialized (missing credentials).")
            return False

        for attempt in range(max_retries):
            try:
                self.token = self.oauth.fetch_token(
                    token_url=self.TOKEN_URL,
                    client_secret=self.client_secret,
                    include_client_id=True,
                    timeout=30
                )
                self._log_info('Token refreshed successfully')
                return True
            except (ConnectionError, Timeout, RequestException) as e:
                self._log_warn(f'Token refresh attempt {attempt + 1}/{max_retries} failed: {e}')
                if attempt < max_retries - 1:
                    time.sleep(2 ** attempt)
                else:
                    self._log_error('All token refresh attempts failed. Services may be unavailable.')
            except Exception as e:
                self._log_error(f'Unexpected error during token refresh: {e}')
                break
        return False

    def is_ready(self) -> bool:
        """
        Provider considered ready if it has an OAuth session and a token.
        """
        return self.oauth is not None and self.token is not None

    def get_point_altitude_amsl(self, latitude: float, longitude: float, resolution: int) -> float:
        """
        Equivalent to original altitude flow:
        - create tiny bbox around point
        - call _get_mean_elevation (returns band[0,0])
        - returns -1.0 on failure
        """
        if not self.is_ready():
            return -1.0

        delta = 0.000001
        bounding_box = [longitude - delta, latitude - delta, longitude + delta, latitude + delta]
        return self._get_point_elevation_from_bbox(self.oauth, bounding_box, resolution)

    def get_highest_point_amsl(self, bounding_box, resolution: int, tile_bboxes) -> float:
        """
        Compute overall max elevation across a list of tile bounding boxes.
        Returns -inf on failure (matches original sentinel usage).
        """
        if not self.is_ready():
            return -np.inf

        max_elevation_overall = -np.inf
        for tile_bbox in tile_bboxes:
            max_elevation = self._get_max_elevation_from_tile(self.oauth, tile_bbox, resolution)
            if max_elevation > max_elevation_overall:
                max_elevation_overall = max_elevation

        return max_elevation_overall

    def _get_point_elevation_from_bbox(self, oauth, bounding_box, resolution=1):
        # This is your original evalscript (kept)
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
                "bounds": {"bbox": bounding_box},
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
                    {"identifier": "default", "format": {"type": "image/tiff"}}
                ]
            },
            "evalscript": evalscript
        }

        try:
            response = oauth.post(self.PROCESS_URL, json=request, timeout=60)
            if response.status_code == 200:
                with io.BytesIO(response.content) as tiff_file:
                    with rasterio.open(tiff_file) as src:
                        band = src.read(1)
                        band = np.ma.masked_invalid(band)
                        elevation = band[0, 0]  # kept exactly as before
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
                "bounds": {"bbox": tile_bbox},
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
                    {"identifier": "default", "format": {"type": "image/tiff"}}
                ]
            },
            "evalscript": evalscript
        }

        try:
            response = oauth.post(self.PROCESS_URL, json=request, timeout=60)
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