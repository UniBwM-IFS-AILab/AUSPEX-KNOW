import os
import rclpy
from rclpy.node import Node
import numpy as np

from auspex_msgs.srv import GetAltitude
from auspex_msgs.srv import GetHighestPoint
from auspex_msgs.srv import GetLocationsROI

from .copernicus_provider import CopernicusProvider
from .dem_tile_provider import DemTileProvider
from .geo_utils import normalize_bbox, split_bbox


class HeightServer(Node):
    def __init__(self):
        super().__init__('height_server')

        # ----------------------------
        # Parameters
        # ----------------------------
        # Default parameter values (regular Python parameters)
        self.mode = "hybrid"  # local_only | online_only | hybrid
        if self.mode not in ("local_only", "online_only", "hybrid"):
            self.get_logger().warning(f"Invalid mode '{self.mode}', defaulting to 'hybrid'")
            self.mode = "hybrid"

        params_dir =  os.getenv('AUSPEX_PARAMS_PATH')
        dem_tiles_dir = os.path.join(params_dir, 'geographic', 'height_data', 'BY_DEM_1m')

        active_window_size_m = 600.0
        reload_margin_m = 100.0

        self.get_logger().info(f"HeightServer mode: {self.mode}")
        self.get_logger().info(f"Local DEM tiles dir: {dem_tiles_dir}")
        self.get_logger().info(f"Active window size [m]: {active_window_size_m}, reload margin [m]: {reload_margin_m}")

        # ----------------------------
        # Providers
        # ----------------------------
        self.copernicus = CopernicusProvider(logger=self.get_logger())
        self.local_dem = DemTileProvider(
            tiles_root_dir=dem_tiles_dir,
            active_window_size_m=active_window_size_m,
            reload_margin_m=reload_margin_m,
            logger=self.get_logger(),
        )

        # ----------------------------
        # Startup: check Copernicus availability first
        # ----------------------------
        self.copernicus_available = False
        if self.mode in ("online_only", "hybrid"):
            if self.copernicus.has_credentials():
                self.copernicus.initialize()
                self.copernicus_available = self.copernicus.check_available()
                if self.copernicus_available:
                    self.get_logger().info("Copernicus: available (startup check passed).")
                else:
                    self.get_logger().warning("Copernicus: NOT available (startup check failed).")
            else:
                self.get_logger().warning("Copernicus credentials missing -> online mode not possible.")
                self.copernicus_available = False

        # ----------------------------
        # ROS Services
        # ----------------------------
        self.get_altitude_server = self.create_service(
            GetAltitude, '/auspex_get_altitude', self.get_altitude_cb
        )
        self.get_highestPoint_server = self.create_service(
            GetHighestPoint, '/auspex_get_highest_point', self.get_highest_point_cb
        )
        self.get_location_roi = self.create_service(
            GetLocationsROI, '/auspex_get_locations_roi', self.get_locations_roi_cb
        )

        # Token refresh timer (only meaningful when online is enabled)
        self.timer = self.create_timer(50.0, self.timer_callback)

        self.get_logger().info("Height server up and running.")

    def timer_callback(self):
        # Only refresh token if online is relevant
        if self.mode in ("online_only", "hybrid") and self.copernicus.has_credentials():
            self.get_logger().info("Refreshing token...")
            self.copernicus.fetch_token_with_retry()
            self.copernicus_available = self.copernicus.is_ready()

    # -------------------------------------------------
    # Routing helpers (policy)
    # -------------------------------------------------
    def _route_point(self, lat, lon, res):
        """
        Returns (success, altitude, source_str)
        source_str in {"local_dem", "copernicus", "none"}
        """

        # ONLINE ONLY
        if self.mode == "online_only":
            if self.copernicus_available:
                alt = self.copernicus.get_point_altitude_amsl(lat, lon, res)
                if alt != -1.0:
                    return True, float(alt), "copernicus"
            return False, -1.0, "none"

        # LOCAL ONLY
        if self.mode == "local_only":
            if self.local_dem.ensure_active_window(lat, lon):
                alt = self.local_dem.get_point_altitude_amsl(lat, lon, res)
                if alt != -1.0:
                    return True, float(alt), "local_dem"
            return False, -1.0, "none"

        # HYBRID
        if self.local_dem.ensure_active_window(lat, lon):
            alt = self.local_dem.get_point_altitude_amsl(lat, lon, res)
            if alt != -1.0:
                return True, float(alt), "local_dem"

        if self.copernicus_available:
            alt = self.copernicus.get_point_altitude_amsl(lat, lon, res)
            if alt != -1.0:
                return True, float(alt), "copernicus"

        return False, -1.0, "none"

    def _route_bbox_max(self, bbox, res, tile_size):
        """
        Returns (success, max_altitude, source_str)
        source_str in {"local_dem", "copernicus", "none"}
        """
        bbox = normalize_bbox(bbox)

        # ONLINE ONLY
        if self.mode == "online_only":
            ok, m = self._copernicus_bbox_max(bbox, res, tile_size)
            return ok, m, ("copernicus" if ok else "none")

        # LOCAL ONLY
        if self.mode == "local_only":
            m = self.local_dem.get_highest_point_amsl(bbox, res, tile_size)
            if m != -np.inf:
                return True, float(m), "local_dem"
            return False, -1.0, "none"

        # HYBRID
        m = self.local_dem.get_highest_point_amsl(bbox, res, tile_size)
        if m != -np.inf:
            return True, float(m), "local_dem"

        ok, m2 = self._copernicus_bbox_max(bbox, res, tile_size)
        return ok, float(m2), ("copernicus" if ok else "none")

    def _copernicus_bbox_max(self, bbox, res, tile_size):
        if not self.copernicus_available:
            return False, -1.0

        if tile_size == 0:
            tile_size = 500
        if tile_size > 1450:
            tile_size = 1400

        tiles = split_bbox(bbox, tile_size)
        if len(tiles) < 1:
            return False, -1.0

        m = self.copernicus.get_highest_point_amsl(bounding_box=bbox, resolution=res, tile_bboxes=tiles)
        if m == -np.inf:
            return False, -1.0
        return True, float(m)

    # -------------------------------------------------
    # ROS Service callbacks
    # -------------------------------------------------
    def get_altitude_cb(self, request, response):
        lat = request.gps_position.latitude
        lon = request.gps_position.longitude
        res = request.resolution

        ok, alt, src = self._route_point(lat, lon, res)
        response.success = bool(ok)
        response.altitude_amsl = float(alt)

        # Log the computed altitude + source
        if ok:
            self.get_logger().info(
                f"GetAltitude: lat={lat:.7f}, lon={lon:.7f}, res={res} -> altitude_amsl={alt:.3f} m (source={src})"
            )
        else:
            self.get_logger().warning(
                f"GetAltitude: lat={lat:.7f}, lon={lon:.7f}, res={res} -> FAILED (mode={self.mode}, source={src})"
            )

        return response

    def get_highest_point_cb(self, request, response):
        if len(request.region_bb) != 2:
            self.get_logger().warning("GetHighestPoint: bounding box must contain exactly 2 points")
            response.success = False
            response.altitude_amsl = -1.0
            return response

        bbox = [
            request.region_bb[0].longitude,
            request.region_bb[0].latitude,
            request.region_bb[1].longitude,
            request.region_bb[1].latitude
        ]
        res = request.resolution
        tile_size = request.tile_size

        ok, m, src = self._route_bbox_max(bbox, res, tile_size)
        response.success = bool(ok)
        response.altitude_amsl = float(m)

        bbox_n = normalize_bbox(bbox)

        # Log max altitude + source
        if ok:
            self.get_logger().info(
                "GetHighestPoint: "
                f"bbox=[{bbox_n[0]:.7f},{bbox_n[1]:.7f},{bbox_n[2]:.7f},{bbox_n[3]:.7f}], "
                f"res={res}, tile_size={tile_size} -> max_altitude_amsl={m:.3f} m (source={src})"
            )
        else:
            self.get_logger().warning(
                "GetHighestPoint: "
                f"bbox=[{bbox_n[0]:.7f},{bbox_n[1]:.7f},{bbox_n[2]:.7f},{bbox_n[3]:.7f}], "
                f"res={res}, tile_size={tile_size} -> FAILED (mode={self.mode}, source={src})"
            )

        return response

    def get_locations_roi_cb(self, request, response):
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HeightServer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node:
                node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()