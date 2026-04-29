import os
import re
import numpy as np
import rasterio
from rasterio.merge import merge
from rasterio.windows import from_bounds

try:
    from pyproj import Transformer
except Exception:  # pyproj missing
    Transformer = None


_TILE_RE = re.compile(r"^(?P<e>\d+)_(?P<n>\d+)\.tif$", re.IGNORECASE)


class DemTileProvider:
    """
    Local DEM provider using BY_DEM_1m tiles:
      filename: {easting_km}_{northing_km}.tif
      CRS: EPSG:25832 (meters)
      tile size: 1000m x 1000m

    Maintains an in-memory "active window" mosaic centered on last-request GPS.
    Rebuilds when the request point is within reload_margin_m of the active window edge.
    """

    def __init__(self, tiles_root_dir: str, active_window_size_m=2000.0, reload_margin_m=200.0, logger=None):
        self.tiles_root_dir = tiles_root_dir
        self.active_window_size_m = float(active_window_size_m)
        self.reload_margin_m = float(reload_margin_m)
        self._logger = logger

        self._available = {}  # (e_km, n_km) -> filepath
        self._index_tiles_from_filenames()

        # Coordinate transform: WGS84 -> ETRS89/UTM32N
        self._transformer = None
        if Transformer is not None:
            self._transformer = Transformer.from_crs("EPSG:4326", "EPSG:25832", always_xy=True)
        else:
            self._log_warn("pyproj not available -> local DEM cannot project lat/lon to EPSG:25832.")

        # Active window
        self._active_bbox_m = None  # (Emin, Nmin, Emax, Nmax) meters
        self._active_dataset = None
        self._active_memfile = None

    def _log_info(self, msg):
        if self._logger:
            self._logger.info(msg)

    def _log_warn(self, msg):
        if self._logger:
            self._logger.warning(msg)

    def _log_error(self, msg):
        if self._logger:
            self._logger.error(msg)

    def _index_tiles_from_filenames(self):
        if not os.path.isdir(self.tiles_root_dir):
            self._log_warn(f"Local DEM: tiles_root_dir not found: {self.tiles_root_dir}")
            return

        for fname in os.listdir(self.tiles_root_dir):
            m = _TILE_RE.match(fname)
            if not m:
                continue
            e_km = int(m.group("e"))
            n_km = int(m.group("n"))
            self._available[(e_km, n_km)] = os.path.join(self.tiles_root_dir, fname)

        self._log_info(f"Local DEM: indexed {len(self._available)} tiles from {self.tiles_root_dir}")

    def is_ready(self) -> bool:
        return self._active_dataset is not None and self._active_bbox_m is not None

    def _latlon_to_utm(self, lat, lon):
        if self._transformer is None:
            return None
        E, N = self._transformer.transform(lon, lat)
        return float(E), float(N)

    def _drop_active(self):
        try:
            if self._active_dataset:
                self._active_dataset.close()
        except Exception:
            pass
        try:
            if self._active_memfile:
                self._active_memfile.close()
        except Exception:
            pass
        self._active_dataset = None
        self._active_memfile = None
        self._active_bbox_m = None

    def _required_tile_keys_for_bbox_m(self, bbox_m):
        Emin, Nmin, Emax, Nmax = bbox_m

        e_min_km = int(Emin // 1000)
        e_max_km = int((Emax - 1) // 1000)
        n_min_km = int(Nmin // 1000)
        n_max_km = int((Nmax - 1) // 1000)

        keys = []
        for e in range(e_min_km, e_max_km + 1):
            for n in range(n_min_km, n_max_km + 1):
                keys.append((e, n))
        return keys

    def _build_active_window(self, center_lat, center_lon) -> bool:
        utm = self._latlon_to_utm(center_lat, center_lon)
        if utm is None:
            return False
        E, N = utm

        half = self.active_window_size_m / 2.0
        bbox_m = (E - half, N - half, E + half, N + half)

        keys = self._required_tile_keys_for_bbox_m(bbox_m)
        missing = [k for k in keys if k not in self._available]
        if missing:
            self._log_warn(f"Local DEM: missing {len(missing)} tiles for active window (fallback needed).")
            return False

        paths = [self._available[k] for k in keys]

        try:
            srcs = [rasterio.open(p) for p in paths]
        except Exception as e:
            self._log_error(f"Local DEM: failed opening tiles: {e}")
            return False

        try:
            mosaic, out_transform = merge(srcs)
            out_meta = srcs[0].meta.copy()
            out_meta.update({
                "height": mosaic.shape[1],
                "width": mosaic.shape[2],
                "transform": out_transform
            })

            memfile = rasterio.io.MemoryFile()
            ds = memfile.open(**out_meta)
            ds.write(mosaic)

            for s in srcs:
                s.close()

            # Swap atomically
            self._drop_active()
            self._active_bbox_m = bbox_m
            self._active_memfile = memfile
            self._active_dataset = ds

            self._log_info(f"Local DEM: active window built. bbox_m={self._active_bbox_m}")
            return True

        except Exception as e:
            for s in srcs:
                try:
                    s.close()
                except Exception:
                    pass
            self._log_error(f"Local DEM: failed merging tiles: {e}")
            return False

    def _needs_reload(self, lat, lon) -> bool:
        if self._active_bbox_m is None:
            return True

        utm = self._latlon_to_utm(lat, lon)
        if utm is None:
            return True
        E, N = utm

        Emin, Nmin, Emax, Nmax = self._active_bbox_m
        m = self.reload_margin_m

        return (
            (E - Emin < m) or
            (Emax - E < m) or
            (N - Nmin < m) or
            (Nmax - N < m)
        )

    def ensure_active_window(self, lat, lon) -> bool:
        """
        Explicit "can we create/use local active window for this request GPS?"
        Used by HeightServer to decide local viability.
        """
        if self._needs_reload(lat, lon):
            return self._build_active_window(lat, lon)
        return self.is_ready()

    def get_point_altitude_amsl(self, lat, lon, resolution) -> float:
        """
        Returns -1.0 on failure (matches Copernicus point sentinel).
        resolution is ignored (local tile has fixed 1m resolution).
        """
        if not self.ensure_active_window(lat, lon):
            return -1.0

        utm = self._latlon_to_utm(lat, lon)
        if utm is None:
            return -1.0
        E, N = utm

        try:
            val = next(self._active_dataset.sample([(E, N)]))[0]
            if np.isnan(val) or val is None:
                return -1.0
            # if nodata is -9999, keep it as failure too
            if float(val) <= -9990.0:
                return -1.0
            return float(val)
        except Exception:
            return -1.0

    def get_highest_point_amsl(self, bbox_latlon, resolution, tile_size) -> float:
        """
        bbox_latlon = [minLon, minLat, maxLon, maxLat] in EPSG:4326.
        Returns -np.inf on failure (matches Copernicus max sentinel).
        """
        minLon, minLat, maxLon, maxLat = bbox_latlon

        utm1 = self._latlon_to_utm(minLat, minLon)
        utm2 = self._latlon_to_utm(maxLat, maxLon)
        if utm1 is None or utm2 is None:
            return -np.inf

        E1, N1 = utm1
        E2, N2 = utm2
        bbox_m = (min(E1, E2), min(N1, N2), max(E1, E2), max(N1, N2))

        # Ensure active window around bbox center
        center_lat = (minLat + maxLat) / 2.0
        center_lon = (minLon + maxLon) / 2.0
        if not self.ensure_active_window(center_lat, center_lon):
            return -np.inf

        # Strict policy: require full bbox within active window
        Emin, Nmin, Emax, Nmax = self._active_bbox_m
        if not (bbox_m[0] >= Emin and bbox_m[2] <= Emax and bbox_m[1] >= Nmin and bbox_m[3] <= Nmax):
            return -np.inf

        try:
            window = from_bounds(*bbox_m, transform=self._active_dataset.transform)
            arr = self._active_dataset.read(1, window=window, masked=True)
            if arr.size == 0:
                return -np.inf
            m = arr.max()
            if hasattr(m, "filled"):
                m = m.filled(-np.inf)
            return float(m)
        except Exception:
            return -np.inf