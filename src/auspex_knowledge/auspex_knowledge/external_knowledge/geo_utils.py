import numpy as np

EARTH_METERS_PER_DEG_LAT = 111320.0

def split_bbox(bbox, tile_size_m):
    min_lon, min_lat, max_lon, max_lat = bbox
    tiles = []

    min_lat_rad = np.radians(min_lat)
    max_lat_rad = np.radians(max_lat)

    lon_step = tile_size_m / (EARTH_METERS_PER_DEG_LAT * np.cos((min_lat_rad + max_lat_rad) / 2))
    lat_step = tile_size_m / EARTH_METERS_PER_DEG_LAT

    lon = min_lon
    while lon < max_lon:
        lat = min_lat
        while lat < max_lat:
            tiles.append([lon, lat, min(lon + lon_step, max_lon), min(lat + lat_step, max_lat)])
            lat += lat_step
        lon += lon_step

    return tiles


def normalize_bbox(bbox):
    """Ensure bbox is [minLon, minLat, maxLon, maxLat] even if corners are swapped."""
    min_lon, min_lat, max_lon, max_lat = bbox
    return [min(min_lon, max_lon), min(min_lat, max_lat), max(min_lon, max_lon), max(min_lat, max_lat)]


def bbox_centered(lat, lon, size_m):
    """Create a bbox of width/height size_m meters centered at (lat, lon)."""
    half = size_m / 2.0
    lat_step = half / EARTH_METERS_PER_DEG_LAT
    lon_step = half / (EARTH_METERS_PER_DEG_LAT * np.cos(np.radians(lat)))
    return [lon - lon_step, lat - lat_step, lon + lon_step, lat + lat_step]


def point_near_bbox_edge(lat, lon, bbox, margin_m):
    """
    Returns True if (lat, lon) is within margin_m of any bbox edge.
    Uses a simple meters-per-degree approximation (good enough for small windows).
    """
    bbox = normalize_bbox(bbox)
    min_lon, min_lat, max_lon, max_lat = bbox

    # Convert lon distances at this latitude
    m_per_deg_lon = EARTH_METERS_PER_DEG_LAT * np.cos(np.radians(lat))

    d_left_m  = (lon - min_lon) * m_per_deg_lon
    d_right_m = (max_lon - lon) * m_per_deg_lon
    d_bot_m   = (lat - min_lat) * EARTH_METERS_PER_DEG_LAT
    d_top_m   = (max_lat - lat) * EARTH_METERS_PER_DEG_LAT

    return (d_left_m < margin_m) or (d_right_m < margin_m) or (d_bot_m < margin_m) or (d_top_m < margin_m)