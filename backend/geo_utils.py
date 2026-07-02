"""GeoJSON loading, lat/lon <-> local-XY projection, and occupancy grid
construction. Mirrors the logic in reference/iitb_digital_twin_matlab.m
(Sections 1, 3, 4) but in Python so it can run inside a normal web backend.
"""
import json
import math
from pathlib import Path

import numpy as np

DATA_FILE = Path(__file__).parent / "data" / "iitb.geojson"

LAT0 = 19.133
LON0 = 72.915
METERS_PER_DEG_LAT = 111320.0


def to_xy(lat, lon):
    x = (lon - LON0) * METERS_PER_DEG_LAT * math.cos(math.radians(LAT0))
    y = (lat - LAT0) * METERS_PER_DEG_LAT
    return x, y


def to_latlon(x, y):
    lon = x / (METERS_PER_DEG_LAT * math.cos(math.radians(LAT0))) + LON0
    lat = y / METERS_PER_DEG_LAT + LAT0
    return lat, lon


def building_height(name):
    """Same name-based height heuristic as the MATLAB script's Section 6
    (no real building-height data in the source GeoJSON, so this is a
    stand-in categorical guess, not a survey).
    """
    n = (name or "").lower()
    if "hostel" in n:
        return 35
    if "aerospace" in n:
        return 30
    if "department" in n:
        return 25
    if "library" in n:
        return 20
    return 12


def _dilate_square(grid, r):
    """Binary dilation with a (2r+1)x(2r+1) square structuring element,
    implemented with plain numpy (avoids a scipy/numpy ABI mismatch seen
    with this environment's system-installed scipy). Separable: dilate
    along rows, then along columns.
    """
    n = grid.shape[0]

    padded = np.zeros((n, n + 2 * r), dtype=bool)
    padded[:, r : r + n] = grid
    row_dilated = np.zeros_like(grid)
    for dx in range(-r, r + 1):
        row_dilated |= padded[:, r + dx : r + dx + n]

    padded2 = np.zeros((n + 2 * r, n), dtype=bool)
    padded2[r : r + n, :] = row_dilated
    result = np.zeros_like(grid)
    for dy in range(-r, r + 1):
        result |= padded2[r + dy : r + dy + n, :]

    return result


def _point_in_polygon(xs, ys, poly_xs, poly_ys):
    """Vectorized PNPOLY (crossing-number) test: xs/ys are 2D arrays of
    pixel-center coordinates, poly_xs/poly_ys are the polygon vertices.
    Returns a boolean array the same shape as xs/ys.
    """
    n = len(poly_xs)
    inside = np.zeros(xs.shape, dtype=bool)
    j = n - 1
    for i in range(n):
        xi, yi = poly_xs[i], poly_ys[i]
        xj, yj = poly_xs[j], poly_ys[j]
        if yi == yj:
            j = i
            continue
        crosses = (yi > ys) != (yj > ys)
        x_at_y = (xj - xi) * (ys - yi) / (yj - yi) + xi
        inside ^= crosses & (xs < x_at_y)
        j = i
    return inside


class CampusData:
    """Loads the GeoJSON once and exposes buildings + an occupancy grid."""

    def __init__(self, map_size=3000, resolution=1, inflate_radius=2):
        self.map_size = map_size
        self.resolution = resolution
        self.inflate_radius = inflate_radius
        self.map_offset = map_size / 2

        self.buildings = []  # list of dicts: {name, ring_latlon, ring_xy}
        self._load()
        self._build_grid()

    def _load(self):
        with open(DATA_FILE) as f:
            data = json.load(f)

        for feature in data["features"]:
            geom = feature.get("geometry")
            if geom is None:
                continue
            props = feature.get("properties", {})
            name = props.get("name", "") or ""

            ring = self._first_ring(geom)
            if ring is None or len(ring) < 3:
                continue

            ring_latlon = [(lat, lon) for lon, lat in ring]  # GeoJSON is [lon, lat]
            ring_xy = [to_xy(lat, lon) for lat, lon in ring_latlon]

            self.buildings.append(
                {
                    "name": name,
                    "ring_latlon": ring_latlon,
                    "ring_xy": ring_xy,
                    "height": building_height(name),
                }
            )

    @staticmethod
    def _first_ring(geom):
        gtype = geom["type"]
        coords = geom["coordinates"]
        if gtype == "Polygon":
            return coords[0]
        if gtype == "MultiPolygon":
            return coords[0][0]
        return None

    def _build_grid(self):
        n = self.map_size
        grid = np.zeros((n, n), dtype=bool)

        for b in self.buildings:
            xs = [p[0] + self.map_offset for p in b["ring_xy"]]
            ys = [p[1] + self.map_offset for p in b["ring_xy"]]
            xmin = max(int(math.floor(min(xs))), 0)
            xmax = min(int(math.ceil(max(xs))), n - 1)
            ymin = max(int(math.floor(min(ys))), 0)
            ymax = min(int(math.ceil(max(ys))), n - 1)
            if xmin > xmax or ymin > ymax:
                continue
            # Rasterize the actual polygon (restricted to its bounding box
            # for speed) instead of filling the whole bbox. A bbox fill
            # blocks off real empty space around non-rectangular/rotated
            # buildings, which was forcing the planner around gaps that
            # are actually open.
            px, py = np.meshgrid(
                np.arange(xmin, xmax + 1) + 0.5, np.arange(ymin, ymax + 1) + 0.5
            )
            mask = _point_in_polygon(px, py, xs, ys)
            grid[ymin : ymax + 1, xmin : xmax + 1] |= mask

        if self.inflate_radius > 0:
            grid = _dilate_square(grid, self.inflate_radius)

        self.grid = grid  # grid[y, x] == True means occupied

    def is_free(self, x_map, y_map):
        xi, yi = int(round(x_map)), int(round(y_map))
        if xi < 0 or xi >= self.map_size or yi < 0 or yi >= self.map_size:
            return False
        return not self.grid[yi, xi]

    def nearest_free_cell(self, x_map, y_map):
        if self.is_free(x_map, y_map):
            return x_map, y_map
        xi, yi = int(round(x_map)), int(round(y_map))
        for r in range(1, 201, 2):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if abs(dx) != r and abs(dy) != r:
                        continue
                    cx, cy = xi + dx, yi + dy
                    if self.is_free(cx, cy):
                        return float(cx), float(cy)
        raise ValueError("No free cell found near the requested point")

    def find_building(self, name):
        name_lower = name.strip().lower()
        for b in self.buildings:
            if b["name"].strip().lower() == name_lower:
                return b
        return None

    def building_centroid_latlon(self, building):
        lats = [p[0] for p in building["ring_latlon"]]
        lons = [p[1] for p in building["ring_latlon"]]
        return (min(lats) + max(lats)) / 2, (min(lons) + max(lons)) / 2

    def to_map_xy(self, lat, lon):
        x, y = to_xy(lat, lon)
        return x + self.map_offset, y + self.map_offset

    def from_map_xy(self, x_map, y_map):
        return to_latlon(x_map - self.map_offset, y_map - self.map_offset)


_campus = None


def get_campus():
    global _campus
    if _campus is None:
        _campus = CampusData()
    return _campus
