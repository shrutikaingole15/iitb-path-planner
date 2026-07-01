import time
from pathlib import Path
from typing import Literal, Optional

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, model_validator

from geo_utils import get_campus
from planner import astar, rrt_star, smooth_path, _dist

app = FastAPI(title="IITB Path Planner")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


class Point(BaseModel):
    lat: Optional[float] = None
    lon: Optional[float] = None
    building: Optional[str] = None

    @model_validator(mode="after")
    def _check(self):
        has_latlon = self.lat is not None and self.lon is not None
        if not has_latlon and not self.building:
            raise ValueError("Point must have either lat/lon or a building name")
        return self


class PlanRequest(BaseModel):
    start: Point
    goal: Point
    algorithm: Literal["astar", "rrtstar"] = "astar"


@app.get("/api/buildings")
def list_buildings():
    campus = get_campus()
    return {
        "buildings": [
            {
                "name": b["name"],
                "ring": [[lat, lon] for lat, lon in b["ring_latlon"]],
            }
            for b in campus.buildings
            if b["name"]
        ]
    }


@app.get("/api/buildings/all_polygons")
def all_polygons():
    """Every polygon (named or not) for drawing the full campus footprint."""
    campus = get_campus()
    return {
        "polygons": [
            [[lat, lon] for lat, lon in b["ring_latlon"]] for b in campus.buildings
        ]
    }


@app.get("/api/buildings/3d")
def buildings_3d():
    """GeoJSON FeatureCollection for a MapLibre fill-extrusion layer.
    Each building's ring plus a name-based height (see geo_utils.building_height)
    since the source data has no real height/floor-count field.
    """
    campus = get_campus()
    features = []
    for b in campus.buildings:
        # GeoJSON ring order is [lon, lat]; also close the ring if needed.
        coords = [[lon, lat] for lat, lon in b["ring_latlon"]]
        if coords[0] != coords[-1]:
            coords.append(coords[0])
        features.append(
            {
                "type": "Feature",
                "properties": {"name": b["name"], "height": b["height"]},
                "geometry": {"type": "Polygon", "coordinates": [coords]},
            }
        )
    return {"type": "FeatureCollection", "features": features}


def _resolve_point(campus, point: Point):
    if point.building:
        b = campus.find_building(point.building)
        if b is None:
            raise HTTPException(status_code=404, detail=f"Building not found: {point.building}")
        lat, lon = campus.building_centroid_latlon(b)
    else:
        lat, lon = point.lat, point.lon
    return lat, lon


@app.post("/api/plan")
def plan_route(req: PlanRequest):
    campus = get_campus()

    start_lat, start_lon = _resolve_point(campus, req.start)
    goal_lat, goal_lon = _resolve_point(campus, req.goal)

    start_map = campus.to_map_xy(start_lat, start_lon)
    goal_map = campus.to_map_xy(goal_lat, goal_lon)

    try:
        start_map = campus.nearest_free_cell(*start_map)
        goal_map = campus.nearest_free_cell(*goal_map)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))

    t0 = time.time()
    if req.algorithm == "astar":
        raw_path = astar(campus, start_map, goal_map)
    else:
        raw_path = rrt_star(campus, start_map, goal_map)
    elapsed = time.time() - t0

    if raw_path is None:
        raise HTTPException(status_code=422, detail="No path found between the given points")

    smoothed = smooth_path(campus, raw_path)

    raw_latlon = [campus.from_map_xy(x, y) for x, y in raw_path]
    smooth_latlon = [campus.from_map_xy(x, y) for x, y in smoothed]

    length_m = sum(_dist(smoothed[i - 1], smoothed[i]) for i in range(1, len(smoothed)))

    return {
        "algorithm": req.algorithm,
        "start": {"lat": start_lat, "lon": start_lon},
        "goal": {"lat": goal_lat, "lon": goal_lon},
        "path_raw": raw_latlon,
        "path_smooth": smooth_latlon,
        "length_m": length_m,
        "compute_seconds": elapsed,
    }


# Serve the frontend as static files at the site root.
FRONTEND_DIR = Path(__file__).parent.parent / "frontend"
app.mount("/", StaticFiles(directory=str(FRONTEND_DIR), html=True), name="frontend")
