# IITB Path Planner

Interactive UAV path planner over the IIT Bombay campus. Pick a start and destination
(click the map or choose a building by name) and get a collision-free route computed
around real building footprints, using either **A\*** (grid search, optimal) or
**RRT\*** (sampling-based). View the campus in 2D satellite or a 3D extruded-buildings
view, and animate the planned flight.

## How it works

- `backend/geo_utils.py` — loads campus building footprints from GeoJSON, projects
  lat/lon to local meters, and rasterizes buildings into an inflated occupancy grid.
- `backend/planner.py` — A* (grid search with a heuristic) and RRT* (sampling-based,
  with a spatial-hash index for fast nearest-neighbor queries), plus a shared
  resample/smooth/collision-safe post-processing pass on the resulting path.
- `backend/main.py` — FastAPI app exposing `/api/buildings`, `/api/buildings/3d`,
  and `/api/plan`, and serving the static frontend.
- `frontend/` — Leaflet (2D satellite map + planning UI) and MapLibre GL
  (3D building extrusion) — no build step, plain JS.
- `reference/iitb_digital_twin_matlab.m` — the original MATLAB desktop prototype
  this project was ported from (not used at runtime).

## Run locally

```bash
cd backend
pip install -r requirements.txt
python3 -m uvicorn main:app --host 127.0.0.1 --port 8420
```

Open `http://127.0.0.1:8420/`.

## Deployment

- **Frontend**: static files in `frontend/`, deployable to GitHub Pages (see
  `.github/workflows/deploy-pages.yml`).
- **Backend**: FastAPI app in `backend/`, deployable to any Python host (see
  `render.yaml` for a Render.com blueprint). Update `API_BASE` in
  `frontend/app.js` to point at wherever the backend ends up running.
