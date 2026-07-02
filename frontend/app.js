// When the backend is served by the same process as this page (e.g. running
// locally via `uvicorn main:app`), same-origin ("") works. When deployed to
// GitHub Pages, the frontend is on a different origin than the backend, so
// point it at the Render deployment instead.
const isLocal = ["localhost", "127.0.0.1"].includes(location.hostname);
const API_BASE = isLocal ? "" : "https://iitb-path-planner.onrender.com";
const CAMPUS_CENTER = [19.133, 72.915];

const map = L.map("map", { zoomControl: true }).setView(CAMPUS_CENTER, 16);

L.tileLayer(
  "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
  {
    maxZoom: 19,
    attribution: "Tiles &copy; Esri",
  }
).addTo(map);

const buildingsLayer = L.layerGroup().addTo(map);
const routeLayer = L.layerGroup().addTo(map);

let allBuildings = []; // {name, ring}
let startPoint = null; // {lat, lon}
let goalPoint = null;
let startMarker = null;
let goalMarker = null;
let lastPlanResult = null;
let uavMarker = null;
let uavMarker3d = null;
let animTimer = null;

const startSelect = document.getElementById("start-building");
const goalSelect = document.getElementById("goal-building");
const startCoords = document.getElementById("start-coords");
const goalCoords = document.getElementById("goal-coords");
const planBtn = document.getElementById("btn-plan");
const clearBtn = document.getElementById("btn-clear");
const animateBtn = document.getElementById("btn-animate");
const statusEl = document.getElementById("status");
const resultEl = document.getElementById("result");
const resLength = document.getElementById("res-length");
const resTime = document.getElementById("res-time");

let algorithm = "astar";
document.querySelectorAll("[data-algo]").forEach((btn) => {
  btn.addEventListener("click", () => {
    document.querySelectorAll("[data-algo]").forEach((b) => b.classList.remove("active"));
    btn.classList.add("active");
    algorithm = btn.dataset.algo;
  });
});

// --- 3D view (MapLibre GL, fill-extrusion buildings) ---------------------
const map2dEl = document.getElementById("map");
const map3dEl = document.getElementById("map3d");
let map3d = null;
let map3dLoaded = false;

function ensureMap3d() {
  if (map3d) return;
  map3d = new maplibregl.Map({
    container: "map3d",
    style: {
      version: 8,
      sources: {
        satellite: {
          type: "raster",
          tiles: [
            "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
          ],
          tileSize: 256,
          attribution: "Tiles &copy; Esri",
        },
      },
      layers: [{ id: "satellite", type: "raster", source: "satellite" }],
    },
    center: [CAMPUS_CENTER[1], CAMPUS_CENTER[0]],
    zoom: 16,
    pitch: 60,
    bearing: -20,
    antialias: true,
  });

  map3d.addControl(new maplibregl.NavigationControl());

  const setupLayers = async () => {
    const res = await fetch(`${API_BASE}/api/buildings/3d`);
    const buildingsGeojson = await res.json();

    map3d.addSource("buildings3d", { type: "geojson", data: buildingsGeojson });
    map3d.addLayer({
      id: "buildings-3d",
      type: "fill-extrusion",
      source: "buildings3d",
      paint: {
        "fill-extrusion-height": ["get", "height"],
        "fill-extrusion-base": 0,
        "fill-extrusion-color": "#8ab4ff",
        "fill-extrusion-opacity": 0.85,
      },
    });

    map3d.addSource("route3d", { type: "geojson", data: emptyLineFeature() });
    map3d.addLayer({
      id: "route-3d",
      type: "line",
      source: "route3d",
      paint: { "line-color": "#00e5ff", "line-width": 4 },
      layout: { "line-cap": "round", "line-join": "round" },
    });

    map3d.addSource("uavtrail3d", { type: "geojson", data: emptyLineFeature() });
    map3d.addLayer({
      id: "uav-trail-3d",
      type: "line",
      source: "uavtrail3d",
      paint: { "line-color": "#ffe082", "line-width": 3 },
      layout: { "line-cap": "round", "line-join": "round" },
    });

    map3dLoaded = true;
    syncRouteTo3d();
  };

  // Since the style is passed as an object (not a URL), MapLibre can fire
  // "load" synchronously before this listener is attached — check
  // isStyleLoaded() first instead of relying solely on the event.
  if (map3d.isStyleLoaded()) {
    setupLayers();
  } else {
    map3d.once("load", setupLayers);
  }
}

function emptyLineFeature() {
  return { type: "Feature", geometry: { type: "LineString", coordinates: [] }, properties: {} };
}

function syncRouteTo3d() {
  if (!map3d || !map3dLoaded) return;
  const src = map3d.getSource("route3d");
  if (!src) return;
  if (!lastPlanResult) {
    src.setData(emptyLineFeature());
    return;
  }
  const coords = lastPlanResult.path_smooth.map((p) => [p[1], p[0]]); // GeoJSON is [lon, lat]
  src.setData({
    type: "Feature",
    geometry: { type: "LineString", coordinates: coords },
    properties: {},
  });
}

document.querySelectorAll("[data-view]").forEach((btn) => {
  btn.addEventListener("click", () => {
    document.querySelectorAll("[data-view]").forEach((b) => b.classList.remove("active"));
    btn.classList.add("active");
    const view = btn.dataset.view;
    if (view === "3d") {
      map2dEl.classList.add("hidden");
      map3dEl.classList.remove("hidden");
      ensureMap3d();
      // Switching the container from display:none to visible leaves the
      // MapLibre canvas blank (fill-extrusion layer in particular) until
      // something nudges it with resize()+triggerRepaint(). Exactly when
      // that "something" needs to happen isn't consistent (rAF timing
      // alone isn't enough), so retry a few times over the following
      // second rather than chase the exact right moment.
      [0, 100, 300, 600, 1000].forEach((ms) =>
        setTimeout(() => {
          map3d.resize();
          map3d.triggerRepaint();
        }, ms)
      );
    } else {
      map3dEl.classList.add("hidden");
      map2dEl.classList.remove("hidden");
      setTimeout(() => map.invalidateSize(), 0);
    }
  });
});

function setStatus(msg, isError) {
  statusEl.textContent = msg || "";
  statusEl.style.color = isError ? "#f66" : "#fc6";
}

function centroidOfRing(ring) {
  const lats = ring.map((p) => p[0]);
  const lons = ring.map((p) => p[1]);
  return {
    lat: (Math.min(...lats) + Math.max(...lats)) / 2,
    lon: (Math.min(...lons) + Math.max(...lons)) / 2,
  };
}

function updatePlanButton() {
  planBtn.disabled = !(startPoint && goalPoint);
}

function setStart(latlng, label) {
  startPoint = latlng;
  if (startMarker) map.removeLayer(startMarker);
  startMarker = L.circleMarker([latlng.lat, latlng.lon], {
    radius: 8,
    color: "#2ecc71",
    fillColor: "#2ecc71",
    fillOpacity: 1,
  }).addTo(map);
  startCoords.textContent = label || `${latlng.lat.toFixed(5)}, ${latlng.lon.toFixed(5)}`;
  updatePlanButton();
}

function setGoal(latlng, label) {
  goalPoint = latlng;
  if (goalMarker) map.removeLayer(goalMarker);
  goalMarker = L.circleMarker([latlng.lat, latlng.lon], {
    radius: 8,
    color: "#e74c3c",
    fillColor: "#e74c3c",
    fillOpacity: 1,
  }).addTo(map);
  goalCoords.textContent = label || `${latlng.lat.toFixed(5)}, ${latlng.lon.toFixed(5)}`;
  updatePlanButton();
}

// Clicking the map: first click sets start, second sets goal, third click
// starts a new start/goal pair.
map.on("click", (e) => {
  const pt = { lat: e.latlng.lat, lon: e.latlng.lng };
  if (!startPoint || (startPoint && goalPoint)) {
    clearRoute();
    setStart(pt);
    startSelect.value = "";
  } else {
    setGoal(pt);
    goalSelect.value = "";
  }
});

startSelect.addEventListener("change", () => {
  const b = allBuildings.find((x) => x.name === startSelect.value);
  if (b) {
    const c = centroidOfRing(b.ring);
    setStart(c, b.name);
  }
});

goalSelect.addEventListener("change", () => {
  const b = allBuildings.find((x) => x.name === goalSelect.value);
  if (b) {
    const c = centroidOfRing(b.ring);
    setGoal(c, b.name);
  }
});

function clearRoute() {
  routeLayer.clearLayers();
  resultEl.classList.add("hidden");
  lastPlanResult = null;
  stopAnimation();
  syncRouteTo3d();
}

clearBtn.addEventListener("click", () => {
  startPoint = null;
  goalPoint = null;
  if (startMarker) map.removeLayer(startMarker);
  if (goalMarker) map.removeLayer(goalMarker);
  startMarker = null;
  goalMarker = null;
  startSelect.value = "";
  goalSelect.value = "";
  startCoords.textContent = "";
  goalCoords.textContent = "";
  updatePlanButton();
  clearRoute();
  setStatus("");
});

async function loadBuildings() {
  const res = await fetch(`${API_BASE}/api/buildings`);
  const data = await res.json();
  allBuildings = data.buildings;
  allBuildings.sort((a, b) => a.name.localeCompare(b.name));

  for (const b of allBuildings) {
    const opt1 = document.createElement("option");
    opt1.value = b.name;
    opt1.textContent = b.name;
    startSelect.appendChild(opt1);

    const opt2 = document.createElement("option");
    opt2.value = b.name;
    opt2.textContent = b.name;
    goalSelect.appendChild(opt2);
  }

  const polyRes = await fetch(`${API_BASE}/api/buildings/all_polygons`);
  const polyData = await polyRes.json();
  for (const ring of polyData.polygons) {
    L.polygon(ring, { color: "#ffffff", weight: 1, fillOpacity: 0.15, fillColor: "#ffffff" }).addTo(
      buildingsLayer
    );
  }
}

planBtn.addEventListener("click", async () => {
  if (!startPoint || !goalPoint) return;
  clearRoute();
  setStatus(`Planning with ${algorithm === "astar" ? "A*" : "RRT*"}…`);
  planBtn.disabled = true;

  try {
    const res = await fetch(`${API_BASE}/api/plan`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        start: { lat: startPoint.lat, lon: startPoint.lon },
        goal: { lat: goalPoint.lat, lon: goalPoint.lon },
        algorithm,
      }),
    });

    if (!res.ok) {
      const err = await res.json();
      throw new Error(err.detail || "Planning failed");
    }

    const data = await res.json();
    lastPlanResult = data;

    const latlngs = data.path_smooth.map((p) => [p[0], p[1]]);
    L.polyline(latlngs, { color: "#00e5ff", weight: 4, opacity: 0.9 }).addTo(routeLayer);

    resLength.textContent = data.length_m.toFixed(0);
    resTime.textContent = data.compute_seconds.toFixed(2);
    resultEl.classList.remove("hidden");
    setStatus("Path found.");
    syncRouteTo3d();
  } catch (e) {
    setStatus(e.message, true);
  } finally {
    planBtn.disabled = false;
  }
});

function stopAnimation() {
  if (animTimer) {
    clearInterval(animTimer);
    animTimer = null;
  }
  if (uavMarker) {
    map.removeLayer(uavMarker);
    uavMarker = null;
  }
  if (uavMarker3d) {
    uavMarker3d.remove();
    uavMarker3d = null;
  }
  if (map3d && map3dLoaded) {
    const src = map3d.getSource("uavtrail3d");
    if (src) src.setData(emptyLineFeature());
  }
}

animateBtn.addEventListener("click", () => {
  if (!lastPlanResult) return;
  stopAnimation();
  ensureMap3d(); // so the 3D view has something to show if the user switches to it mid-flight

  const pts = lastPlanResult.path_smooth;
  const trail = L.polyline([], { color: "#ffe082", weight: 3 }).addTo(routeLayer);

  const uavIcon = L.divIcon({
    className: "leaflet-marker-uav",
    html: '<div style="width:16px;height:16px;border-radius:50%;background:#ff3b30;border:2px solid white;"></div>',
    iconSize: [16, 16],
    iconAnchor: [8, 8],
  });
  uavMarker = L.marker(pts[0], { icon: uavIcon }).addTo(map);

  const uavEl3d = document.createElement("div");
  uavEl3d.style.cssText =
    "width:16px;height:16px;border-radius:50%;background:#ff3b30;border:2px solid white;box-shadow:0 0 4px rgba(255,80,80,0.9);";
  uavMarker3d = new maplibregl.Marker({ element: uavEl3d, anchor: "center" })
    .setLngLat([pts[0][1], pts[0][0]])
    .addTo(map3d);
  const trail3dCoords = [[pts[0][1], pts[0][0]]];

  let i = 0;
  const totalMs = 8000; // full flight animated over 8s regardless of route length
  const stepMs = Math.max(16, totalMs / pts.length);

  animTimer = setInterval(() => {
    if (i >= pts.length) {
      clearInterval(animTimer);
      animTimer = null;
      return;
    }
    uavMarker.setLatLng(pts[i]);
    trail.addLatLng(pts[i]);

    const lngLat = [pts[i][1], pts[i][0]];
    uavMarker3d.setLngLat(lngLat);
    trail3dCoords.push(lngLat);
    if (map3dLoaded) {
      const src = map3d.getSource("uavtrail3d");
      if (src) {
        src.setData({
          type: "Feature",
          geometry: { type: "LineString", coordinates: trail3dCoords },
          properties: {},
        });
      }
    }
    i++;
  }, stepMs);
});

loadBuildings().catch((e) => setStatus("Failed to load buildings: " + e.message, true));
