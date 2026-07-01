%% iitb_digital_twin.m
% IIT Bombay Campus Digital Twin
% Loads campus building footprints from GeoJSON, plans a UAV route with
% RRT* between Hostel 4 and the Aerospace Department, visualizes the
% route on a 3D building model and a satellite basemap, then animates
% the UAV flying the route both over the 3D model and, in real time,
% over the actual satellite map.
%
% Run section by section (Ctrl+Enter in each %% cell) or run the whole
% file top to bottom.

close all
clear
clc

% struct() on a geopolyshape triggers a harmless "implementation details"
% warning on every call in Section 3 (one per building) — silence it.
warning('off','MATLAB:structOnObject')

%% Section 1: Load GeoJSON
geojsonFile = '/home/antidrone/iitb.geojson';
if ~exist(geojsonFile,'file')
    error('GeoJSON file not found: %s', geojsonFile);
end

G = readgeotable(geojsonFile);
fprintf('Loaded %d features from %s\n', height(G), geojsonFile);

% Reference origin for local XY conversion (roughly center of campus)
lat0 = 19.133;
lon0 = 72.915;

figure('Name','IIT Bombay Buildings')
geobasemap satellite
geoplot(G.Shape,'r')
title('IIT Bombay Buildings')

%% Section 2: Find Hostel 4 & Aerospace
idxHostel4 = find(G.name == "Hostel 4");
idxAero    = find(G.name == "Aerospace Department");

if isempty(idxHostel4) || isempty(idxAero)
    error('Could not locate Hostel 4 and/or Aerospace Department in the GeoJSON data.');
end

[latlimH4, lonlimH4] = bounds(G.Shape(idxHostel4));
[latlimAE, lonlimAE] = bounds(G.Shape(idxAero));

startLat = mean(latlimH4);
startLon = mean(lonlimH4);
goalLat  = mean(latlimAE);
goalLon  = mean(lonlimAE);

figure('Name','Hostel 4 -> Aerospace Department (Geo)')
geobasemap satellite
geoplot(G.Shape,'k')
hold on
geoscatter(startLat,startLon,100,'g','filled')
geoscatter(goalLat,goalLon,100,'r','filled')
title('Hostel 4 \rightarrow Aerospace Department')

%% Section 3: Convert GPS -> XY
% Simple equirectangular projection around (lat0, lon0), units in meters.
toXY = @(lat,lon) deal((lon-lon0)*111320*cosd(lat0), (lat-lat0)*111320);

[startX, startY] = toXY(startLat, startLon);
[goalX,  goalY]  = toXY(goalLat,  goalLon);

nBuildings = height(G);
buildingXY = cell(nBuildings,1);
buildingName = strings(nBuildings,1);

for k = 1:nBuildings
    shape = G.Shape(k);
    S = struct(shape);
    lat = S.InternalData.VertexCoordinate1;
    lon = S.InternalData.VertexCoordinate2;
    [x,y] = toXY(lat,lon);
    buildingXY{k} = [x(:) y(:)];
    buildingName(k) = G.name(k);
end

figure('Name','Local XY Frame')
hold on
for k = 1:nBuildings
    xy = buildingXY{k};
    plot(xy(:,1), xy(:,2), 'k')
end
plot(startX, startY, 'go', 'MarkerSize',12, 'LineWidth',3)
plot(goalX,  goalY,  'ro', 'MarkerSize',12, 'LineWidth',3)
axis equal
grid on
xlabel('X (m)')
ylabel('Y (m)')
title('Hostel 4 \rightarrow Aerospace Department (Local XY)')

%% Section 4: Occupancy Map
mapSize = 3000;      % map is mapSize x mapSize meters
mapOffset = mapSize/2; % shift local XY (which is centered on lat0/lon0) into map coords
resolution = 1;       % 1 cell per meter
inflateRadius = 5;    % meters, safety margin around buildings

map = binaryOccupancyMap(mapSize, mapSize, resolution);

for k = 1:nBuildings
    xy = buildingXY{k};
    if size(xy,1) < 3
        continue
    end
    xMap = xy(:,1) + mapOffset;
    yMap = xy(:,2) + mapOffset;

    xmin = max(floor(min(xMap)), 1);
    xmax = min(ceil(max(xMap)), mapSize);
    ymin = max(floor(min(yMap)), 1);
    ymax = min(ceil(max(yMap)), mapSize);

    if xmin > xmax || ymin > ymax
        continue
    end

    [xg,yg] = meshgrid(xmin:xmax, ymin:ymax);
    setOccupancy(map, [xg(:) yg(:)], 1);
end

inflate(map, inflateRadius)

% Building centroids sit inside their own footprint, so they read as
% occupied once rasterized+inflated. Snap start/goal to the nearest free
% cell (spiral search) instead of planning from inside a wall.
startMap = findNearestFreeCell(map, [startX + mapOffset, startY + mapOffset], mapSize);
goalMap  = findNearestFreeCell(map, [goalX  + mapOffset, goalY  + mapOffset], mapSize);

figure('Name','Occupancy Map')
show(map)
hold on
scatter(startMap(1), startMap(2), 150, 'g', 'filled')
scatter(goalMap(1),  goalMap(2),  150, 'r', 'filled')
title('IIT Bombay Occupancy Map')

%% Section 5: RRT*
% Sampling uniformly over the full 3000x3000 map wastes most samples far
% from the start/goal corridor and makes RRT* unreliable within a
% reasonable iteration budget. Restrict the state bounds to a padded box
% around start/goal (the validator still checks against the full map).
boundsMargin = 400;
xlo = max(1, min(startMap(1),goalMap(1)) - boundsMargin);
xhi = min(mapSize, max(startMap(1),goalMap(1)) + boundsMargin);
ylo = max(1, min(startMap(2),goalMap(2)) - boundsMargin);
yhi = min(mapSize, max(startMap(2),goalMap(2)) + boundsMargin);

ss = stateSpaceSE2;
ss.StateBounds = [xlo xhi; ylo yhi; -pi pi];

sv = validatorOccupancyMap(ss);
sv.Map = map;
sv.ValidationDistance = 5;

start = [startMap 0];
goal  = [goalMap 0];

if ~isStateValid(sv, start) || ~isStateValid(sv, goal)
    error('Start or goal state is inside an obstacle. Adjust mapOffset/inflateRadius.');
end

% RRT* is stochastic; retry with a fresh tree (and more iterations) a
% few times in the rare case a run doesn't connect.
maxAttempts = 3;
solnInfo.IsPathFound = false;
attempt = 0;
while ~solnInfo.IsPathFound && attempt < maxAttempts
    attempt = attempt + 1;
    planner = plannerRRTStar(ss, sv);
    planner.MaxIterations = 20000 * attempt;
    planner.MaxConnectionDistance = 150;
    [pathObj, solnInfo] = plan(planner, start, goal);
end

if ~solnInfo.IsPathFound
    error('RRT* failed to find a path after %d attempts.', maxAttempts);
end

figure('Name','RRT* Path')
show(map)
hold on
plot(pathObj.States(:,1), pathObj.States(:,2), 'b-', 'LineWidth', 2)
scatter(start(1), start(2), 100, 'g', 'filled')
scatter(goal(1),  goal(2),  100, 'r', 'filled')
title('RRT* Route: Hostel 4 \rightarrow Aerospace Department')

% Route back in local XY (centered on lat0/lon0)
waypointsXY = pathObj.States(:,1:2) - mapOffset;

%% Section 6: 3D Buildings
altitude = 60; % UAV cruise altitude, meters
waypoints3D = [waypointsXY, altitude*ones(size(waypointsXY,1),1)];

figure('Name','3D Campus Model')
hold on
for k = 1:nBuildings
    xy = buildingXY{k};
    if size(xy,1) < 3
        continue
    end
    x = xy(:,1);
    y = xy(:,2);

    nameStr = lower(char(buildingName(k)));
    if contains(nameStr,'hostel')
        h = 35;
    elseif contains(nameStr,'aerospace')
        h = 30;
    elseif contains(nameStr,'department')
        h = 25;
    elseif contains(nameStr,'library')
        h = 20;
    else
        h = 12;
    end

    % Roof
    fill3(x, y, h*ones(size(x)), [0.7 0.7 0.7], 'EdgeColor','none')
    % Walls
    for i = 1:length(x)-1
        fill3([x(i) x(i+1) x(i+1) x(i)], ...
              [y(i) y(i+1) y(i+1) y(i)], ...
              [0 0 h h], ...
              [0.6 0.6 0.6], 'EdgeColor','none');
    end
end

plot3(waypoints3D(:,1), waypoints3D(:,2), waypoints3D(:,3), 'b-', 'LineWidth', 4)
scatter3(waypoints3D(1,1),   waypoints3D(1,2),   waypoints3D(1,3),   200,'g','filled')
scatter3(waypoints3D(end,1), waypoints3D(end,2), waypoints3D(end,3), 200,'r','filled')

axis equal
grid on
view(3)
camlight
lighting gouraud
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Height (m)')
title('IIT Bombay 3D Digital Twin with UAV Route')

%% Section 7: Satellite Ground
pathLon = waypointsXY(:,1) ./ (111320*cosd(lat0)) + lon0;
pathLat = waypointsXY(:,2) ./ 111320 + lat0;

figure('Name','Route on Satellite Map')
geobasemap satellite
hold on
geoplot(G.Shape,'k')
geoplot(pathLat, pathLon, 'c', 'LineWidth', 4)
geoscatter(startLat, startLon, 120, 'g', 'filled')
geoscatter(goalLat,  goalLon,  120, 'r', 'filled')
title('RRT* Route on IIT Bombay Satellite Map')

%% Section 8: UAV Animation
% RRT* waypoints are sparse and jagged, so waypointTrajectory's spline
% overshoots at each corner (worst at tight turns, e.g. squeezing past
% the hostel cluster). Fix: (1) resample the raw path to a fine, evenly
% spaced polyline so no single corner spans a big gap between points,
% (2) smooth that polyline with a moving average, (3) wherever the
% smoothed point would clip a building, bisect back toward the raw
% (known-safe) point instead of snapping fully back to it — a partial
% correction keeps the curve continuous instead of re-introducing a
% sharp kink.
stepM = 15; % meters between resampled points
d = [0; cumsum(hypot(diff(waypointsXY(:,1)), diff(waypointsXY(:,2))))];
sFine = (0:stepM:d(end))';
if sFine(end) ~= d(end)
    sFine(end+1) = d(end);
end
fineXY = [interp1(d, waypointsXY(:,1), sFine), interp1(d, waypointsXY(:,2), sFine)];

smoothWindowM = 60;
winSamples = max(3, round(smoothWindowM/stepM));
if mod(winSamples,2) == 0
    winSamples = winSamples + 1;
end
smoothXY = fineXY;
smoothXY(:,1) = smoothdata(fineXY(:,1), 'movmean', winSamples);
smoothXY(:,2) = smoothdata(fineXY(:,2), 'movmean', winSamples);
smoothXY(1,:)   = fineXY(1,:);
smoothXY(end,:) = fineXY(end,:);

for i = 1:size(smoothXY,1)
    pRaw = fineXY(i,:);
    pSm  = smoothXY(i,:);
    pMap = pSm + mapOffset;
    isSafe = ~(any(pMap < 1) || any(pMap > mapSize)) && getOccupancy(map, pMap) == 0;
    if isSafe
        continue
    end
    lo = 0; hi = 1; % blend factor toward the raw point (1 = fully raw)
    for iter = 1:20
        mid = (lo+hi)/2;
        cand = (1-mid)*pSm + mid*pRaw;
        candMap = cand + mapOffset;
        candSafe = ~(any(candMap < 1) || any(candMap > mapSize)) && getOccupancy(map, candMap) == 0;
        if candSafe
            hi = mid;
        else
            lo = mid;
        end
    end
    smoothXY(i,:) = (1-hi)*pSm + hi*pRaw;
end

waypointsXYsmooth = smoothXY;
waypoints3Dsmooth = [waypointsXYsmooth, altitude*ones(size(waypointsXYsmooth,1),1)];

toa = linspace(0, 120, size(waypoints3Dsmooth,1)); % 120 s total flight time
% Default SampleRate is 100 Hz -> 12000 drawnow calls; 10 Hz is plenty
% smooth for a marker animation and far faster to render.
traj = waypointTrajectory(waypoints3Dsmooth, TimeOfArrival = toa, SampleRate = 10);

figure('Name','UAV Animation')
hold on
for k = 1:nBuildings
    xy = buildingXY{k};
    if size(xy,1) < 3
        continue
    end
    patch(xy(:,1), xy(:,2), [0.4 0.4 0.4], 'EdgeColor','none');
end
plot3(waypoints3D(:,1), waypoints3D(:,2), waypoints3D(:,3), 'k--')
axis equal
grid on
view(3)
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Altitude (m)')
title('UAV Flight Animation: Hostel 4 \rightarrow Aerospace Department')

uavMarker = scatter3(waypoints3Dsmooth(1,1), waypoints3Dsmooth(1,2), waypoints3Dsmooth(1,3), 250, 'r', 'filled');

reset(traj)
while ~isDone(traj)
    [pos, ~, ~, ~] = traj();
    set(uavMarker, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));
    drawnow
end

%% Section 9: Satellite UAV Animation
% Combines Section 7 (real satellite basemap) with Section 8 (real-time
% marker animation): the smoothed route is drawn on satellite imagery
% and the UAV marker moves along it in real time, tracing the flown
% path as it advances.
pathLonSmooth = waypointsXYsmooth(:,1) ./ (111320*cosd(lat0)) + lon0;
pathLatSmooth = waypointsXYsmooth(:,2) ./ 111320 + lat0;

figure('Name','Satellite UAV Animation')
geobasemap satellite
hold on
geoplot(G.Shape,'k')
geoplot(pathLatSmooth, pathLonSmooth, 'c--', 'LineWidth', 2)
geoscatter(startLat, startLon, 120, 'g', 'filled')
geoscatter(goalLat,  goalLon,  120, 'r', 'filled')
title('Real-Time UAV Flight on IIT Bombay Satellite Map')

uavGeoMarker = geoscatter(pathLatSmooth(1), pathLonSmooth(1), 250, 'r', 'filled');
trailLat = pathLatSmooth(1);
trailLon = pathLonSmooth(1);
trailPlot = geoplot(trailLat, trailLon, 'b-', 'LineWidth', 3);

reset(traj)
while ~isDone(traj)
    [pos, ~, ~, ~] = traj();
    latPos = pos(2)/111320 + lat0;
    lonPos = pos(1)/(111320*cosd(lat0)) + lon0;
    trailLat(end+1) = latPos; %#ok<AGROW>
    trailLon(end+1) = lonPos; %#ok<AGROW>
    set(uavGeoMarker, 'LatitudeData', latPos, 'LongitudeData', lonPos);
    set(trailPlot, 'LatitudeData', trailLat, 'LongitudeData', trailLon);
    drawnow
end

%% Section 10: Save Results
outFile = fullfile(pwd, 'iitb_digital_twin_results.mat');
save(outFile, 'G', 'lat0', 'lon0', 'map', 'pathObj', 'solnInfo', ...
    'start', 'goal', 'waypointsXY', 'waypoints3D', 'waypoints3Dsmooth', ...
    'startLat', 'startLon', 'goalLat', 'goalLon', 'buildingXY', 'buildingName');

fprintf('Results saved to %s\n', outFile);

%% Local Functions
function pFree = findNearestFreeCell(map, p, mapSize)
% Spiral outward from p (map coordinates) until an unoccupied, in-bounds
% cell is found. Needed because building centroids fall inside their own
% (inflated) footprint.
    if getOccupancy(map, p) == 0
        pFree = p;
        return
    end
    for r = 1:2:200
        for dx = -r:r
            for dy = -r:r
                if abs(dx) ~= r && abs(dy) ~= r
                    continue % only test the ring perimeter
                end
                cand = p + [dx dy];
                if any(cand < 1) || any(cand > mapSize)
                    continue
                end
                if getOccupancy(map, cand) == 0
                    pFree = cand;
                    return
                end
            end
        end
    end
    error('findNearestFreeCell:notFound', ...
        'No free cell found within search radius of [%g %g].', p(1), p(2));
end
