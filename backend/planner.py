"""Path planning on the occupancy grid: A* (grid search) and RRT* (sampling
based), plus the same resample -> smooth -> bisect-back-if-unsafe path
post-processing used in reference/iitb_digital_twin_matlab.m (Section 8) to
avoid corner overshoot when the route is later animated.
"""
import heapq
import itertools
import math
import random


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


# ---------------------------------------------------------------------------
# A* on a downsampled occupancy grid
# ---------------------------------------------------------------------------
class CoarseGrid:
    def __init__(self, campus, factor=4):
        self.campus = campus
        self.factor = factor
        n = campus.map_size
        self.size = n // factor
        fine = campus.grid
        trimmed = fine[: self.size * factor, : self.size * factor]
        reshaped = trimmed.reshape(self.size, factor, self.size, factor)
        self.coarse = reshaped.any(axis=(1, 3))

    def fine_to_coarse(self, x, y):
        return int(x // self.factor), int(y // self.factor)

    def coarse_to_fine(self, cx, cy):
        return (cx + 0.5) * self.factor, (cy + 0.5) * self.factor

    def is_free(self, cx, cy):
        if cx < 0 or cx >= self.size or cy < 0 or cy >= self.size:
            return False
        return not self.coarse[cy, cx]


_NEIGHBORS_8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def astar(campus, start_xy, goal_xy, factor=1):
    # factor>1 (coarsening) can close the narrow gaps between closely
    # spaced buildings and make an otherwise-connected route unreachable,
    # so plan on the full-resolution (1m) grid by default. It costs ~2s
    # instead of ~20ms but is what's actually needed for correctness here.
    cg = CoarseGrid(campus, factor=factor)
    start = cg.fine_to_coarse(*start_xy)
    goal = cg.fine_to_coarse(*goal_xy)

    if not cg.is_free(*start) or not cg.is_free(*goal):
        return None

    counter = itertools.count()
    open_heap = [(_dist(start, goal), next(counter), start)]
    came_from = {}
    g_score = {start: 0.0}
    closed = set()

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)

        if current == goal:
            path = [current]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            path.reverse()
            return [cg.coarse_to_fine(*p) for p in path]

        for dx, dy in _NEIGHBORS_8:
            nxt = (current[0] + dx, current[1] + dy)
            if nxt in closed or not cg.is_free(*nxt):
                continue
            step_cost = math.hypot(dx, dy)
            tentative = g_score[current] + step_cost
            if tentative < g_score.get(nxt, math.inf):
                g_score[nxt] = tentative
                came_from[nxt] = current
                heapq.heappush(open_heap, (tentative + _dist(nxt, goal), next(counter), nxt))

    return None


# ---------------------------------------------------------------------------
# RRT* on the fine occupancy grid
# ---------------------------------------------------------------------------
class _Node:
    __slots__ = ("x", "y", "parent", "cost")

    def __init__(self, x, y, parent=None, cost=0.0):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = cost


def _collision_free(campus, p1, p2, step=5.0):
    d = _dist(p1, p2)
    n = max(1, int(d / step))
    for i in range(n + 1):
        t = i / n
        x = p1[0] + (p2[0] - p1[0]) * t
        y = p1[1] + (p2[1] - p1[1]) * t
        if not campus.is_free(x, y):
            return False
    return True


class _SpatialIndex:
    """Uniform-grid bucket index for approximate nearest-neighbor and
    radius queries over RRT* tree nodes. Plain min()-over-all-nodes scans
    are O(n) per iteration, which made RRT* both slow and unreliable
    within a web-request time budget (it couldn't afford enough
    iterations to reliably find the narrow gaps between buildings).
    """

    def __init__(self, cell_size):
        self.cell_size = cell_size
        self.cells = {}

    def _key(self, x, y):
        return (int(x // self.cell_size), int(y // self.cell_size))

    def insert(self, node):
        self.cells.setdefault(self._key(node.x, node.y), []).append(node)

    def nearest(self, pt):
        cx, cy = self._key(*pt)
        best, best_d = None, math.inf
        radius = 0
        found_ring = None
        while True:
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    for n in self.cells.get((cx + dx, cy + dy), []):
                        d = _dist((n.x, n.y), pt)
                        if d < best_d:
                            best_d, best = d, n
            if best is not None and found_ring is None:
                found_ring = radius
            if found_ring is not None and radius >= found_ring + 1:
                break
            radius += 1
            if radius > 5000:
                break
        return best

    def near(self, pt, radius_m):
        cx, cy = self._key(*pt)
        r_cells = int(radius_m // self.cell_size) + 1
        result = []
        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                for n in self.cells.get((cx + dx, cy + dy), []):
                    if _dist((n.x, n.y), pt) <= radius_m:
                        result.append(n)
        return result


def rrt_star(
    campus,
    start_xy,
    goal_xy,
    bounds_margin=400,
    max_iterations=25000,
    step_size=60.0,
    search_radius=100.0,
    goal_tol=30.0,
    goal_sample_rate=0.1,
):
    if not campus.is_free(*start_xy) or not campus.is_free(*goal_xy):
        return None

    xlo = max(0.0, min(start_xy[0], goal_xy[0]) - bounds_margin)
    xhi = min(float(campus.map_size), max(start_xy[0], goal_xy[0]) + bounds_margin)
    ylo = max(0.0, min(start_xy[1], goal_xy[1]) - bounds_margin)
    yhi = min(float(campus.map_size), max(start_xy[1], goal_xy[1]) + bounds_margin)

    start_node = _Node(start_xy[0], start_xy[1])
    index = _SpatialIndex(cell_size=search_radius)
    index.insert(start_node)
    best_goal_node = None

    for _ in range(max_iterations):
        if random.random() < goal_sample_rate:
            rnd = goal_xy
        else:
            rnd = (random.uniform(xlo, xhi), random.uniform(ylo, yhi))

        nearest_node = index.nearest(rnd)
        d = _dist((nearest_node.x, nearest_node.y), rnd)
        if d <= step_size:
            new_pt = rnd
        else:
            theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)
            new_pt = (
                nearest_node.x + step_size * math.cos(theta),
                nearest_node.y + step_size * math.sin(theta),
            )

        if not campus.is_free(*new_pt):
            continue
        if not _collision_free(campus, (nearest_node.x, nearest_node.y), new_pt):
            continue

        near_nodes = index.near(new_pt, search_radius)

        best_parent = nearest_node
        best_cost = nearest_node.cost + _dist((nearest_node.x, nearest_node.y), new_pt)
        for n in near_nodes:
            c = n.cost + _dist((n.x, n.y), new_pt)
            if c < best_cost and _collision_free(campus, (n.x, n.y), new_pt):
                best_cost = c
                best_parent = n

        new_node = _Node(new_pt[0], new_pt[1], best_parent, best_cost)
        index.insert(new_node)

        for n in near_nodes:
            c = new_node.cost + _dist((new_node.x, new_node.y), (n.x, n.y))
            if c < n.cost and _collision_free(campus, (new_node.x, new_node.y), (n.x, n.y)):
                n.parent = new_node
                n.cost = c

        if _dist(new_pt, goal_xy) <= goal_tol and _collision_free(campus, new_pt, goal_xy):
            cost = new_node.cost + _dist(new_pt, goal_xy)
            if best_goal_node is None or cost < best_goal_node.cost:
                best_goal_node = _Node(goal_xy[0], goal_xy[1], new_node, cost)

    if best_goal_node is None:
        return None

    path = []
    node = best_goal_node
    while node is not None:
        path.append((node.x, node.y))
        node = node.parent
    path.reverse()
    return path


# ---------------------------------------------------------------------------
# Shared post-processing: resample to fixed spacing, movmean-smooth, and
# bisect back toward the raw point wherever smoothing would clip a building.
# Same technique validated in the MATLAB script's Section 8 fix.
# ---------------------------------------------------------------------------
def smooth_path(campus, path_xy, step_m=15.0, smooth_window_m=60.0):
    if len(path_xy) < 3:
        return list(path_xy)

    # Cumulative arc length.
    dists = [0.0]
    for i in range(1, len(path_xy)):
        dists.append(dists[-1] + _dist(path_xy[i - 1], path_xy[i]))
    total_len = dists[-1]
    if total_len == 0:
        return list(path_xy)

    s_fine = list(_frange(0, total_len, step_m))
    if s_fine[-1] != total_len:
        s_fine.append(total_len)

    fine_xy = [_interp_along(path_xy, dists, s) for s in s_fine]

    win_samples = max(3, round(smooth_window_m / step_m))
    if win_samples % 2 == 0:
        win_samples += 1
    half = win_samples // 2

    smooth_xy = []
    for i in range(len(fine_xy)):
        lo = max(0, i - half)
        hi = min(len(fine_xy), i + half + 1)
        window = fine_xy[lo:hi]
        sx = sum(p[0] for p in window) / len(window)
        sy = sum(p[1] for p in window) / len(window)
        smooth_xy.append((sx, sy))
    smooth_xy[0] = fine_xy[0]
    smooth_xy[-1] = fine_xy[-1]

    result = []
    for raw, sm in zip(fine_xy, smooth_xy):
        if campus.is_free(*sm):
            result.append(sm)
            continue
        lo, hi = 0.0, 1.0
        for _ in range(20):
            mid = (lo + hi) / 2
            cand = (sm[0] + (raw[0] - sm[0]) * mid, sm[1] + (raw[1] - sm[1]) * mid)
            if campus.is_free(*cand):
                hi = mid
            else:
                lo = mid
        blend = hi
        result.append((sm[0] + (raw[0] - sm[0]) * blend, sm[1] + (raw[1] - sm[1]) * blend))

    return result


def _frange(start, stop, step):
    x = start
    while x < stop:
        yield x
        x += step


def _interp_along(path_xy, dists, s):
    for i in range(1, len(dists)):
        if dists[i] >= s:
            t = 0 if dists[i] == dists[i - 1] else (s - dists[i - 1]) / (dists[i] - dists[i - 1])
            x = path_xy[i - 1][0] + (path_xy[i][0] - path_xy[i - 1][0]) * t
            y = path_xy[i - 1][1] + (path_xy[i][1] - path_xy[i - 1][1]) * t
            return (x, y)
    return path_xy[-1]
