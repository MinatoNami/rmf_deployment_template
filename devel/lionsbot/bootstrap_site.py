#!/usr/bin/env python3
"""Generate an Open-RMF site from the LionsBot cloud API.

Pulls a robot's map, markers and zones from api.lionsbot.io and writes the four
files the fleet adapter and rmf-core need:

    configs/<site>/config_<fleet>.yaml   fleet + robot config
    maps/<site>/<n>.yaml                 nav graph
    maps/<site>/dock_summary.yaml        cleaning zone footprints
    maps/<site>/rmf_<site>.building.yaml floor plan for the dashboard
    maps/<site>/rmf_<site>.png           the robot's occupancy grid

Coordinates. The robot's grid PNG is used as the RMF floor plan, so the two
share a pixel frame and the map transform reduces to a single scale:

    rmf_x_metres =  pixel_x * scale
    rmf_y_metres = -pixel_y * scale

That is why the emitted transform has rotation 0 and no translation. It is
correct by construction, not a placeholder.

Scale. Derived from cleaning job reports: a zone's plannedPath is in map pixels
and the same zone's `distance` from /zones is in metres, so the ratio gives
metres-per-pixel. Needs at least one completed job. Override with --scale.

Lanes. Cannot be derived -- LionsBot has no lane concept. A placeholder star
topology is emitted so the stack starts and you can prove the pipe end to end.
REPLACE IT before running real traffic.

Usage:
    python3 bootstrap_site.py --list-robots
    python3 bootstrap_site.py --robot R5SCR-DV30019 --site mysite --fleet r5
"""

from __future__ import annotations

import argparse
import getpass
import json
import math
import os
import statistics
import struct
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

try:
    import yaml
except ImportError:
    sys.exit('This script needs PyYAML:  pip install pyyaml')

API_ROOT = 'https://api.lionsbot.io/openapi/v2'
TIMEOUT = 30
YEAR_MS = 365 * 24 * 3600 * 1000


# ---------------------------------------------------------------------------
# API client
# ---------------------------------------------------------------------------

class LionsbotApi:
    def __init__(self, prefix: str = API_ROOT):
        self.prefix = prefix.rstrip('/')
        self.token: str | None = None

    def login(self, email: str, password: str) -> None:
        body = json.dumps({
            'email': email,
            'password': password,
            'applicationName': 'DASHBOARD',
        }).encode()
        req = urllib.request.Request(
            f'{self.prefix}/edge/auth/login',
            data=body,
            headers={'Content-Type': 'application/json', 'Accept': '*/*'},
            method='POST')
        try:
            with urllib.request.urlopen(req, timeout=TIMEOUT) as response:
                data = json.load(response)
        except urllib.error.HTTPError as err:
            sys.exit(f'Login failed ({err.code}). Check the email and password.')
        except urllib.error.URLError as err:
            sys.exit(f'Cannot reach {self.prefix}: {err.reason}')
        self.token = data.get('token')
        if not self.token:
            sys.exit('Login response contained no token.')
        expiry = data.get('tokenExpiryIsoUtcTime', 'unknown')
        print(f'  logged in, token expires {expiry}')

    def get(self, path: str, params: dict | None = None, quiet: bool = False):
        url = f'{self.prefix}{path}'
        if params:
            url = f'{url}?{urllib.parse.urlencode(params)}'
        req = urllib.request.Request(
            url, headers={'Accept': 'application/json',
                          'Authorization': f'Bearer {self.token}'})
        try:
            with urllib.request.urlopen(req, timeout=TIMEOUT) as response:
                return json.load(response)
        except urllib.error.HTTPError as err:
            detail = ''
            try:
                detail = f" -- {json.load(err).get('errorMessage', '')}"
            except Exception:
                pass
            if not quiet:
                print(f'  ! GET {path} returned {err.code}{detail}',
                      file=sys.stderr)
            return None
        except urllib.error.URLError as err:
            if not quiet:
                print(f'  ! GET {path} failed: {err.reason}', file=sys.stderr)
            return None

    def download(self, url: str, target: Path, attempts: int = 3) -> bool:
        for attempt in range(1, attempts + 1):
            if self._download_once(url, target, final=attempt == attempts):
                return True
            if attempt < attempts:
                time.sleep(1.0)
        return False

    def _download_once(self, url: str, target: Path, final: bool) -> bool:
        try:
            api_host = urllib.parse.urlparse(self.prefix).hostname or ''
            host = urllib.parse.urlparse(url).hostname or ''
            # Presigned links (S3) carry their own signature and 400 if an
            # Authorization header is also present.
            headers = ({'Authorization': f'Bearer {self.token}'}
                       if host == api_host else {})
            req = urllib.request.Request(url, headers=headers)
            with urllib.request.urlopen(req, timeout=TIMEOUT) as response:
                target.write_bytes(response.read())
            return True
        except Exception as err:  # noqa: BLE001 - best effort, reported below
            if final:
                print(f'  ! could not download the grid image: {err}',
                      file=sys.stderr)
            return False


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def parse_marker_coordinates(raw) -> list[tuple[float, float]]:
    """LionsBot marker coordinates: "727 382 727 445 953 450" -> [(x, y), ...]."""
    if raw is None:
        return []
    if isinstance(raw, (list, tuple)):
        flat = [float(v) for v in raw]
    else:
        flat = [float(tok) for tok in str(raw).replace(',', ' ').split() if tok]
    return [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]


def parse_path_segments(raw) -> list[list[tuple[float, float]]]:
    """Parse a job report path into segments of (x, y) pixel points.

    Observed shape is a list of segments, each a list of [x, y] pairs:
        [[[464, 288], [465, 288], ...]]
    Flat [x1, y1, x2, y2, ...] and the space-separated string form are also
    accepted, since the API docs type this only as `array[number]`.
    """
    if not raw:
        return []
    if isinstance(raw, str):
        points = parse_marker_coordinates(raw)
        return [points] if points else []
    if not isinstance(raw, (list, tuple)):
        return []

    first = next((item for item in raw if item is not None), None)

    # [[x, y], ...] - one bare segment of pairs
    if isinstance(first, (list, tuple)) and first and not isinstance(
            first[0], (list, tuple)):
        return [[(float(a), float(b)) for a, b in raw if a is not None]]

    # [[[x, y], ...], ...] - segments of pairs
    if isinstance(first, (list, tuple)):
        segments = []
        for segment in raw:
            points = [(float(a), float(b)) for a, b in segment]
            if len(points) >= 2:
                segments.append(points)
        return segments

    # flat [x1, y1, x2, y2, ...]
    flat = [float(v) for v in raw]
    points = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]
    return [points] if len(points) >= 2 else []


def flatten_segments(segments) -> list[tuple[float, float]]:
    return [point for segment in segments for point in segment]


def segments_length(segments) -> float:
    """Total length, measured within segments so gaps between them do not count."""
    return sum(polyline_length(segment) for segment in segments)


def centroid(points: list[tuple[float, float]]) -> tuple[float, float] | None:
    if not points:
        return None
    return (sum(p[0] for p in points) / len(points),
            sum(p[1] for p in points) / len(points))


def polyline_length(points: list[tuple[float, float]]) -> float:
    return sum(math.dist(points[i], points[i + 1])
               for i in range(len(points) - 1))


def convex_hull(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Andrew's monotone chain. Used to turn a coverage path into a footprint."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def half(seq):
        out: list[tuple[float, float]] = []
        for p in seq:
            while len(out) >= 2:
                (ox, oy), (px, py) = out[-2], out[-1]
                if (px - ox) * (p[1] - oy) - (py - oy) * (p[0] - ox) > 0:
                    break
                out.pop()
            out.append(p)
        return out[:-1]

    return half(pts) + half(reversed(pts))


def to_rmf(point: tuple[float, float], scale: float) -> tuple[float, float]:
    """Map pixels -> RMF metres. The y axis flips; see the module docstring."""
    return (point[0] * scale, -point[1] * scale)


# ---------------------------------------------------------------------------
# Data gathering
# ---------------------------------------------------------------------------

def choose_robot(api: LionsbotApi, encoding_id: str | None):
    robots: list[dict] = []
    page = 0
    while True:
        batch = api.get('/robot', {'page': page, 'size': 100})
        if not batch:
            break
        items = batch.get('content', batch) if isinstance(batch, dict) else batch
        if not items:
            break
        robots.extend(items)
        if len(items) < 100:
            break
        page += 1

    if not robots:
        sys.exit('No robots visible on this account.')

    if encoding_id is None:
        print(f'\n{len(robots)} robot(s) on this account:\n')
        for robot in robots:
            print(f"  {robot.get('robotEncodingId', '?'):24s} "
                  f"{robot.get('robotType', '?'):12s} "
                  f"worksite={robot.get('worksiteName', '?')} "
                  f"online={robot.get('isOnline')}")
        print('\nRe-run with --robot <robotEncodingId>.')
        sys.exit(0)

    for robot in robots:
        if robot.get('robotEncodingId') == encoding_id:
            return robot
    sys.exit(f'No robot with encoding ID {encoding_id!r} on this account.')


def choose_map(api: LionsbotApi, robot_uuid: str,
               map_name: str | None, map_level: str | None):
    payload = api.get(f'/robot/{robot_uuid}/map', {'requireMarkers': 'true'})
    if not payload:
        sys.exit('Could not read the robot map list.')

    maps = payload.get('workSiteMaps') or []
    if not maps:
        sys.exit('This robot has no maps. Create one on the touchscreen first.')

    current = payload.get('currentMapId')
    candidates = maps
    if map_name:
        candidates = [m for m in candidates if m.get('name') == map_name]
    if map_level:
        candidates = [m for m in candidates
                      if str(m.get('level', '')).strip() == map_level]

    if not candidates:
        print('\nAvailable maps:\n')
        for m in maps:
            print(f"  name={m.get('name')!r} level={m.get('level')!r} "
                  f"synced={m.get('synced')} draft={m.get('draft')}")
        sys.exit('\nNo map matched --map-name / --map-level.')

    chosen = next((m for m in candidates if m.get('id') == current), candidates[0])

    if not chosen.get('synced'):
        print('  ! this map is not synced to the robot yet', file=sys.stderr)
    if chosen.get('draft'):
        print('  ! this map is still a draft', file=sys.stderr)
    if chosen.get('equalizersSyncStatus') not in (None, 'SUCCESS'):
        print(f"  ! equalizers are {chosen.get('equalizersSyncStatus')}, "
              f'cleaning may be rejected', file=sys.stderr)
    return chosen


def collect_zone_paths(api: LionsbotApi, robot_uuid: str,
                       days: int) -> dict[str, list[tuple[float, float]]]:
    """zone name -> coverage path in map pixels, from recent job reports."""
    end = int(time.time() * 1000)
    start = max(0, end - min(days, 365) * 24 * 3600 * 1000)

    jobs = api.get(f'/report/job/by-robot-id/{robot_uuid}', {
        'startTime': start,
        'endTime': end,
        'includeManualReport': 'true',
    })
    if not jobs:
        return {}
    if isinstance(jobs, dict):
        jobs = jobs.get('content') or jobs.get('jobs') or []

    paths: dict[str, list[tuple[float, float]]] = {}
    for job in jobs:
        job_id = job.get('id')
        if not job_id:
            continue
        detail = api.get(f'/report/job/{job_id}', quiet=True)
        if not detail:
            continue
        for zone in detail.get('zones') or []:
            name = zone.get('zoneName')
            # plannedPath is often empty while actualPath carries the run.
            raw = zone.get('plannedPath') or zone.get('actualPath')
            if not name or not raw or name in paths:
                continue
            segments = parse_path_segments(raw)
            if segments:
                paths[name] = segments
    return paths


def derive_scale(zones: list[dict], zone_paths: dict
                 ) -> tuple[float | None, list[str]]:
    """metres-per-pixel = zone distance (m) / coverage path length (px)."""
    notes: list[str] = []
    estimates: list[float] = []
    for zone in zones:
        name = zone.get('name')
        metres = zone.get('distance')
        segments = zone_paths.get(name)
        if not name or not metres or not segments:
            continue
        pixels = segments_length(segments)
        if pixels <= 0:
            continue
        estimate = metres / pixels
        estimates.append(estimate)
        notes.append(f'    {name}: {metres:.2f} m / {pixels:.1f} px '
                     f'= {estimate:.6f}')
    if not estimates:
        return None, notes
    return statistics.median(estimates), notes


# ---------------------------------------------------------------------------
# File builders
# ---------------------------------------------------------------------------

def build_waypoints(markers: list[dict], home_poi: dict | None,
                    zones: list[dict], zone_paths: dict,
                    scale: float) -> list[dict]:
    """One entry per waypoint: name, pixel position, RMF position, role."""
    waypoints: list[dict] = []
    seen: set[str] = set()

    def add(name: str, pixel: tuple[float, float], role: str) -> None:
        if not name or name in seen:
            return
        seen.add(name)
        waypoints.append({
            'name': name,
            'pixel': pixel,
            'rmf': to_rmf(pixel, scale),
            'role': role,
        })

    for marker in markers:
        point = centroid(parse_marker_coordinates(marker.get('coordinates')))
        if point is None:
            continue
        marker_type = str(marker.get('type', '')).upper()
        if 'DOCK' in marker_type:
            role = 'dock'
        elif 'LOC' in marker_type:
            role = 'locpoint'
        elif 'HOME' in marker_type:
            role = 'home'
        else:
            role = 'poi'
        add(marker.get('name'), point, role)

    if home_poi:
        point = centroid(parse_marker_coordinates(home_poi.get('coordinates')))
        if point:
            add(home_poi.get('name') or 'home', point, 'home')

    cleanable = [z for z in zones
                 if str(z.get('type', '')).lower() in ('clean', 'cleaning', '')]
    skipped = [z.get('name') for z in zones if z not in cleanable]
    if skipped:
        print(f'  skipping {len(skipped)} non-cleaning zone(s): '
              f"{', '.join(str(n) for n in skipped)}")

    unplaced = []
    for zone in cleanable:
        name = zone.get('name')
        if not name or name in seen:
            continue
        point = centroid(flatten_segments(zone_paths.get(name, [])))
        if point is None:
            unplaced.append(name)
            continue
        add(name, point, 'zone')

    if unplaced:
        # No coverage path and no polygon, so the API gives no position for
        # these. Lay them in a ring around the known markers so the names
        # exist and can be dragged into place in rmf_site.
        anchor = centroid([wp['pixel'] for wp in waypoints]) or (0.0, 0.0)
        radius = 3.0 / scale if scale else 60.0
        for index, name in enumerate(unplaced):
            # Offset the ring by an eighth turn. Starting at 0 would place a
            # lone zone at the same y as the anchor, giving the nav graph a
            # zero-height bounding box, which the dashboard cannot frame.
            theta = 2 * math.pi * index / len(unplaced) + math.pi / 4
            add(name,
                (anchor[0] + radius * math.cos(theta),
                 anchor[1] + radius * math.sin(theta)),
                'zone')
        print(f'  ! {len(unplaced)} zone(s) have no coverage path; placed in a '
              f'ring around the markers as PLACEHOLDERS: '
              f"{', '.join(unplaced)}", file=sys.stderr)

    return waypoints


def pick_charger(waypoints: list[dict]) -> dict | None:
    """A real dock if one exists, else the home point, else a loc point.

    RMF needs a charger waypoint and the nav graph vertex must be flagged
    is_charger, or the fleet comes up with a charger it cannot resolve.
    """
    for role in ('dock', 'home', 'locpoint'):
        match = next((wp for wp in waypoints if wp['role'] == role), None)
        if match is not None:
            if role != 'dock':
                print(f"  ! no dock marker on this map; using "
                      f"{match['name']!r} as the charger waypoint",
                      file=sys.stderr)
            return match
    return None


def build_nav_graph(site: str, level: str, waypoints: list[dict]) -> dict:
    vertices = []
    for wp in waypoints:
        props: dict = {'name': wp['name']}
        if wp['role'] == 'dock':
            props.update({'is_charger': True,
                          'dock_name': wp['name'],
                          'robot_dock_name': wp['name']})
        elif wp.get('is_charger'):
            props['is_charger'] = True
        if wp['role'] == 'zone':
            props.update({'is_cleaning_zone': True,
                          'robot_zone_name': wp['name']})
        vertices.append([wp['rmf'][0], wp['rmf'][1], props])

    hub = next((i for i, wp in enumerate(waypoints) if wp['role'] == 'home'), None)
    if hub is None:
        hub = next((i for i, wp in enumerate(waypoints)
                    if wp['role'] == 'locpoint'), None)
    if hub is None:
        hub = next((i for i, wp in enumerate(waypoints)
                    if wp['role'] != 'dock'), 0)

    lanes = []
    for i, wp in enumerate(waypoints):
        if i == hub:
            continue
        if wp['role'] == 'dock':
            lanes.append([hub, i, {'dock_name': wp['name']}])
            lanes.append([i, hub, {'undock_name': wp['name']}])
        else:
            lanes.append([hub, i, {}])
            lanes.append([i, hub, {}])

    return {
        'building_name': f'rmf_{site}',
        'doors': {},
        'levels': {level: {'lanes': lanes, 'vertices': vertices}},
        'lifts': {},
    }


def build_building(site: str, level: str, waypoints: list[dict],
                   scale: float, png_name: str,
                   grid_size: tuple[int, int] | None = None) -> dict:
    vertices = [[wp['pixel'][0], wp['pixel'][1], 0, wp['name']]
                for wp in waypoints]

    lane_params = {
        'bidirectional': [4, True],
        'demo_mock_floor_name': [1, ''],
        'demo_mock_lift_name': [1, ''],
        'graph_idx': [2, 0],
        'mutex': [1, ''],
        'orientation': [1, ''],
        'speed_limit': [3, 0],
    }
    lanes = [[0, i, dict(lane_params)] for i in range(1, len(vertices))]

    # A measurement line pins the scale when the file is opened in rmf_site or
    # traffic_editor. Two synthetic vertices keep it independent of waypoints.
    base_index = len(vertices)
    vertices.append([0.0, 0.0, 0, ''])
    vertices.append([100.0, 0.0, 0, ''])
    measurements = [[base_index, base_index + 1,
                     {'distance': [3, round(100.0 * scale, 6)]}]]

    # A building with no walls gives the dashboard nothing to frame the view
    # with, and it renders an empty canvas. Trace the floor plan's extent so
    # there is always a wall graph.
    walls = []
    if grid_size:
        width, height = grid_size
        corner = len(vertices)
        for x, y in ((0, 0), (width, 0), (width, height), (0, height)):
            vertices.append([float(x), float(y), 0, ''])
        def wall_params():
            # Built fresh per wall: sharing the inner lists makes PyYAML emit
            # anchors and aliases, which is needless noise in the output.
            return {
                'alpha': [3, 1], 'texture_height': [3, 2.5],
                'texture_name': [1, 'default'], 'texture_scale': [3, 1],
                'texture_width': [3, 1],
            }
        for offset in range(4):
            walls.append([corner + offset, corner + (offset + 1) % 4,
                          wall_params()])

    return {
        'coordinate_system': 'reference_image',
        'crowd_sim': {
            'agent_groups': [], 'agent_profiles': [], 'enable': 0,
            'goal_sets': [], 'model_types': [], 'states': [],
            'transitions': [], 'update_time_step': 0.1,
        },
        'graphs': {},
        'levels': {
            level: {
                'drawing': {'filename': png_name},
                'elevation': 0,
                'flattened_x_offset': 0,
                'flattened_y_offset': 0,
                'floors': [],
                'lanes': lanes,
                'measurements': measurements,
                'vertices': vertices,
                'walls': walls,
            }
        },
        'lifts': {},
        'name': f'rmf_{site}',
    }


def build_dock_summary(fleet: str, level: str, waypoints: list[dict],
                       zone_paths: dict, scale: float) -> dict:
    entries: dict = {}
    pathless = []
    for wp in waypoints:
        if wp['role'] != 'zone':
            continue
        hull = convex_hull(flatten_segments(zone_paths.get(wp['name'], [])))
        if hull:
            path = [[*to_rmf(point, scale), 0.0] for point in hull]
        else:
            # No coverage path known. Reserve the waypoint itself so the zone
            # is still advertised and dispatchable; the reservation is just a
            # point rather than the real footprint.
            path = [[wp['rmf'][0], wp['rmf'][1], 0.0]]
            pathless.append(wp['name'])
        entries[wp['name']] = {
            'finish_waypoint': wp['name'],
            'level_name': level,
            'robot_zone_name': wp['name'],
            'path': path,
        }
    if pathless:
        print(f'  ! {len(pathless)} zone(s) reserved as a single point rather '
              f"than a footprint: {', '.join(pathless)}", file=sys.stderr)
    return {fleet: entries}


def build_config(fleet: str, level: str, robot: dict, map_data: dict,
                 waypoints: list[dict], scale: float,
                 angle_to_radians, vendor_level: str | None = None,
                 charger: dict | None = None) -> dict:
    start = next((wp for wp in waypoints if wp['role'] == 'locpoint'), None)
    if start is None:
        start = next((wp for wp in waypoints if wp['role'] == 'home'), None)

    start_block = {'rmf_level': level, 'robot_map_name': map_data.get('name')}
    # The RMF level name need not match the vendor's. When it does not, the
    # adapter bridges them via rmf_config['robot_maps'] (fleet_adapter.py:297).
    robot_maps = {}
    if vendor_level and str(vendor_level).strip() != level:
        robot_maps = {level: {'name': map_data.get('name'),
                              'level': str(vendor_level).strip()}}
    if start is not None:
        start_block['localization_starting_point'] = {
            'x': round(start['pixel'][0]),
            'y': round(start['pixel'][1]),
            'heading': angle_to_radians(start.get('angle', 0.0)),
        }

    return {
        'map_transform': {
            level: {
                'transform_values': {
                    'rotation_degrees': 0,
                    'scale': round(scale, 7),
                    'tx_meters': 0,
                    'ty_meters': 0,
                }
            }
        },
        'rmf_fleet': {
            'name': fleet,
            'fleet_manager': {
                'prefix': 'api.lionsbot.io',
                'user': '$LIONSBOT_USER',
                'password': '$LIONSBOT_PASSWORD',
            },
            # --- VERIFY AGAINST YOUR MACHINE -------------------------------
            # Copied from the reference config. footprint and vicinity drive
            # RMF's collision avoidance; the battery and mass figures drive
            # its charge estimates. None of this comes from the API.
            'profile': {'footprint': 0.5, 'vicinity': 0.6},
            'limits': {'linear': [0.2, 0.03], 'angular': [0.1, 0.1]},
            'battery_system': {'voltage': 24.0, 'capacity': 40.0,
                               'charging_current': 26.4},
            'mechanical_system': {'mass': 80.0, 'moment_of_inertia': 20.0,
                                  'friction_coefficient': 0.2},
            'ambient_system': {'power': 20.0},
            'tool_system': {'power': 760.0},
            # ---------------------------------------------------------------
            'account_for_battery_drain': True,
            'recharge_threshold': 0.3,
            'recharge_soc': 1,
            'publish_fleet_state': True,
            'reversible': False,
            'task_capabilities': {'clean': True, 'loop': False,
                                  'delivery': False,
                                  'finishing_request': 'charge'},
        },
        'robots': {
            robot['robotEncodingId']: {
                'rmf_config': {
                    **({'robot_maps': robot_maps} if robot_maps else {}),
                    'robot_state_update_frequency': 10,
                    'charger': {'waypoint': charger['name'] if charger else None},
                    'start': start_block,
                },
                'robot_config': {
                    'uuid': robot['id'],
                    'max_delay': 15.0,
                    'navigation': {'max_retries': 5,
                                   'timeout_grace_sec': 60,
                                   'max_timeout_sec': 120,
                                   'xy_goal_tolerance': 20},
                },
            }
        },
    }


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def write_yaml(path: Path, data: dict, banner: str = '') -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w') as handle:
        if banner:
            handle.write(banner)
        yaml.safe_dump(data, handle, sort_keys=False, default_flow_style=False)
    print(f'  wrote {path}')


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Generate an Open-RMF site from the LionsBot cloud API.')
    parser.add_argument('--user', default=os.environ.get('LIONSBOT_USER'),
                        help='account email (or set LIONSBOT_USER)')
    parser.add_argument('--password', default=os.environ.get('LIONSBOT_PASSWORD'),
                        help='account password (or set LIONSBOT_PASSWORD; '
                             'prompted if omitted)')
    parser.add_argument('--list-robots', action='store_true',
                        help='print the robots on this account and exit')
    parser.add_argument('--robot', help='robotEncodingId of the robot to use')
    parser.add_argument('--site', help='site folder name, e.g. mysite')
    parser.add_argument('--fleet', help='RMF fleet name, e.g. r5')
    parser.add_argument('--level', help='RMF level name to emit; defaults to '
                                        "the vendor's level. If they differ, "
                                        'robot_maps is written to bridge them')
    parser.add_argument('--map-level', help="select the vendor map by its own "
                                            'level (disambiguates same-named maps)')
    parser.add_argument('--map-name', help='LionsBot map name (default: the '
                                           "robot's currently selected map)")
    parser.add_argument('--nav-graph', default='0.yaml',
                        help='nav graph filename (default: 0.yaml)')
    parser.add_argument('--scale', type=float,
                        help='metres per pixel; skips job-report derivation')
    parser.add_argument('--job-days', type=int, default=90,
                        help='how far back to look for cleaning jobs '
                             '(default: 90, max 365)')
    parser.add_argument('--angle-unit', choices=('rad', 'deg'), default='rad',
                        help='unit of marker angles (default: rad). The API '
                             'docs do not state this, but observed values are '
                             'full-precision floats near pi/2, so radians.')
    parser.add_argument('--output', default='../../../fleet_adapter_lionsbot/fleet_adapter',
                        help='adapter package directory to write into')
    args = parser.parse_args()

    if not args.user:
        sys.exit('Need --user or LIONSBOT_USER.')
    password = args.password or getpass.getpass('LionsBot password: ')

    api = LionsbotApi()
    print('Logging in...')
    api.login(args.user, password)

    # With no --robot this prints the roster and exits, which is also what
    # --list-robots does.
    robot = choose_robot(api, None if args.list_robots else args.robot)
    for required in ('site', 'fleet'):
        if not getattr(args, required):
            sys.exit(f'Need --{required}.')

    robot_uuid = robot['id']
    print(f"\nRobot {robot['robotEncodingId']} ({robot.get('robotType', '?')}) "
          f"uuid={robot_uuid}")

    print('\nReading maps...')
    map_data = choose_map(api, robot_uuid, args.map_name, args.map_level)
    map_id = map_data['id']
    vendor_level = str(map_data.get('level') or '').strip()
    level = args.level or vendor_level or 'L1'
    print(f"  map {map_data.get('name')!r} vendor level {vendor_level!r} "
          f'id={map_id}')
    if level != vendor_level:
        print(f'  RMF level {level!r} -> vendor level {vendor_level!r} '
              f'(bridged via robot_maps)')

    print('\nReading markers and zones...')
    markers = api.get(f'/robot/map/{map_id}/markers') or map_data.get('markers') or []
    zones = api.get(f'/robot/map/{map_id}/zones') or []
    home_poi = map_data.get('homePoi')
    print(f'  {len(markers)} marker(s), {len(zones)} zone(s)')

    print('\nReading cleaning job reports...')
    zone_paths = collect_zone_paths(api, robot_uuid, args.job_days)
    print(f'  coverage paths for {len(zone_paths)} zone(s)')

    if args.scale:
        scale = args.scale
        print(f'\nScale: {scale} m/px (from --scale)')
    else:
        scale, notes = derive_scale(zones, zone_paths)
        for note in notes:
            print(note)
        if scale is None:
            sys.exit(
                '\nCould not derive the scale: no zone has both a coverage '
                'path and a distance.\n'
                'Run one cleaning job per zone from the robot touchscreen, '
                'then retry -- or pass --scale.')
        print(f'\nScale: {scale:.6f} m/px (median of {len(notes)} zone(s))')

    waypoints = build_waypoints(markers, home_poi, zones, zone_paths, scale)
    charger = pick_charger(waypoints)
    if charger is not None:
        charger['is_charger'] = True
    for marker in markers:
        for wp in waypoints:
            if wp['name'] == marker.get('name'):
                wp['angle'] = marker.get('angle', 0.0)
    if not waypoints:
        sys.exit('No usable markers or zones -- nothing to generate.')

    print(f'\n{len(waypoints)} waypoint(s):')
    for wp in waypoints:
        print(f"  {wp['name']:20s} {wp['role']:9s} "
              f"px=({wp['pixel'][0]:7.1f},{wp['pixel'][1]:7.1f}) "
              f"rmf=({wp['rmf'][0]:8.3f},{wp['rmf'][1]:9.3f})")

    if args.angle_unit == 'deg':
        def angle_to_radians(value):
            return round(math.radians(float(value or 0.0)), 9)
    else:
        def angle_to_radians(value):
            return round(float(value or 0.0), 9)

    root = Path(args.output).expanduser().resolve()
    if not root.is_dir():
        sys.exit(f'--output is not a directory: {root}')
    config_dir = root / 'configs' / args.site
    map_dir = root / 'maps' / args.site
    png_name = f'rmf_{args.site}.png'

    print('\nWriting files...')
    grid_link = map_data.get('gridPngLink')
    if not grid_link:
        refreshed = api.get(f'/robot/{robot_uuid}/map', {'requireMarkers': 'true'})
        for candidate in (refreshed or {}).get('workSiteMaps', []):
            if candidate.get('id') == map_id and candidate.get('gridPngLink'):
                grid_link = candidate['gridPngLink']
                print('  (re-fetched the grid link)')
                break
    if grid_link:
        map_dir.mkdir(parents=True, exist_ok=True)
        if api.download(grid_link, map_dir / png_name):
            print(f'  wrote {map_dir / png_name}')
    else:
        print('  ! map has no gridPngLink; supply the floor plan image '
              f'yourself as {png_name}', file=sys.stderr)

    write_yaml(config_dir / f'config_{args.fleet}.yaml',
               build_config(args.fleet, level, robot, map_data, waypoints,
                            scale, angle_to_radians, vendor_level, charger),
               '# Generated by bootstrap_site.py. Check the block marked\n'
               '# VERIFY AGAINST YOUR MACHINE before running real traffic.\n')

    write_yaml(map_dir / args.nav_graph,
               build_nav_graph(args.site, level, waypoints),
               '# Generated by bootstrap_site.py.\n'
               '# Any waypoint the run reported as a PLACEHOLDER has no real\n'
               '# position -- the API gave none. Drag it onto the floor plan.\n'
               '# LANES ARE PLACEHOLDERS: every waypoint is joined to a single\n'
               '# hub in a straight line, ignoring walls and corridors. Good\n'
               '# enough to prove the stack works. Redraw before real use.\n')

    write_yaml(map_dir / 'dock_summary.yaml',
               build_dock_summary(args.fleet, level, waypoints, zone_paths,
                                  scale),
               '# Generated by bootstrap_site.py. Zone footprints are the\n'
               '# convex hull of each coverage path, used only to reserve\n'
               '# space in the RMF traffic schedule.\n')

    grid_size = None
    png_path = map_dir / png_name
    if png_path.is_file():
        try:
            header = png_path.read_bytes()[16:24]
            grid_size = struct.unpack('>II', header)
        except Exception:  # noqa: BLE001 - walls are optional
            grid_size = None

    write_yaml(map_dir / f'rmf_{args.site}.building.yaml',
               build_building(args.site, level, waypoints, scale, png_name,
                              grid_size),
               '# Generated by bootstrap_site.py.\n')

    print(f"""
Done. Next:

  1. Put these in devel/lionsbot/.env

       FOLDER_NAME={args.site}
       FLOOR_NAME={level}
       FLEET_1_CONFIG=config_{args.fleet}.yaml
       FLEET_1_NAV_GRAPH={args.nav_graph}

  2. docker compose up -d --build

  3. Open the dashboard and check the robot sits where you expect.
     If it is offset, the scale is wrong; if rotated, your floor plan is
     not the robot's grid image.

  4. Redraw the lanes in rmf_site before you run real traffic.
""")


if __name__ == '__main__':
    main()
