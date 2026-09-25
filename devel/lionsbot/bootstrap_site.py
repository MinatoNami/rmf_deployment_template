#!/usr/bin/env python3
"""Generate a multi-level Open-RMF site from the LionsBot cloud API.

Pulls each floor's map, markers and zones from api.lionsbot.io and writes the
four files the fleet adapter and rmf-core need:

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

# A marker naming a cleaning zone may carry one of these suffixes, since some
# setups will not let a marker and a zone share a name outright.
ZONE_MARKER_SUFFIXES = ('_loc', '_point', '_marker', '_wp')
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

    # A cleaning zone has no geometry anywhere in the API -- /zones returns
    # name, type, area, distance and time and nothing else -- so its position
    # normally has to come from a coverage path. A marker placed at the zone is
    # the way to supply it directly: markers DO carry coordinates, and the
    # robot cleans by zone NAME, so the waypoint keeps the zone's name and the
    # marker contributes only the position.
    zone_names = {z.get('name') for z in zones
                  if str(z.get('type', '')).lower() in ('clean', 'cleaning', '')}

    zone_points: dict[str, tuple[float, float]] = {}
    zone_markers: dict[str, str] = {}
    for marker in markers:
        name = str(marker.get('name') or '')
        point = centroid(parse_marker_coordinates(marker.get('coordinates')))
        if point is None or not name:
            continue
        target = name if name in zone_names else None
        if target is None:
            for suffix in ZONE_MARKER_SUFFIXES:
                if name.endswith(suffix) and name[:-len(suffix)] in zone_names:
                    target = name[:-len(suffix)]
                    break
        if target and target not in zone_points:
            zone_points[target] = point
            zone_markers[target] = name
    consumed_markers = set(zone_markers.values())

    if zone_points:
        pairs = ', '.join(f'{z} <- {zone_markers[z]}'
                          for z in sorted(zone_markers))
        print(f'  {len(zone_points)} zone(s) positioned from a marker: {pairs}')

    for marker in markers:
        name = str(marker.get('name') or '')
        if name in consumed_markers:
            # Folded into its zone below; adding it again would leave a stray
            # waypoint sitting on top of the zone.
            continue
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
        add(name, point, role)

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
        # A coverage path is better than a marker -- it is the real extent
        # rather than one point someone stood the robot on -- so prefer it.
        point = centroid(flatten_segments(zone_paths.get(name, [])))
        if point is None:
            point = zone_points.get(name)
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


# ---------------------------------------------------------------------------
# Multi-level assembly
#
# Every floor is its own LionsBot map with its own pixel grid and its own
# metres-per-pixel. RMF needs them in ONE frame, because a lift has a single
# position shared by every level it serves. Two things make that work:
#
#   * an ANCHOR marker present on every floor -- the lift, whose shaft is
#     physically the same place on each floor. Translating each level so its
#     anchors coincide puts every level in a common frame.
#
#   * FIDUCIALS, which are how building_map_tools aligns a non-reference level.
#     Only the reference level takes its scale from its own `measurements`;
#     every other level is aligned to the reference through shared named
#     points, and Transform.set_from_fiducials needs at least TWO pairs because
#     it derives scale and rotation from the bearings and distances BETWEEN
#     pairs. Given none it returns early and the level silently keeps scale 1.0,
#     publishing its whole graph in pixels.
#
# Rather than trust hand-placed fiducials, this synthesises three per level at
# fixed METRE offsets from the anchor, converted through that level's own
# scale. set_from_fiducials then recovers exactly that scale and exactly the
# translation that makes the anchors coincide. See FIDUCIAL_OFFSETS_M.
# ---------------------------------------------------------------------------

# Non-collinear, and far enough apart that pixel rounding is negligible.
FIDUCIAL_OFFSETS_M = ((0.0, 0.0), (20.0, 0.0), (0.0, 20.0))


def level_waypoint_name(name: str, level: str) -> str:
    """RMF resolves waypoints by name across the WHOLE building, not per level,
    so two floors cannot both own a waypoint called `lift`. Suffix the level,
    unless whoever placed the marker already did."""
    return name if name.endswith(f'_{level}') else f'{name}_{level}'


def classify_lift_role(marker_name: str, lift_marker: str,
                       landing_marker: str) -> str | None:
    """'cabin', 'landing' or None, from the marker's name."""
    base = marker_name.lower()
    if base == lift_marker.lower() or base.startswith(f'{lift_marker.lower()}_'):
        if base.startswith(landing_marker.lower()):
            return 'landing'
        return 'cabin'
    if base.startswith(landing_marker.lower()):
        return 'landing'
    return None


def build_levels(api: LionsbotApi, selections: list[dict],
                 scales: dict, zone_paths: dict,
                 lift_marker: str, landing_marker: str) -> list[dict]:
    """One entry per floor, with waypoints already in the common frame."""
    levels = []
    for selection in selections:
        map_data = selection['map']
        level = selection['level']
        map_id = map_data['id']
        print(f"\n--- {level} (vendor map {map_data.get('name')!r}) ---")

        markers = (api.get(f'/robot/map/{map_id}/markers', quiet=True)
                   or map_data.get('markers') or [])
        zones = api.get(f'/robot/map/{map_id}/zones', quiet=True) or []
        print(f'  {len(markers)} marker(s), {len(zones)} zone(s)')

        scale = scales.get(level)
        if scale is None:
            scale, notes = derive_scale(zones, zone_paths)
            for note in notes:
                print(note)
            if scale is None:
                sys.exit(
                    f'\nCould not derive the scale for {level}: no zone on '
                    f'{map_data.get("name")!r} has both a coverage path and a '
                    'distance.\nRun one cleaning job in a zone on that map, '
                    f'then retry -- or pass --scale {level}=<metres-per-pixel>.')
            print(f'  scale {scale:.7f} m/px (derived)')
        else:
            print(f'  scale {scale:.7f} m/px (given)')

        waypoints = build_waypoints(markers, map_data.get('homePoi'), zones,
                                    zone_paths, scale)
        if not waypoints:
            sys.exit(f'{level}: no usable markers or zones.')

        angles = {m.get('name'): m.get('angle', 0.0) for m in markers}
        anchor_pixel = None
        for wp in waypoints:
            raw = wp['name']
            wp['angle'] = angles.get(raw, 0.0)
            wp['lift_role'] = classify_lift_role(raw, lift_marker,
                                                 landing_marker)
            if wp['lift_role'] == 'cabin':
                anchor_pixel = wp['pixel']
            # The vendor name is what the robot is commanded with -- zones are
            # cleaned by name, not by RMF coordinates -- so keep it alongside
            # the level-suffixed RMF waypoint name.
            wp['vendor_name'] = raw
            wp['name'] = level_waypoint_name(raw, level)

        if anchor_pixel is None:
            sys.exit(
                f'{level}: no {lift_marker!r} marker. Every floor needs one -- '
                'it is the shared physical point the levels are aligned on. '
                'Place it on the robot touchscreen, or pass --lift-marker.')

        levels.append({
            'level': level,
            'is_current': selection.get('is_current', False),
            'map': map_data,
            'vendor_level': str(map_data.get('level') or '').strip(),
            'scale': scale,
            'waypoints': waypoints,
            'zones': zones,
            'anchor_pixel': anchor_pixel,
            'anchor_native_rmf': to_rmf(anchor_pixel, scale),
        })
    return levels


def place_levels_in_common_frame(levels: list[dict]) -> None:
    """Translate every level so all anchors land on the reference anchor."""
    reference = levels[0]
    ref_anchor = reference['anchor_native_rmf']
    for entry in levels:
        dx = ref_anchor[0] - entry['anchor_native_rmf'][0]
        dy = ref_anchor[1] - entry['anchor_native_rmf'][1]
        entry['offset'] = (dx, dy)
        for wp in entry['waypoints']:
            wp['rmf'] = (wp['rmf'][0] + dx, wp['rmf'][1] + dy)
        if dx or dy:
            print(f"  {entry['level']}: shifted ({dx:+.3f}, {dy:+.3f}) m to "
                  f'put its lift on the reference lift')


def build_fiducials(scale: float, anchor_pixel: tuple) -> list:
    """Three synthetic fiducials at fixed metre offsets from the anchor."""
    return [[round(anchor_pixel[0] + dx / scale, 4),
             round(anchor_pixel[1] + dy / scale, 4),
             f'ref{index + 1}']
            for index, (dx, dy) in enumerate(FIDUCIAL_OFFSETS_M)]


def build_nav_graph(site: str, levels: list[dict], lift_name: str,
                    lift_dims: tuple) -> dict:
    """The file the fleet adapter actually reads.

    Lift behaviour is decided here and nowhere else. A waypoint carrying
    `lift: <name>` is inside the cabin, and rmf_fleet_adapter attaches the
    events to the lanes that cross the boundary (parse_graph.cpp:286):

        landing -> cabin   LiftSessionBegin  (calls the lift, waits for it)
        cabin -> landing   LiftDoorOpen + LiftSessionEnd
        cabin -> cabin     nothing -- both ends in one cabin IS the ride

    The `lifts:` block is mandatory once any vertex names a lift; without it
    the adapter throws "Lift properties for [...] were not provided".
    """
    graph_levels = {}
    for entry in levels:
        waypoints = entry['waypoints']
        vertices = []
        for wp in waypoints:
            props: dict = {'name': wp['name']}
            if wp['lift_role'] == 'cabin':
                props['lift'] = lift_name
            elif wp['lift_role'] == 'landing':
                props['is_holding_point'] = True
                props['is_parking_spot'] = True
            if wp['role'] == 'dock':
                props.update({'is_charger': True, 'is_parking_spot': True,
                              'dock_name': wp['name'],
                              'robot_dock_name': wp['name']})
            elif wp.get('is_charger'):
                props['is_charger'] = True
                props['is_parking_spot'] = True
            if wp['role'] == 'zone':
                props.update({'is_cleaning_zone': True,
                              'robot_zone_name': wp['vendor_name']})
            vertices.append([wp['rmf'][0], wp['rmf'][1], props])

        cabin = next((i for i, wp in enumerate(waypoints)
                      if wp['lift_role'] == 'cabin'), None)
        landing = next((i for i, wp in enumerate(waypoints)
                        if wp['lift_role'] == 'landing'), None)

        # Everything hangs off the landing where there is one: it is the only
        # waypoint guaranteed to exist on every floor, and it is what the lift
        # lanes attach to. Falling back to the charger, then anything.
        hub = landing
        if hub is None:
            hub = next((i for i, wp in enumerate(waypoints)
                        if wp['role'] == 'dock' or wp.get('is_charger')), None)
        if hub is None:
            hub = next((i for i, wp in enumerate(waypoints)
                        if i != cabin), 0)

        lanes = []
        for i, wp in enumerate(waypoints):
            if i == hub or i == cabin:
                continue
            if wp['role'] == 'dock':
                lanes.append([hub, i, {'dock_name': wp['name']}])
                lanes.append([i, hub, {'undock_name': wp['name']}])
            else:
                lanes.append([hub, i, {}])
                lanes.append([i, hub, {}])

        # The two lanes that make the lift work. Keep the cabin reachable ONLY
        # from the landing: a lane from anywhere else would have RMF plan a
        # route into the shaft without opening a session.
        if cabin is not None and landing is not None:
            lanes.append([landing, cabin, {}])
            lanes.append([cabin, landing, {}])

        graph_levels[entry['level']] = {'lanes': lanes,
                                        'vertices': vertices}

    reference = levels[0]
    anchor = reference['anchor_native_rmf']
    return {
        'building_name': f'rmf_{site}',
        'doors': {},
        'lifts': {
            lift_name: {
                'position': [round(anchor[0], 6), round(anchor[1], 6), 0.0],
                'dims': [lift_dims[0], lift_dims[1]],
            }
        },
        'levels': graph_levels,
    }


def build_building(site: str, levels: list[dict], lift_name: str,
                   lift_dims: tuple, level_height: float,
                   grid_sizes: dict) -> dict:
    """The floor plan rmf-web draws, and where the lift's geometry lives.

    Coordinates here are PIXELS in each level's own drawing; the level
    transform converts them. Lift door x/y are metres from the cabin centre.
    """
    lane_params = {
        'bidirectional': [4, True],
        'demo_mock_floor_name': [1, ''],
        'demo_mock_lift_name': [1, ''],
        'graph_idx': [2, 0],
        'mutex': [1, ''],
        'orientation': [1, ''],
        'speed_limit': [3, 0],
    }
    wall_params = {
        'alpha': [3, 1], 'texture_height': [3, 2.5],
        'texture_name': [1, 'default'], 'texture_scale': [3, 1],
        'texture_width': [3, 1],
    }

    building_levels = {}
    for index, entry in enumerate(levels):
        level = entry['level']
        waypoints = entry['waypoints']
        scale = entry['scale']
        vertices = [[wp['pixel'][0], wp['pixel'][1], 0, wp['name']]
                    for wp in waypoints]

        hub = next((i for i, wp in enumerate(waypoints)
                    if wp['lift_role'] == 'landing'), 0)
        lanes = [[hub, i, dict(lane_params)]
                 for i in range(len(vertices)) if i != hub]

        # A measurement pins the REFERENCE level's scale; other levels get
        # theirs from the fiducials below.
        base = len(vertices)
        vertices.append([0.0, 0.0, 0, ''])
        vertices.append([100.0, 0.0, 0, ''])
        measurements = [[base, base + 1,
                         {'distance': [3, round(100.0 * scale, 6)]}]]

        # Without walls the dashboard has nothing to frame the view with and
        # renders an empty canvas, so trace the drawing's extent.
        walls = []
        grid = grid_sizes.get(level)
        if grid:
            corner = len(vertices)
            width, height = grid
            for x, y in ((0, 0), (width, 0), (width, height), (0, height)):
                vertices.append([float(x), float(y), 0, ''])
            for offset in range(4):
                walls.append([corner + offset, corner + (offset + 1) % 4,
                              dict(wall_params)])

        building_levels[level] = {
            'drawing': {'filename': entry['png_name']},
            'elevation': round(index * level_height, 3),
            'fiducials': build_fiducials(scale, entry['anchor_pixel']),
            'flattened_x_offset': 0,
            'flattened_y_offset': 0,
            'floors': [],
            'lanes': lanes,
            'measurements': measurements,
            'vertices': vertices,
            'walls': walls,
        }

    reference = levels[0]
    names = [entry['level'] for entry in levels]
    return {
        'coordinate_system': 'reference_image',
        'crowd_sim': {
            'agent_groups': [], 'agent_profiles': [], 'enable': 0,
            'goal_sets': [], 'model_types': [], 'states': [],
            'transitions': [], 'update_time_step': 0.1,
        },
        'graphs': {},
        'levels': building_levels,
        'reference_level_name': reference['level'],
        'lifts': {
            lift_name: {
                # Pixels in the REFERENCE level's drawing.
                'x': reference['anchor_pixel'][0],
                'y': reference['anchor_pixel'][1],
                'yaw': 0,
                'width': lift_dims[0],
                'depth': lift_dims[1],
                'initial_floor_name': names[0],
                'lowest_floor': names[0],
                'highest_floor': names[-1],
                'reference_floor_name': names[0],
                # Every level named here must exist under `levels:`;
                # building_map_tools looks its elevation up directly and raises
                # KeyError on one that does not.
                'level_doors': {name: [f'{lift_name}_door'] for name in names},
                'doors': {
                    f'{lift_name}_door': {
                        'door_type': 2,          # double sliding
                        'motion_axis_orientation': 0,
                        'width': round(min(lift_dims) * 0.8, 3),
                        'x': 0,                  # metres from the cabin centre
                        'y': round(lift_dims[1] / 2.0, 3),
                    }
                },
                'plugins': False,
            }
        },
        'name': f'rmf_{site}',
    }


def build_dock_summary(fleet: str, levels: list[dict],
                       zone_paths: dict) -> dict:
    entries: dict = {}
    pathless = []
    for entry in levels:
        for wp in entry['waypoints']:
            if wp['role'] != 'zone':
                continue
            raw = wp['vendor_name']
            hull = convex_hull(flatten_segments(zone_paths.get(raw, [])))
            if hull:
                dx, dy = entry['offset']
                path = [[*[c + d for c, d in
                           zip(to_rmf(point, entry['scale']), (dx, dy))], 0.0]
                        for point in hull]
            else:
                path = [[wp['rmf'][0], wp['rmf'][1], 0.0]]
                pathless.append(wp['name'])
            entries[wp['name']] = {
                'finish_waypoint': wp['name'],
                'level_name': entry['level'],
                'robot_zone_name': raw,
                'path': path,
            }
    if pathless:
        print(f'  ! {len(pathless)} zone(s) reserved as a single point rather '
              f"than a footprint: {', '.join(pathless)}", file=sys.stderr)
    return {fleet: entries}


def build_config(fleet: str, levels: list[dict], robot: dict,
                 angle_to_radians, charger: dict | None,
                 start_level: str | None = None,
                 merge_lane: float = 5.0,
                 merge_waypoint: float = 0.5) -> dict:
    """Fleet config. Every level in the nav graph needs a map_transform entry:
    the adapter looks the transform up by RMF level name and refuses to place a
    robot on a level it cannot find.

    The transform is per level because each floor is its own robot map with its
    own scale. tx/ty stay 0 -- the adapter converts robot pixels to RMF metres
    for that level, and the nav graph is already in the common frame because
    bootstrap wrote it that way.
    """
    map_transform = {}
    robot_maps = {}
    for entry in levels:
        level = entry['level']
        # ty is NEGATED. MapTransform.robot_to_rmf_meters feeds the
        # transform (px, -py) and builds it as Transform(.., tx, -ty_meters),
        # so rmf_y = -scale*py - ty_meters. To shift a level by +dy metres the
        # config therefore needs ty_meters = -dy. tx has no such flip.
        map_transform[level] = {
            'transform_values': {
                'rotation_degrees': 0,
                'scale': round(entry['scale'], 7),
                'tx_meters': round(entry['offset'][0], 6),
                'ty_meters': round(-entry['offset'][1], 6),
            }
        }
        robot_maps[level] = {'name': entry['map'].get('name'),
                             'level': entry['vendor_level'] or level}

    # Where the robot is when the adapter starts. The adapter switches the
    # robot to this map and hot-localizes it at this pixel, so it has to be the
    # floor the robot is actually standing on -- localization fails forever
    # otherwise. Defaults to the charger's floor, then to whichever map the
    # robot currently has selected.
    start_entry = None
    if start_level:
        start_entry = next((e for e in levels if e['level'] == start_level),
                           None)
        if start_entry is None:
            sys.exit(f'--start-level {start_level!r} is not one of '
                     f"{[e['level'] for e in levels]}")
    if start_entry is None and charger is not None:
        start_entry = next((e for e in levels
                            if any(w is charger for w in e['waypoints'])), None)
    if start_entry is None:
        start_entry = next((e for e in levels if e.get('is_current')),
                           levels[0])

    start_block = {
        'rmf_level': start_entry['level'],
        'robot_map_name': start_entry['map'].get('name'),
    }
    start_wp = charger if (charger is not None
                           and any(w is charger
                                   for w in start_entry['waypoints'])) else None
    if start_wp is None:
        start_wp = next((w for w in start_entry['waypoints']
                         if w['lift_role'] == 'landing'),
                        start_entry['waypoints'][0])
    start_block['localization_starting_point'] = {
        'x': round(start_wp['pixel'][0]),
        'y': round(start_wp['pixel'][1]),
        'heading': angle_to_radians(start_wp.get('angle', 0.0)),
    }

    return {
        'map_transform': map_transform,
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
            # How far off the graph a robot may be and still be placed on it.
            # RMF defaults to 0.3 m from a lane and 1 mm from a waypoint, which
            # assumes a nav graph drawn along the routes robots actually take.
            # The graph this script emits is a placeholder star, so a robot
            # parked anywhere sensible is metres from the nearest lane and RMF
            # reports "can't get location" and refuses to plan at all. Tighten
            # these once the lanes are redrawn.
            'max_merge_lane_distance': merge_lane,
            'max_merge_waypoint_distance': merge_waypoint,
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
                    'robot_maps': robot_maps,
                    'robot_state_update_frequency': 10,
                    'charger': {
                        'waypoint': charger['name'] if charger else None},
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


def parse_scales(raw: str | None) -> dict:
    """--scale 0.052  or  --scale L8=0.0519913,L9=0.0632"""
    if not raw:
        return {}
    if '=' not in raw:
        return {'*': float(raw)}
    scales = {}
    for item in raw.split(','):
        level, _, value = item.partition('=')
        scales[level.strip()] = float(value)
    return scales


def select_maps(api: LionsbotApi, robot_uuid: str,
                spec: str | None, map_level: str | None) -> list[dict]:
    """--maps NAME[=LEVEL],... in the order given; the FIRST is the reference
    level, whose own measurements set the scale everything else aligns to."""
    payload = api.get(f'/robot/{robot_uuid}/map', {'requireMarkers': 'true'})
    if not payload:
        sys.exit('Could not read the robot map list.')
    maps = payload.get('workSiteMaps') or []
    if not maps:
        sys.exit('This robot has no maps. Create one on the touchscreen first.')
    by_name = {m.get('name'): m for m in maps}

    if not spec:
        current = payload.get('currentMapId')
        chosen = next((m for m in maps if m.get('id') == current), maps[0])
        if map_level:
            chosen = next((m for m in maps
                           if str(m.get('level', '')).strip() == map_level),
                          chosen)
        return [{'map': chosen, 'is_current': True,
                 'level': str(chosen.get('level') or '').strip() or 'L1'}]

    selections = []
    for item in spec.split(','):
        name, _, level = item.partition('=')
        name = name.strip()
        if name not in by_name:
            print('\nAvailable maps:\n', file=sys.stderr)
            for m in maps:
                print(f"  {m.get('name')!r} level={m.get('level')!r} "
                      f"synced={m.get('synced')}", file=sys.stderr)
            sys.exit(f'\nNo map named {name!r}.')
        chosen = by_name[name]
        selections.append({
            'map': chosen,
            'is_current': chosen.get('id') == payload.get('currentMapId'),
            'level': (level.strip()
                      or str(chosen.get('level') or '').strip() or name),
        })

    seen = [s['level'] for s in selections]
    if len(seen) != len(set(seen)):
        sys.exit(f'Duplicate RMF level names in --maps: {seen}. '
                 'Disambiguate with NAME=LEVEL.')
    return selections


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Generate a multi-level Open-RMF site from the LionsBot '
                    'cloud API.')
    parser.add_argument('--user', default=os.environ.get('LIONSBOT_USER'),
                        help='account email (or set LIONSBOT_USER)')
    parser.add_argument('--password',
                        default=os.environ.get('LIONSBOT_PASSWORD'),
                        help='account password (or set LIONSBOT_PASSWORD; '
                             'prompted if omitted)')
    parser.add_argument('--list-robots', action='store_true',
                        help='print the robots on this account and exit')
    parser.add_argument('--robot', help='robotEncodingId of the robot to use')
    parser.add_argument('--site', help='site folder name, e.g. mysite')
    parser.add_argument('--fleet', help='RMF fleet name, e.g. r3')
    parser.add_argument('--maps',
                        help='floors to generate, in order, as '
                             'NAME[=LEVEL],... e.g. '
                             'office_new=L8,office_L9=L9. The FIRST is the '
                             'reference level. LEVEL defaults to the map\'s '
                             'own vendor level. Omit to generate just the '
                             "robot's current map")
    parser.add_argument('--map-level',
                        help='single-map mode: pick the map by vendor level')
    parser.add_argument('--nav-graph', default='0.yaml',
                        help='nav graph filename (default: 0.yaml)')
    parser.add_argument('--scale',
                        help='metres per pixel: one value for every level, or '
                             'LEVEL=VALUE,... Skips job-report derivation for '
                             'the levels named')
    parser.add_argument('--zone-paths',
                        help='JSON of {zone_name: [x1,y1,x2,y2,...]} in map '
                             'pixels, as written by derive_zone_paths.py. '
                             'Merged over the job-report paths, so a zone that '
                             'has never been cleaned still gets a real '
                             'position, a real footprint and a derived scale')
    parser.add_argument('--job-days', type=int, default=365,
                        help='how far back to look for cleaning jobs '
                             '(default: 365, max 365)')
    parser.add_argument('--lift-marker', default='lift',
                        help='marker name that means "inside the cabin". '
                             'Required on every floor: it is the shared '
                             'physical point the levels are aligned on '
                             '(default: lift)')
    parser.add_argument('--landing-marker', default='lift_waiting',
                        help='marker name prefix for the lift landing '
                             '(default: lift_waiting)')
    parser.add_argument('--lift-name', default='lift_1',
                        help='RMF lift name (default: lift_1)')
    parser.add_argument('--lift-dims', default='1.5x1.5',
                        help='cabin WIDTHxDEPTH in metres (default: 1.5x1.5)')
    parser.add_argument('--start-level',
                        help='floor the robot is standing on when the adapter '
                             'starts. The adapter switches the robot to that '
                             'map and hot-localizes it there, so a wrong value '
                             'means localization never succeeds. Defaults to '
                             "the charger's floor")
    parser.add_argument('--max-merge-lane-distance', type=float, default=5.0,
                        help='how far off a lane a robot may be and still be '
                             'placed on the graph, in metres (RMF default 0.3, '
                             'raised here because the emitted lanes are a '
                             'placeholder star, so a robot parked anywhere \n'
                             'sensible is metres off it). Tighten once lanes '
                             'are drawn')
    parser.add_argument('--max-merge-waypoint-distance', type=float,
                        default=0.5,
                        help='same, for snapping onto a waypoint rather than a '
                             'lane (RMF default 0.001)')
    parser.add_argument('--level-height', type=float, default=4.0,
                        help='metres between floors, for elevation '
                             '(default: 4.0)')
    parser.add_argument('--angle-unit', choices=('rad', 'deg'), default='rad',
                        help='unit of marker angles (default: rad). The API '
                             'docs do not state this, but observed values are '
                             'full-precision floats near pi/2, so radians.')
    parser.add_argument(
        '--output',
        default=str(Path(__file__).resolve().parent
                    / '../../../fleet_adapter_lionsbot/fleet_adapter'),
        help='adapter package directory to write into (default: a '
             'fleet_adapter_lionsbot checkout beside this repo, resolved '
             'from this script rather than the working directory)')
    args = parser.parse_args()

    if not args.user:
        sys.exit('Need --user or LIONSBOT_USER.')
    password = args.password or getpass.getpass('LionsBot password: ')

    api = LionsbotApi()
    print('Logging in...')
    api.login(args.user, password)

    robot = choose_robot(api, None if args.list_robots else args.robot)
    for required in ('site', 'fleet'):
        if not getattr(args, required):
            sys.exit(f'Need --{required}.')
    robot_uuid = robot['id']
    print(f"\nRobot {robot['robotEncodingId']} "
          f"({robot.get('robotType', '?')}) uuid={robot_uuid}")

    try:
        width, _, depth = args.lift_dims.partition('x')
        lift_dims = (float(width), float(depth))
    except ValueError:
        sys.exit(f'--lift-dims must be WIDTHxDEPTH, got {args.lift_dims!r}')

    if args.angle_unit == 'deg':
        def angle_to_radians(value):
            return round(math.radians(float(value or 0.0)), 9)
    else:
        def angle_to_radians(value):
            return round(float(value or 0.0), 9)

    print('\nReading maps...')
    selections = select_maps(api, robot_uuid, args.maps, args.map_level)
    for selection in selections:
        vendor = str(selection['map'].get('level') or '').strip()
        note = '' if vendor == selection['level'] else f' (vendor {vendor!r})'
        print(f"  {selection['level']}: {selection['map'].get('name')!r}{note}")

    print('\nReading cleaning job reports...')
    zone_paths = collect_zone_paths(api, robot_uuid, args.job_days)
    print(f'  coverage paths for {len(zone_paths)} zone(s): '
          f"{', '.join(zone_paths) or 'none'}")

    if args.zone_paths:
        # Paths captured live from the robot. They win over job reports: they
        # were planned for the zone as it is now, whereas a report can predate
        # the zone being redrawn.
        supplied = json.loads(Path(args.zone_paths).read_text())
        added = []
        for name, raw in supplied.items():
            segments = parse_path_segments(raw)
            if segments:
                zone_paths[name] = segments
                added.append(name)
        print(f"  plus {len(added)} live path(s) from {args.zone_paths}: "
              f"{', '.join(added) or 'none usable'}")

    scales_raw = parse_scales(args.scale)
    scales = {}
    for selection in selections:
        level = selection['level']
        if level in scales_raw:
            scales[level] = scales_raw[level]
        elif '*' in scales_raw:
            scales[level] = scales_raw['*']

    levels = build_levels(api, selections, scales, zone_paths,
                          args.lift_marker, args.landing_marker)

    print('\nAligning levels on the lift...')
    place_levels_in_common_frame(levels)

    charger = None
    for entry in levels:
        charger = pick_charger(entry['waypoints'])
        if charger is not None:
            charger['is_charger'] = True
            print(f"  charger: {charger['name']} on {entry['level']}")
            break
    if charger is None:
        print('  ! no charger waypoint on any floor; the fleet will come up '
              'with a charger it cannot resolve', file=sys.stderr)

    for entry in levels:
        print(f"\n{entry['level']}: {len(entry['waypoints'])} waypoint(s)")
        for wp in entry['waypoints']:
            role = wp['lift_role'] or wp['role']
            print(f"  {wp['name']:22s} {role:9s} "
                  f"px=({wp['pixel'][0]:7.1f},{wp['pixel'][1]:7.1f}) "
                  f"rmf=({wp['rmf'][0]:8.3f},{wp['rmf'][1]:9.3f})")

    root = Path(args.output).expanduser().resolve()
    if not root.is_dir():
        sys.exit(f'--output is not a directory: {root}')
    config_dir = root / 'configs' / args.site
    map_dir = root / 'maps' / args.site
    map_dir.mkdir(parents=True, exist_ok=True)

    print('\nWriting files...')
    grid_sizes = {}
    for entry in levels:
        level = entry['level']
        png_name = (f'rmf_{args.site}.png' if len(levels) == 1
                    else f'rmf_{args.site}_{level}.png')
        entry['png_name'] = png_name
        link = entry['map'].get('gridPngLink')
        if link and api.download(link, map_dir / png_name):
            print(f'  wrote {map_dir / png_name}')
        elif not link:
            print(f'  ! {level} has no gridPngLink; supply {png_name} '
                  'yourself', file=sys.stderr)
        png_path = map_dir / png_name
        if png_path.is_file():
            try:
                grid_sizes[level] = struct.unpack(
                    '>II', png_path.read_bytes()[16:24])
            except Exception:  # noqa: BLE001 - walls are optional
                pass

    write_yaml(config_dir / f'config_{args.fleet}.yaml',
               build_config(args.fleet, levels, robot, angle_to_radians,
                            charger, args.start_level,
                            args.max_merge_lane_distance,
                            args.max_merge_waypoint_distance),
               '# Generated by bootstrap_site.py. Check the block marked\n'
               '# VERIFY AGAINST YOUR MACHINE before running real traffic.\n')

    write_yaml(map_dir / args.nav_graph,
               build_nav_graph(args.site, levels, args.lift_name, lift_dims),
               '# Generated by bootstrap_site.py.\n'
               '# LANES ARE PLACEHOLDERS: on each floor every waypoint is\n'
               '# joined to the lift landing in a straight line, ignoring\n'
               '# walls and corridors. Good enough to prove the stack works.\n'
               '# Redraw before real use.\n'
               '# The lift lanes (landing <-> cabin) are the exception: those\n'
               '# are what generate the lift events, so keep them.\n')

    write_yaml(map_dir / 'dock_summary.yaml',
               build_dock_summary(args.fleet, levels, zone_paths),
               '# Generated by bootstrap_site.py. Zone footprints are the\n'
               '# convex hull of each coverage path, used only to reserve\n'
               '# space in the RMF traffic schedule.\n')

    write_yaml(map_dir / f'rmf_{args.site}.building.yaml',
               build_building(args.site, levels, args.lift_name, lift_dims,
                              args.level_height, grid_sizes),
               '# Generated by bootstrap_site.py.\n'
               '# Each level carries synthetic `fiducials` placed at fixed\n'
               '# metre offsets from the lift. Only the reference level takes\n'
               '# its scale from `measurements`; the rest are aligned through\n'
               '# these, and with fewer than two a level silently keeps scale\n'
               '# 1.0 and publishes its graph in pixels.\n')

    names = [entry['level'] for entry in levels]
    print(f"""
Done. Next:

  1. Put these in devel/lionsbot/.env

       FOLDER_NAME={args.site}
       FLOOR_NAME={names[0]}
       FLEET_1_CONFIG=config_{args.fleet}.yaml
       FLEET_1_NAV_GRAPH={args.nav_graph}

  2. docker compose up -d --build

  3. Check every level in the dashboard's Levels menu. A level that opens
     absurdly zoomed in kept scale 1.0 -- compare the wall spans across
     levels in GET /building_map; they should all be in metres.

  4. Redraw the lanes in rmf_site before you run real traffic.

Levels generated: {', '.join(names)}
""")


if __name__ == '__main__':
    main()
