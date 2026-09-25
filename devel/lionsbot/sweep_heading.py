#!/usr/bin/env python3
"""Sweep hot-localize heading through a full circle and score each one.

WHY THIS EXISTS

`PUT /robot/command/hot-localize/{robotId}` answers with
`{"success": bool, "percentage": int}`, where percentage is documented as the
"percentage confidence of robot being localized at requested position" (the
websocket `HotLocalizeFeedback` example shows 86 for a good one). That makes
heading a *measurable* parameter rather than a guess: hold x/y fixed at a
known localization point, walk the heading around the circle, and read the
confidence back for each.

Use it when hot-localize fails and you cannot tell whether the heading is
wrong, the position is wrong, or the robot is not where you think it is:

  - one clear peak          -> heading was wrong; take the argmax
  - low everywhere          -> x/y or the map is wrong, or the robot is not
                              physically standing on the point
  - 400 RobotStateNotRight  -> nothing to do with heading; fix the state

WHAT THE DOCS SAY THE PROCEDURE IS

Hot-localize is a *confirmation*, not a teleport. Per the LionsCloud guide
("Tier 2 Feature: Localization") the robot must already be physically standing
on a localization point that exists on the map:

  1. create a localization point with the touchscreen map editor
  2. physically push the robot onto it, as accurately as possible
  3. then call hot-localize with that point's coordinates

So the sweep only means anything if the robot really is parked on the point
named by --point. If it is not, every heading will score badly and that is the
correct answer, not a bug.

Headings are radians. This is stated twice in the API docs -- the REST body
documents `heading number<float>` as "heading in radians", and the websocket
`HotLocalizeCommand` as "Heading of the desired pose in radians" with the
example 3.141. Marker `angle` is radians too (example 1.57). Only the
`robotpose` websocket feed is degrees (example 103), which is why the fleet
adapter converts that one and only that one.

THIS COMMANDS A REAL ROBOT. Each step overwrites the robot's pose estimate.
It does not drive the machine, but it will leave the robot believing whatever
the last accepted call said -- so the sweep re-applies the best heading on the
way out (disable with --no-apply-best). `--yes` is required. Use --list to
inspect localization points without touching the robot at all.

The account is limited to 100 requests per minute, so --delay is clamped to
keep a sweep inside that.

EXAMPLES

    # what localization points exist on the L9 map? (read-only)
    python3 sweep_heading.py --robot R3-2200888-SCR --map office_L9 --list

    # sweep 24 headings at the L9 lift waiting point
    python3 sweep_heading.py --robot R3-2200888-SCR --map office_L9 \
        --point lift_waiting --steps 24 --yes
"""

from __future__ import annotations

import argparse
import getpass
import importlib.util
import json
import math
import os
import sys
import time
import types
import urllib.error
import urllib.request
from pathlib import Path

# bootstrap_site.py exits at import time if PyYAML is missing, but it only
# needs yaml to *write* the generated configs -- nothing this tool borrows
# (the API client, the marker parsing) touches it. Stub it when it is absent so
# a sweep stays stdlib-only: this runs on whatever laptop is next to the robot.
try:
    import yaml  # noqa: F401
except ImportError:
    sys.modules['yaml'] = types.ModuleType('yaml')

_spec = importlib.util.spec_from_file_location(
    'bootstrap_site', Path(__file__).with_name('bootstrap_site.py'))
_bs = importlib.util.module_from_spec(_spec)
_argv, sys.argv = sys.argv, ['bootstrap_site']
_spec.loader.exec_module(_bs)
sys.argv = _argv

# 100 requests/minute on the account, shared with everything else running.
# 0.8s between calls leaves headroom for the adapter if it is up.
MIN_DELAY = 0.8


def wrap(radians: float) -> float:
    """Into [-pi, pi), matching MapTransform.wrap_orientation in the adapter,
    so a value printed here drops straight into a fleet config."""
    return (radians + math.pi) % (2 * math.pi) - math.pi


def hot_localize(api, robot_uuid: str, x: int, y: int, heading: float,
                 accuracy: int | None = None) -> tuple[int, dict]:
    """PUT the command. Returns (http_status, body). Never raises for an HTTP
    error -- the error body carries `errorCode`, which is the whole point of
    calling this by hand rather than reading the adapter's logs."""
    payload = {'x': x, 'y': y, 'heading': heading}
    if accuracy is not None:
        payload['accuracy'] = accuracy
    request = urllib.request.Request(
        f'{api.prefix}/robot/command/hot-localize/{robot_uuid}',
        data=json.dumps(payload).encode(),
        headers={'Content-Type': 'application/json',
                 'Accept': 'application/json',
                 'Authorization': f'Bearer {api.token}'},
        method='PUT')
    try:
        with urllib.request.urlopen(request, timeout=_bs.TIMEOUT) as response:
            return response.status, json.load(response)
    except urllib.error.HTTPError as err:
        try:
            return err.code, json.load(err)
        except Exception:  # noqa: BLE001 - body may not be JSON
            return err.code, {}
    except urllib.error.URLError as err:
        return 0, {'errorMessage': str(err.reason)}


def resolve_point(api, map_id: str, name: str | None,
                  explicit: tuple[int, int] | None):
    """The localization points on a map, and the one to sweep at.

    /markers/locpoint is the dedicated endpoint -- it returns only real
    localization points, which is exactly the set hot-localize will accept.
    The generic /markers endpoint also returns POIs and dock points, and a POI
    is not a valid target however good its coordinates look.
    """
    locpoints = api.get(f'/robot/map/{map_id}/markers/locpoint') or []
    if explicit is not None:
        match = None
        for point in locpoints:
            centre = _bs.centroid(
                _bs.parse_marker_coordinates(point.get('coordinates')))
            if centre and (round(centre[0]), round(centre[1])) == explicit:
                match = point
                break
        return locpoints, match, explicit

    if not name:
        return locpoints, None, None

    exact = [p for p in locpoints if str(p.get('name', '')) == name]
    prefixed = [p for p in locpoints
                if str(p.get('name', '')).lower().startswith(name.lower())]
    candidates = exact or prefixed
    if not candidates:
        return locpoints, None, None
    if len(candidates) > 1:
        sys.exit(f'{name!r} matches {len(candidates)} localization points: '
                 f"{', '.join(str(p.get('name')) for p in candidates)}.\n"
                 'Name one exactly, or pass --x/--y.')
    point = candidates[0]
    centre = _bs.centroid(_bs.parse_marker_coordinates(point.get('coordinates')))
    if centre is None:
        sys.exit(f"Localization point {point.get('name')!r} has no usable "
                 f"coordinates: {point.get('coordinates')!r}")
    return locpoints, point, (round(centre[0]), round(centre[1]))


def print_locpoints(locpoints: list[dict]) -> None:
    if not locpoints:
        print('  (none -- create one with the touchscreen map editor)')
        return
    for point in locpoints:
        centre = _bs.centroid(
            _bs.parse_marker_coordinates(point.get('coordinates')))
        where = (f'x={round(centre[0]):>5} y={round(centre[1]):>5}'
                 if centre else 'no coordinates')
        angle = point.get('angle')
        angle_text = (f'{float(angle):+.4f} rad ({math.degrees(float(angle)):+7.1f} deg)'
                      if angle is not None else '?')
        print(f"  {str(point.get('name', '?')):28s} {where}  "
              f"angle={angle_text}  accuracy={point.get('accuracy')}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Sweep hot-localize heading through a full circle and '
                    'report the confidence the robot returns for each.',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--user', default=os.environ.get('LIONSBOT_USER'))
    parser.add_argument('--password',
                        default=os.environ.get('LIONSBOT_PASSWORD'))
    parser.add_argument('--robot', help='robot encoding ID, e.g. '
                                        'R3-2200888-SCR. Omit to list robots')
    parser.add_argument('--map', help='robot map name to sweep on, e.g. '
                                      'office_L9. Must be the map the robot '
                                      'currently has selected')
    parser.add_argument('--point', help='localization point name to sweep at. '
                                        'A unique prefix is enough')
    parser.add_argument('--x', type=int, help='pixel x, instead of --point')
    parser.add_argument('--y', type=int, help='pixel y, instead of --point')
    parser.add_argument('--accuracy', type=int,
                        help='optional accuracy field. The API accepts one and '
                             'markers carry one, but the fleet adapter has '
                             'never sent it. Defaults to the point\'s own '
                             'value; pass -1 to omit the field entirely')
    parser.add_argument('--steps', type=int, default=24,
                        help='headings to try (default: 24, i.e. every 15 deg)')
    parser.add_argument('--start-deg', type=float, default=0.0,
                        help='first heading in degrees (default: 0)')
    parser.add_argument('--span-deg', type=float, default=360.0,
                        help='arc to cover in degrees (default: 360)')
    parser.add_argument('--delay', type=float, default=2.0,
                        help=f'seconds between calls (default: 2.0, min '
                             f'{MIN_DELAY}; the account allows 100 req/min)')
    parser.add_argument('--no-apply-best', action='store_true',
                        help='leave the robot on the last heading tried '
                             'instead of re-applying the best one')
    parser.add_argument('--list', action='store_true',
                        help='list the localization points on the map and '
                             'exit. Commands nothing')
    parser.add_argument('--json', help='write the full result table here')
    parser.add_argument('--yes', action='store_true',
                        help='required: this overwrites the robot pose estimate')
    args = parser.parse_args()

    if not args.user:
        sys.exit('Need --user or LIONSBOT_USER.')
    if (args.x is None) != (args.y is None):
        sys.exit('--x and --y go together.')
    if args.steps < 1:
        sys.exit('--steps must be at least 1.')
    delay = max(args.delay, MIN_DELAY)
    if delay != args.delay:
        print(f'  ! --delay raised to {delay}s to stay inside the '
              '100 req/min limit', file=sys.stderr)

    password = args.password or getpass.getpass('LionsBot password: ')
    api = _bs.LionsbotApi()
    print('Logging in...')
    api.login(args.user, password)

    robot = _bs.choose_robot(api, args.robot)  # exits with a list if None
    robot_uuid = robot['id']
    print(f"\nRobot {robot['robotEncodingId']} "
          f"({robot.get('robotType', '?')}) online={robot.get('isOnline')}")
    if robot.get('isOnline') is False:
        sys.exit('The robot is offline. hot-localize reaches it over MQTT and '
                 'will answer 500 NoResponseFromRobot.')

    if not args.map:
        sys.exit('Need --map. Run with --robot only to see the robot list, or '
                 'check the map names in the fleet config.')
    payload = api.get(f'/robot/{robot_uuid}/map')
    maps = (payload or {}).get('workSiteMaps') or []
    chosen = next((m for m in maps if m.get('name') == args.map), None)
    if chosen is None:
        sys.exit(f'No map named {args.map!r} on this robot. '
                 f"Available: {[m.get('name') for m in maps]}")
    current = (payload or {}).get('currentMapId')
    on_this_map = chosen.get('id') == current

    print(f"\nMap {args.map!r} (level {chosen.get('level')!r}) "
          f"id={chosen.get('id')}")
    locpoints, point, pixel = resolve_point(
        api, chosen['id'], args.point,
        (args.x, args.y) if args.x is not None else None)

    print(f'\n{len(locpoints)} localization point(s) on this map:')
    print_locpoints(locpoints)

    if args.list:
        return

    if not on_this_map:
        sys.exit(f'\nThe robot is not on {args.map!r} right now (currentMapId='
                 f'{current}). hot-localize resolves against the selected map, '
                 'so switch the robot to this map first.')

    if pixel is None:
        sys.exit('\nNothing to sweep at. Pass --point NAME (from the list '
                 'above) or --x/--y.')
    if point is None:
        print(f'\n! x={pixel[0]} y={pixel[1]} is not one of this map\'s '
              'localization points. The docs say hot-localize expects the '
              'robot to be standing on a real one, so a low score here may '
              'mean nothing more than that.', file=sys.stderr)

    accuracy = args.accuracy
    if accuracy is None and point is not None:
        accuracy = point.get('accuracy')
    if accuracy == -1:
        accuracy = None

    label = point.get('name') if point else f'({pixel[0]}, {pixel[1]})'
    expected = point.get('angle') if point else None

    print(f'\nSweeping at {label}: x={pixel[0]} y={pixel[1]} '
          f'accuracy={accuracy if accuracy is not None else "(omitted)"}')
    if expected is not None:
        print(f'  the point\'s own angle is {float(expected):+.6f} rad '
              f'({math.degrees(float(expected)):+.1f} deg) -- the sweep should '
              'peak near here if everything is consistent')
    print(f'  {args.steps} heading(s) over {args.span_deg} deg from '
          f'{args.start_deg} deg, {delay}s apart '
          f'(~{args.steps * delay / 60:.1f} min)')

    if not args.yes:
        sys.exit('\nRefusing without --yes: every step overwrites the robot\'s '
                 'pose estimate. Re-run with --yes when someone is watching it.')

    results = []
    best = None
    try:
        for step in range(args.steps):
            degrees = args.start_deg + args.span_deg * step / args.steps
            heading = wrap(math.radians(degrees))
            status, body = hot_localize(api, robot_uuid, pixel[0], pixel[1],
                                        heading, accuracy)

            if status != 200:
                code = body.get('errorCode') or f'HTTP {status}'
                message = body.get('errorMessage') or ''
                print(f'  {degrees:7.1f} deg  {heading:+.4f} rad  '
                      f'! {code} {message}'.rstrip())
                # A rejected state is not a heading problem and will reject
                # every remaining step identically. Stop rather than burn
                # through the rate limit proving it.
                if code in ('RobotStateNotRight', 'UnsupportedCommand',
                            'MissingRobotById', 'RobotThingNotFoundById'):
                    print('\nThe robot rejected the command itself, so no '
                          'heading will work until that is fixed. Check that '
                          'it is undocked, idle and not in a critical state.',
                          file=sys.stderr)
                    break
                results.append({'degrees': degrees, 'radians': heading,
                                'error': code, 'message': message})
                time.sleep(delay)
                continue

            success = bool(body.get('success'))
            percentage = body.get('percentage')
            score = percentage if isinstance(percentage, (int, float)) else 0
            bar = '#' * int(round(score / 2.5))
            # The fleet adapter accepts on success and percentage > 25, so
            # mark that threshold to show what it would have done here.
            verdict = 'ACCEPT' if (success and score > 25) else '      '
            print(f'  {degrees:7.1f} deg  {heading:+.4f} rad  '
                  f'success={str(success):5s} {score:5.1f}%  {verdict} {bar}')
            results.append({'degrees': degrees, 'radians': heading,
                            'success': success, 'percentage': percentage})
            if success and (best is None or score > best[1]):
                best = (heading, score, degrees)
            time.sleep(delay)
    except KeyboardInterrupt:
        print('\nInterrupted.', file=sys.stderr)

    scored = [r for r in results if r.get('percentage') is not None]
    print(f'\n{len(scored)} of {args.steps} step(s) answered.')

    if best is None:
        print('No heading was accepted. Per the docs the robot has to be '
              'physically standing on this localization point -- if it is '
              'not, that is the first thing to fix, not the heading.')
    else:
        heading, score, degrees = best
        print(f'Best: {heading:+.8f} rad ({degrees:.1f} deg) at {score:.1f}% '
              'confidence')
        if expected is not None:
            delta = math.degrees(wrap(heading - float(expected)))
            print(f'  that is {delta:+.1f} deg from the point\'s own angle')
        print('\nFor the fleet config:\n'
              '        localization_starting_point:\n'
              f'          x: {pixel[0]}\n'
              f'          y: {pixel[1]}\n'
              f'          heading: {round(heading, 9)}')

        if not args.no_apply_best:
            print('\nRe-applying the best heading so the robot is not left on '
                  'the last one tried...')
            # The scan match is live, so the score moves a little between
            # identical calls and the robot can be briefly busy right after a
            # sweep. One retry, then report whatever actually came back --
            # "failed" without the numbers is useless for deciding what next.
            for attempt in (1, 2):
                status, body = hot_localize(api, robot_uuid, pixel[0],
                                            pixel[1], heading, accuracy)
                if status == 200 and body.get('success'):
                    print(f"  done, {body.get('percentage')}%")
                    break
                if status == 200:
                    print(f'  ! rejected at {body.get("percentage")}% '
                          f'(success=False)'
                          + ('; retrying' if attempt == 1 else ''),
                          file=sys.stderr)
                else:
                    print(f'  ! HTTP {status} '
                          f'{body.get("errorCode") or ""} '
                          f'{body.get("errorMessage") or ""}'.rstrip()
                          + ('; retrying' if attempt == 1 else ''),
                          file=sys.stderr)
                if attempt == 1:
                    time.sleep(max(delay, 2.0))
            else:
                print('  ! could not re-apply; the robot is left on the last '
                      'heading the sweep tried', file=sys.stderr)

    if args.json:
        out = Path(args.json)
        out.write_text(json.dumps(
            {'robot': robot['robotEncodingId'], 'map': args.map,
             'point': label, 'x': pixel[0], 'y': pixel[1],
             'accuracy': accuracy, 'point_angle': expected,
             'results': results}, indent=2))
        print(f'\nWrote {out}')


if __name__ == '__main__':
    main()
