#!/usr/bin/env python3
"""Derive cleaning-zone geometry by asking the robot to plan the zone.

WHY THIS EXISTS

A zone's position is the one thing the LionsBot REST API will not tell you.
`GET /robot/map/{mapId}/zones` returns name, type, area, distance and time --
no coordinates, no polygon -- and `clean_start` takes a zone by NAME, so the
caller is never expected to know where it is. The only sources of zone geometry
are the coverage path in a completed job report, and the robot's own live
navigation path.

Without either, bootstrap_site.py falls back to laying zones on a ring around
the known markers, which is a placeholder and nothing more. This tool replaces
that guess with the real thing.

HOW

The robot streams its planned path on request (websocket API, `RequestNavigationPath`):

    -> robotstatus   {"operation_cmd": "request_path", "content": {"status": true}}
    -> robotstatus   {"operation_cmd": "clean_start",  "content": {"zones": [...]}}
    <- robotpose     {"operation_fb": "robot_path", "content": {"path": [x1,y1,x2,y2,...]}}
    -> robotstatus   {"operation_cmd": "clean_stop",   "content": {"status": true}}

The path arrives in map pixels, which is the same frame the job-report coverage
path uses, so the output drops straight into bootstrap_site.py via
`--zone-paths`.

THIS COMMANDS A REAL ROBOT. It starts a cleaning run to make the robot plan,
then stops it. The robot will move. `--yes` is required, and `clean_stop` is
sent from a finally block so an interrupt still stops the machine.
"""

from __future__ import annotations

import argparse
import getpass
import importlib.util
import json
import os
import sys
import threading
import time
from pathlib import Path

try:
    import websocket
except ImportError:
    sys.exit('This tool needs websocket-client:\n'
             '  uv run --with pyyaml --with websocket-client '
             'python3 derive_zone_paths.py ...')

_spec = importlib.util.spec_from_file_location(
    'bootstrap_site', Path(__file__).with_name('bootstrap_site.py'))
_bs = importlib.util.module_from_spec(_spec)
_argv, sys.argv = sys.argv, ['bootstrap_site']
_spec.loader.exec_module(_bs)
sys.argv = _argv

WS_STATUS = '/openapi/v2/ws/robotstatus'
WS_POSE = '/openapi/v2/ws/robotpose'


class RobotChannel:
    """One websocket to LionsCloud. The JWT goes on the handshake as a
    subprotocol, which becomes the Sec-WebSocket-Protocol header."""

    def __init__(self, prefix: str, path: str, token: str, label: str):
        self.label = label
        self.url = f'wss://{prefix}{path}'
        self._messages: list[dict] = []
        self._lock = threading.Lock()
        self._open = threading.Event()
        self.app = websocket.WebSocketApp(
            self.url,
            subprotocols=[token],
            on_open=lambda _: self._open.set(),
            on_message=self._on_message,
            on_error=lambda _, err: print(f'  ! {label} error: {err}',
                                          file=sys.stderr),
        )
        self._thread = threading.Thread(
            target=self.app.run_forever, kwargs={'ping_interval': 20},
            daemon=True)

    def _on_message(self, _app, raw: str) -> None:
        try:
            message = json.loads(raw)
        except (TypeError, ValueError):
            return
        with self._lock:
            self._messages.append(message)

    def connect(self, timeout: float = 20.0) -> None:
        self._thread.start()
        if not self._open.wait(timeout):
            sys.exit(f'{self.label}: websocket did not open within {timeout}s. '
                     'A 429 here means the account is rate limited; wait and '
                     'retry.')
        print(f'  {self.label} connected')

    def send(self, payload: dict) -> None:
        self.app.send(json.dumps(payload))

    def drain(self, operation_fb: str | None = None) -> list[dict]:
        with self._lock:
            messages, self._messages = self._messages, []
        if operation_fb is None:
            return messages
        return [m for m in messages if m.get('operation_fb') == operation_fb]

    def close(self) -> None:
        try:
            self.app.close()
        except Exception:  # noqa: BLE001 - shutting down anyway
            pass


def wait_for_status(status: RobotChannel, robot_id: str,
                    timeout: float = 20.0) -> dict | None:
    """The robot's own view of itself, from the telemetry it already streams."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for message in status.drain('touchscreen_robot_status'):
            if message.get('robot_id') in (None, robot_id):
                return message.get('content') or {}
        time.sleep(0.25)
    return None


def collect_path(pose: RobotChannel, robot_id: str, settle: float,
                 timeout: float) -> list[float]:
    """Longest `robot_path` seen. The robot re-sends the whole path as it
    plans, so the longest is the finished one; stop once it stops growing."""
    best: list[float] = []
    deadline = time.monotonic() + timeout
    unchanged_since = None
    while time.monotonic() < deadline:
        for message in pose.drain('robot_path'):
            if message.get('robot_id') not in (None, robot_id):
                continue
            path = (message.get('content') or {}).get('path') or []
            if len(path) > len(best):
                best = list(path)
                unchanged_since = None
        if best and unchanged_since is None:
            unchanged_since = time.monotonic()
        if unchanged_since and time.monotonic() - unchanged_since >= settle:
            break
        time.sleep(0.25)
    return best


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--user', default=os.environ.get('LIONSBOT_USER'))
    parser.add_argument('--password',
                        default=os.environ.get('LIONSBOT_PASSWORD'))
    parser.add_argument('--robot', required=True, help='robotEncodingId')
    parser.add_argument('--map', required=True,
                        help='vendor map name holding the zones')
    parser.add_argument('--zones',
                        help='comma-separated zone names (default: every '
                             'clean zone on the map)')
    parser.add_argument('--out', default='zone_paths.json',
                        help='where to write the paths (default: '
                             'zone_paths.json)')
    parser.add_argument('--settle-seconds', type=float, default=6.0,
                        help='stop once the path has not grown for this long')
    parser.add_argument('--timeout-seconds', type=float, default=90.0,
                        help='give up on a zone after this long')
    parser.add_argument('--yes', action='store_true',
                        help='required: confirms the robot will be commanded '
                             'to start cleaning and will MOVE')
    args = parser.parse_args()

    if not args.yes:
        sys.exit('This starts a real cleaning run so the robot plans a path, '
                 'then stops it.\nThe robot WILL move. Re-run with --yes when '
                 'someone is watching it.')
    if not args.user:
        sys.exit('Need --user or LIONSBOT_USER.')
    password = args.password or getpass.getpass('LionsBot password: ')

    api = _bs.LionsbotApi()
    print('Logging in...')
    api.login(args.user, password)

    robot = _bs.choose_robot(api, args.robot)
    robot_uuid = robot['id']
    payload = api.get(f'/robot/{robot_uuid}/map', {'requireMarkers': 'true'})
    maps = (payload or {}).get('workSiteMaps') or []
    chosen = next((m for m in maps if m.get('name') == args.map), None)
    if chosen is None:
        sys.exit(f'No map named {args.map!r} on this robot. '
                 f'Available: {[m.get("name") for m in maps]}')
    if chosen.get('id') != (payload or {}).get('currentMapId'):
        sys.exit(f'The robot is not on {args.map!r} right now. It can only '
                 'plan a path for zones on its currently selected map -- '
                 'switch it first, and make sure it is localized.')

    zones = api.get(f"/robot/map/{chosen['id']}/zones") or []
    cleanable = [z for z in zones
                 if str(z.get('type', '')).lower() in ('clean', 'cleaning', '')]
    wanted = ([z.strip() for z in args.zones.split(',')] if args.zones
              else [z.get('name') for z in cleanable])
    targets = [z for z in cleanable if z.get('name') in wanted]
    if not targets:
        sys.exit(f'No matching clean zones on {args.map!r}. '
                 f'Found: {[z.get("name") for z in cleanable]}')
    print(f"\nZones to plan: {', '.join(z['name'] for z in targets)}")

    status = RobotChannel(api.prefix.split('/')[2], WS_STATUS, api.token,
                          'robotstatus')
    pose = RobotChannel(api.prefix.split('/')[2], WS_POSE, api.token,
                        'robotpose')
    print('\nConnecting...')
    status.connect()
    pose.connect()

    robot_id = args.robot
    results: dict[str, list[float]] = {}
    try:
        for channel in (status, pose):
            channel.send({'operation_cmd': 'subscribe', 'robot_id': robot_id})
        time.sleep(2.0)

        # A lost robot cannot plan anything, and telling one to start cleaning
        # is how it ends up somewhere nobody expects. Check before commanding.
        state = wait_for_status(status, robot_id)
        if state is None:
            sys.exit('No touchscreen status from the robot; refusing to '
                     'command it blind.')
        if state.get('localized') is not True:
            codes = ', '.join(
                c.get('description', '?')
                for c in (state.get('response_codes') or [])) or 'none'
            sys.exit(f"Robot reports localized={state.get('localized')!r} "
                     f'(status {state.get("status")!r}, codes: {codes}).\n'
                     'Localize it on the touchscreen first -- planning a zone '
                     'needs a robot that knows where it is.')
        print(f"  robot is localized, status {state.get('status')!r}, "
              f"battery {state.get('battery_soc')}%")

        status.send({'operation_cmd': 'request_path', 'robot_id': robot_id,
                     'content': {'status': True}})

        for zone in targets:
            name = zone['name']
            print(f'\n--- {name} ---')
            pose.drain()
            status.send({
                'operation_cmd': 'clean_start',
                'robot_id': robot_id,
                'content': {'zones': [{'name': name, 'path_id': 0,
                                       'boundary_mode': False}]},
            })
            print('  clean_start sent; waiting for the planned path...')
            path = collect_path(pose, robot_id, args.settle_seconds,
                                args.timeout_seconds)
            status.send({'operation_cmd': 'clean_stop', 'robot_id': robot_id,
                         'content': {'status': True}})
            if path:
                results[name] = path
                print(f'  captured {len(path) // 2} point(s); clean_stop sent')
            else:
                print('  ! no path received; clean_stop sent', file=sys.stderr)
            time.sleep(3.0)
    finally:
        # Whatever happened, do not leave the robot cleaning.
        try:
            status.send({'operation_cmd': 'clean_stop', 'robot_id': robot_id,
                         'content': {'status': True}})
            status.send({'operation_cmd': 'request_path', 'robot_id': robot_id,
                         'content': {'status': False}})
        except Exception:  # noqa: BLE001 - best effort on the way out
            pass
        status.close()
        pose.close()

    if not results:
        sys.exit('\nNo paths captured. The robot must be on this map, '
                 'localized and idle for it to plan anything.')

    out = Path(args.out)
    out.write_text(json.dumps(results, indent=2))
    print(f'\nWrote {out} with {len(results)} zone path(s).\n'
          f'Feed it to the generator:\n'
          f'  bootstrap_site.py ... --zone-paths {out}')


if __name__ == '__main__':
    main()
