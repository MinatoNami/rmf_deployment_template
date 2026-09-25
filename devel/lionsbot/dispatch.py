#!/usr/bin/env python3
"""Dispatch tasks to the local stack without a dashboard.

The api-server signs with the upstream stub secret in this no-auth setup, so a
token is minted here rather than asked for. See the Auth section of README.md.

    python3 dispatch.py go lift_waiting_L9        # ride the lift up to L9
    python3 dispatch.py clean cZone_349           # clean a zone, riding down
    python3 dispatch.py status                    # robots, levels, tasks
    python3 dispatch.py cancel <task_id>

`go` is an RMF go_to_place; `clean` is the compose request rmf_demos'
dispatch_clean.py builds, which is what the LionsBot adapter's `clean` task
capability answers to. Both let the planner choose the route -- if the only way
from here to there is through lift_1, RMF calls the lift on its own.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import hmac
import json
import os
import sys
import time
import urllib.error
import urllib.request

API = os.environ.get('RMF_SERVER_URL', 'http://localhost:8000')
SECRET = os.environ.get('RMF_WEB_JWT_SECRET', 'rmfisawesome')


def token() -> str:
    def seg(raw: bytes) -> bytes:
        return base64.urlsafe_b64encode(raw).rstrip(b'=')

    now = int(time.time())
    head = seg(json.dumps({'alg': 'HS256', 'typ': 'JWT'},
                          separators=(',', ':')).encode())
    body = seg(json.dumps({'iss': 'stub', 'aud': 'rmf_api_server',
                           'preferred_username': 'admin',
                           'iat': now, 'exp': now + 3600},
                          separators=(',', ':')).encode())
    signed = head + b'.' + body
    sig = seg(hmac.new(SECRET.encode(), signed, hashlib.sha256).digest())
    return (signed + b'.' + sig).decode()


def call(path: str, payload: dict | None = None):
    url = f'{API}{path}'
    data = json.dumps(payload).encode() if payload is not None else None
    req = urllib.request.Request(
        url, data=data, method='POST' if data else 'GET',
        headers={'Authorization': f'Bearer {token()}',
                 'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=30) as response:
            return json.load(response)
    except urllib.error.HTTPError as err:
        sys.exit(f'{err.code} from {path}: {err.read().decode()[:400]}')
    except urllib.error.URLError as err:
        sys.exit(f'Cannot reach {API}: {err.reason}')


def dispatch(request: dict, fleet: str | None) -> None:
    now_ms = int(time.time() * 1000)
    request['unix_millis_request_time'] = now_ms
    request['unix_millis_earliest_start_time'] = now_ms
    request['requester'] = 'dispatch.py'
    if fleet:
        request['fleet_name'] = fleet
    result = call('/tasks/dispatch_task',
                  {'type': 'dispatch_task_request', 'request': request})
    state = result.get('state') or {}
    if result.get('success') is False or not state:
        sys.exit(f'Dispatch refused: {json.dumps(result)[:500]}')
    assigned = state.get('assigned_to') or {}
    print(f"task {state.get('booking', {}).get('id')} "
          f"-> {state.get('status')} "
          f"fleet={assigned.get('group') or 'unassigned'} "
          f"robot={assigned.get('name') or '-'}")


def cmd_go(args) -> None:
    dispatch({
        'category': 'compose',
        'description': {
            'category': 'go_to_place',
            'phases': [{'activity': {
                'category': 'sequence',
                'description': {'activities': [{
                    'category': 'go_to_place',
                    'description': {'one_of': [{'waypoint': args.waypoint}]},
                }]},
            }}],
        },
    }, args.fleet)


def cmd_clean(args) -> None:
    activities = [
        {'category': 'go_to_place', 'description': args.zone},
        {'category': 'perform_action', 'description': {
            'unix_millis_action_duration_estimate': 60000,
            'category': 'clean',
            'expected_finish_location': args.zone,
            'description': {'zone': args.zone},
            'use_tool_sink': True,
        }},
    ]
    dispatch({
        'category': 'compose',
        'description': {
            'category': 'clean',
            'phases': [{'activity': {
                'category': 'sequence',
                'description': {'activities': activities},
            }}],
        },
    }, args.fleet)


def cmd_status(_args) -> None:
    for fleet in call('/fleets'):
        print(f"fleet {fleet['name']}")
        for name, robot in (fleet.get('robots') or {}).items():
            location = robot['location']
            print(f"  {name}: {robot['status']} level={location['map']} "
                  f"({location['x']:.2f}, {location['y']:.2f}) "
                  f"battery={robot['battery'] * 100:.0f}% "
                  f"task={robot['task_id'] or '-'}")
    for lift in call('/lifts'):
        state = call(f"/lifts/{lift['name']}/state")
        print(f"lift {lift['name']}: floor={state['current_floor']} "
              f"dest={state['destination_floor']!r} "
              f"door={state['door_state']} session={state['session_id']!r}")


def cmd_cancel(args) -> None:
    print(json.dumps(call('/tasks/cancel_task',
                          {'type': 'cancel_task_request',
                           'task_id': args.task_id}), indent=2))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fleet', default=None,
                        help='Pin the task to one fleet instead of bidding')
    sub = parser.add_subparsers(dest='command', required=True)

    go = sub.add_parser('go', help='go_to_place')
    go.add_argument('waypoint')
    go.set_defaults(func=cmd_go)

    clean = sub.add_parser('clean', help='clean a zone')
    clean.add_argument('zone')
    clean.set_defaults(func=cmd_clean)

    sub.add_parser('status', help='robots, levels and lift').set_defaults(
        func=cmd_status)

    cancel = sub.add_parser('cancel', help='cancel a task')
    cancel.add_argument('task_id')
    cancel.set_defaults(func=cmd_cancel)

    args = parser.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
