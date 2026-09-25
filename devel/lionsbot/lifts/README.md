# Lifts

How a lift gets into this stack, and what RMF needs to know about it.

## The pieces

```
fleet adapter  --adapter_lift_requests-->  lift supervisor  --lift_requests-->  lift adapter
                                                   ^                                 |
                                                   +----------- lift_states ---------+
                                                                      |
                                                                 api-server --> dashboard
```

| Piece | Where it runs | Source |
| --- | --- | --- |
| lift events on lanes | derived by the fleet adapter from the nav graph | `parse_graph.cpp` |
| lift supervisor | `rmf-core` | [`../rmf-core.launch.xml`](../rmf-core.launch.xml) |
| lift adapter | `lift-adapter-1` | [`lift_adapter_template`](https://github.com/MinatoNami/lift_adapter_template/tree/feat/mock-lift), `feat/mock-lift` branch |
| lift geometry for the UI | `building_map_server` in `rmf-core` | `rmf_<site>.building.yaml` |

Only the lift adapter is yours to write. The supervisor grants one session at a
time and is what turns `adapter_lift_requests` into the `lift_requests` the
adapter subscribes to — without it the adapter sees nothing at all.

## The contract the adapter has to satisfy

A fleet adapter considers a lift request done when, and only when, all three of
these hold in one `lift_states` message (`RequestLift.cpp:324`):

```
current_floor == the requested floor
door_state    == LiftState.DOOR_OPEN
session_id    == the requester's id
```

So `MockLiftAPI` never reports `current_floor` as the destination until the
cabin has actually arrived, and holds the doors open while stopped. A lift that
claims to be at the destination with the doors still moving will make RMF drive
a robot into a closed door.

Two things fall out of that:

- **AGV mode.** RMF's default (`REQUEST_AGV_MODE`) means the doors are held open
  whenever the cabin is stopped. The adapter never has to act on the request's
  `door_state`; that field only matters in `REQUEST_HUMAN_MODE`.
- **`REQUEST_END_SESSION` is not a move, and carries no floor.** The fleet
  adapter sends it with `destination_floor` left empty
  (`RobotContext.cpp:1704`) and the supervisor forwards it unchanged. The
  template's floor-availability check drops an empty floor, so a stock adapter
  ignores the release and only lets go one round trip later, when the
  supervisor notices the orphaned session and re-sends the release with the
  cabin's current floor filled in (`Node.cpp:136`) — which also re-commands the
  lift to the floor it is already on. Handling `REQUEST_END_SESSION` before the
  floor check makes the release land on the first message, and both shapes of
  the message work.

## Configuring the floors

Three files have to agree on the floor names, or the request is rejected before
it reaches the lift:

1. `mock_lift.yaml` → `mock.floors` (becomes `available_floors`)
2. `rmf_<site>.building.yaml` → the keys under `levels:`
3. the nav graph → the levels that hold lift waypoints

`LIFT_1_NAME` in `.env` is the fourth name to keep in step: it must equal the key
under `lifts:` in both the building map and the nav graph, and the value of the
`lift:` property on the cabin waypoints.

## Configuring the lift waypoints

A lift waypoint is an ordinary waypoint that carries `lift: <LiftName>`. There
is one per level the lift serves, they all sit at the **same x, y** (the cabin
does not move horizontally), and they differ only in which level they belong to.

The fleet adapter reads the nav graph and attaches events to lanes purely from
that property (`parse_graph.cpp:286`):

| Lane | Event attached |
| --- | --- |
| outside → cabin | `LiftSessionBegin` — calls the lift and waits for it |
| cabin → cabin (same level) | nothing |
| cabin → outside | `LiftDoorOpen` on entry, `LiftSessionEnd` on exit |

You do not write those events. You write the `lift:` property and the lanes, and
they are generated.

### Nav graph (`maps/<site>/0.yaml`)

```yaml
building_name: rmf_mysite
doors: {}
lifts:
  lift_1:
    # Cabin centre in RMF metres, and its yaw in radians.
    position: [25.5, -17.2, 0.0]
    # Cabin footprint in metres: [width, depth].
    dims: [1.5, 1.5]
levels:
  L8:
    vertices:
      - [25.5, -17.2, {name: lift_1_L8, lift: lift_1}]   # inside the cabin
      - [23.9, -17.2, {name: lift_1_L8_lobby}]           # on the landing
      - [20.0, -14.0, {name: cleaning_zone, is_cleaning_zone: true}]
    lanes:
      - [1, 0, {}]    # lobby -> cabin   : LiftSessionBegin
      - [0, 1, {}]    # cabin -> lobby   : LiftDoorOpen + LiftSessionEnd
      - [2, 1, {}]
      - [1, 2, {}]
  L9:
    vertices:
      - [25.5, -17.2, {name: lift_1_L9, lift: lift_1}]   # same x, y as L8
      - [23.9, -17.2, {name: lift_1_L9_lobby}]
    lanes:
      - [1, 0, {}]
      - [0, 1, {}]
```

Vertex indices are per level and start at 0 on each one.

The `lifts:` block is not optional once any vertex names a lift: the fleet
adapter throws `Lift properties for [lift_1] were not provided` at startup if
the name is missing from it. Note that these two are **not** the same units as
the building map below — the nav graph is in metres, the building map in pixels.

### Building map (`maps/<site>/rmf_<site>.building.yaml`)

This one is for the humans and the dashboard. rmf-web builds its lift list from
`/building_map` (`repositories/rmf.py:69`), so a lift missing here has no card in
the UI even though RMF drives it perfectly.

```yaml
levels:
  L8:
    drawing: {filename: rmf_mysite.png}
    elevation: 0
    # ... vertices, lanes, walls as generated
  L9:
    drawing: {filename: rmf_mysite_l9.png}
    elevation: 4.0
    # ...
lifts:
  lift_1:
    x: 490          # pixels in the level's reference image
    y: 330
    yaw: 0
    width: 1.5      # metres
    depth: 1.5
    initial_floor_name: L8
    lowest_floor: L8
    highest_floor: L9
    reference_floor_name: L8
    level_doors:
      L8: [lift_1_door]
      L9: [lift_1_door]
    doors:
      lift_1_door:
        door_type: 2        # 2 = double sliding
        motion_axis_orientation: 0
        width: 1.2
        x: 0                # relative to the cabin centre
        y: -0.75
    plugins: false          # true only when Gazebo should simulate the cabin
```

Every level named in `level_doors`, `lowest_floor` and `highest_floor` must
exist under `levels:` in the same file. `building_map_tools` looks their
elevation up directly (`lift.py:186`) and dies with a `KeyError` on one that is
missing, taking `building_map_server` with it. On a single-level site, name only
that level and omit `lowest_floor`/`highest_floor` — the floors RMF actually
uses come from the adapter's `available_floors`, not from here, so the lift can
serve L9 with only L8 in the building map.

`x`/`y` are pixels because the generated building map is
`coordinate_system: reference_image`; multiply by the level's scale to get the
metres that go into the nav graph's `position`. The scale for this site is in
`config_<fleet>.yaml` under `map_transform.<level>.transform_values.scale`.

### Where the site files live

They are in the **fleet adapter repo**, not this one —
`$LIONSBOT_ADAPTER_PATH/maps/<site>/`. With `--maps`, `bootstrap_site.py`
writes every floor and the lift into them (see
[below](#the-two-level-office_new-site)). Without it you get a single level and
`lifts: {}`, and a lift is a hand edit on top, or a pass through the
[rmf_site editor](https://open-rmf.github.io/rmf_site/).

## Running it

```bash
docker compose --profile lift up -d --build
```

```bash
docker compose logs -f lift-adapter-1
```

A healthy start looks like this — the mock logs both halves of every exchange,
so the request and the response the adapter would make against a real lift are
visible without one:

```
[WARN] Running against a MOCK lift [mock_lift_1] with floors ['L8', 'L9'], starting at L8.
[INFO] Running LiftAdapterTemplate
[INFO] --> POST /lift/mock_lift_1/command {"floor": "L9"}
[INFO] <-- 200 {"accepted": true, "restarted": true, "from_floor": "L8", "to_floor": "L9", "eta_seconds": 8.0}
[INFO] Requested lift to L9.
[INFO] Session tinyRobot/robot_1 released lift lift_1.
```

Set `log_polls: true` in `mock_lift.yaml` to see the 2 Hz state queries too.

## Driving it by hand

You do not need a robot, a task, or even a nav graph to test the adapter. Watch
the state:

```bash
docker compose exec rmf-core bash -lc 'source /ros_entrypoint.sh && ros2 topic echo /lift_states'
```

And send a request the way the supervisor would:

```bash
docker compose exec rmf-core bash -lc 'source /ros_entrypoint.sh && ros2 topic pub --once /adapter_lift_requests rmf_lift_msgs/msg/LiftRequest "{lift_name: lift_1, session_id: manual_test, request_type: 1, destination_floor: L9, door_state: 2}"'
```

Publish to `adapter_lift_requests`, not `lift_requests`: going through the
supervisor is what exercises the session handling, and it is the only publisher
the adapter should ever have. Release it when you are done — with no
`destination_floor`, which is the shape a real fleet adapter sends:

```bash
docker compose exec rmf-core bash -lc 'source /ros_entrypoint.sh && ros2 topic pub --once /adapter_lift_requests rmf_lift_msgs/msg/LiftRequest "{lift_name: lift_1, session_id: manual_test, request_type: 0}"'
```

`session_id` goes back to `''` in the next `lift_states` message. If it does
not, the lift stays locked and every other robot gets `Lift is currently busy
with another request`.

The dashboard's lift card is at <http://localhost:3001/dashboard>, and the REST
view is `GET /lifts` and `GET /lifts/lift_1/state` (see the token recipe in the
[main README](../README.md#auth)).

## The two-level office_new site

Two real LionsBot maps, one per floor, joined by `lift_1`:

| level | vendor map | waypoints |
| --- | --- | --- |
| L8 | `office_new` | `docking_L8` (charger), `lift_waiting_L8`, `lift_L8`, `cleaning_zone_L8` |
| L9 | `office_L9` | `lift_waiting_L9`, `lift_L9`, `cleaning_L9` |

Generated by [`../bootstrap_site.py`](../bootstrap_site.py):

```bash
uv run --with pyyaml python3 bootstrap_site.py --robot R3-2200888-SCR \
  --site office_new --fleet r3 --maps office_new=L8,office_L9=L9 \
  --scale 0.0519913
```

`--maps` takes the floors in order and the **first is the reference level**.
Waypoint names get a `_<level>` suffix, because RMF resolves waypoints by name
across the whole building and both floors have a marker called `lift`. A marker
already ending in the suffix keeps its name, so `lift_waiting_L9` does not
become `lift_waiting_L9_L9`.

### How the floors are tied together

Each floor is its own SLAM map with its own pixel grid and its own origin, so
they have to be put in one frame — a lift has a single position shared by every
level it serves.

The generator uses the **lift marker as the anchor**: the shaft is physically
the same place on every floor, so translating each level until its `lift`
marker lands on the reference level's puts them all in a common frame. That is
why `--lift-marker` is required on every floor, and why the run prints

```
L9: shifted (+4.471, -0.624) m to put its lift on the reference lift
```

The same shift is written three times, in three different conventions, and all
three must agree:

| file | field | value |
| --- | --- | --- |
| nav graph | vertex coordinates | already shifted, in metres |
| building map | `fiducials` per level | synthetic, see below |
| fleet config | `map_transform.<level>.tx/ty_meters` | `(+4.471, +0.624)` |

`ty_meters` is **negated** relative to the shift. `MapTransform.robot_to_rmf_meters`
feeds the transform `(px, -py)` and constructs it as `Transform(.., tx, -ty_meters)`,
so `rmf_y = -scale*py - ty_meters`. Shifting a level by +dy metres needs
`ty_meters = -dy`. `tx` has no such flip. Getting this wrong puts the robot on
that floor off by twice the offset.

### Non-reference levels need fiducials

Only the reference level takes its scale from its own `measurements`. Every
other level is aligned through **fiducials** — shared named points — and
`Transform.set_from_fiducials` derives scale and rotation from the bearings and
distances *between pairs*, so it needs **at least two**. Given none it returns
early and the level silently keeps a scale of 1.0, publishing its whole graph
in pixels:

```
L8: walls x 0.0..38.5    y -35.4..0.0      <- metres
L9: walls x 0.0..740.0   y -680.0..0.0     <- pixels, scale never applied
```

Nothing errors. The visible symptom is a level that opens absurdly zoomed in
whose camera buttons appear dead. Check it with `/building_map`, not by eye:

```bash
curl -s -H "Authorization: Bearer $TOKEN" http://localhost:8000/building_map \
  | python3 -c "import json,sys; [print(l['name'], [round(v['x'],1) for v in l['wall_graph']['vertices']]) for l in json.load(sys.stdin)['levels']]"
```

Rather than trust hand-placed fiducials, the generator synthesises three per
level at fixed metre offsets from the lift anchor, converted through that
level's own scale (`FIDUCIAL_OFFSETS_M`). `set_from_fiducials` then recovers
exactly that scale and exactly the translation that makes the anchors coincide
— no dependence on anyone placing matching points by hand.

### Scale is still the hard part

Neither map can derive a scale today. A zone's position and the
metres-per-pixel both come solely from the coverage path of a **completed
cleaning job**, and the only job report on this account is for `cZone_349`, a
zone that no longer exists. `--scale 0.0519913` above is L8's old derived value
applied to both floors, which is sound only because both grids come from the
same robot with the same SLAM resolution.

Until that is fixed both `cleaning_zone_L8` and `cleaning_L9` sit at
**placeholder ring positions** -- 3 m from the mean of the real markers, at 45
degrees. Not approximately wrong; unrelated to the zone.

There is no endpoint that will just tell you. `GET /robot/map/{id}/zones`
returns `area, distance, name, time, type, zoneId` and nothing else, the
equalizer-config endpoint is cleaning parameters, and `clean_start` takes a
zone by NAME -- the caller is never expected to know where it is. Zone geometry
exists in exactly two places: a completed job report's `plannedPath`, and the
robot's live navigation path.

[`../derive_zone_paths.py`](../derive_zone_paths.py) takes the second route, so
a zone that has never been cleaned still gets a real position:

```bash
uv run --with pyyaml --with websocket-client python3 derive_zone_paths.py \
  --robot R3-2200888-SCR --map office_L9 --zones cleaning_L9 --yes
```

```bash
uv run --with pyyaml python3 bootstrap_site.py --robot R3-2200888-SCR \
  --site office_new --fleet r3 --maps office_new=L8,office_L9=L9 \
  --zone-paths zone_paths.json
```

It asks the robot to plan the zone and captures the path it streams:

```
-> robotstatus   {"operation_cmd": "request_path", "content": {"status": true}}
-> robotstatus   {"operation_cmd": "clean_start",  "content": {"zones": [...]}}
<- robotpose     {"operation_fb": "robot_path", "content": {"path": [x1,y1,...]}}
-> robotstatus   {"operation_cmd": "clean_stop",   "content": {"status": true}}
```

The path is in map pixels, the same frame job reports use, so it feeds the
existing geometry code unchanged -- real centroid, real convex hull in
`dock_summary.yaml`, and a **derived scale**, which removes the `--scale`
assumption for any floor it covers.

**It commands a real robot.** It starts a cleaning run so the robot plans, then
stops it; `--yes` is required, `clean_stop` is sent from a `finally` block, and
it refuses up front if the robot is on a different map or reports
`localized != true`. Dropping `--scale` once every zone has a path is what gets
each floor its own measured metres-per-pixel.

### Or place a marker at the zone

Markers **do** carry coordinates, and the robot cleans by zone name rather than
by position, so a marker named after a zone can supply the position the API
will not:

1. On the robot's touchscreen, drop a localization point in the middle of the
   cleaning zone.
2. Name it after the zone: either exactly (`cleaning_L9`), or with one of the
   suffixes `_loc`, `_point`, `_marker` or `_wp` (`cleaning_L9_point`) for
   setups that will not let a marker and a zone share a name.
3. Re-run the generator. It reports what it matched, zone first:

```
2 zone(s) positioned from a marker: cleaning_L9 <- cleaning_L9_point, cleaning_zone <- cleaning_zone
```

The matched marker is folded into the zone rather than also becoming a waypoint
of its own.

The waypoint keeps `is_cleaning_zone` and `robot_zone_name`, so dispatch is
unaffected; only its coordinate changes, from the ring placeholder to where you
put the marker. No robot motion and no cleaning job needed.

The limitation is that a marker is a point, not an outline, so
`dock_summary.yaml` still reserves a single point rather than the real
footprint, and the zone still cannot contribute a derived scale. For the
footprint and the scale you need a path -- either a completed job or
`derive_zone_paths.py`.

### Running the scenario

[`../dispatch.py`](../dispatch.py) dispatches without the dashboard and prints
the robot and the lift together:

```bash
python3 dispatch.py status
```

Ride up to L9, which is only reachable through the cabin:

```bash
python3 dispatch.py go lift_waiting_L9
```

Then clean a zone back down on L8. The planner sees the only route is through
`lift_1` and calls it:

```bash
python3 dispatch.py clean cleaning_zone_L8
```

The plan RMF builds is `lift_waiting_L9 → lift_L9 → lift_L8 → lift_waiting_L8 →
cleaning_zone_L8`. The `lift_L9 → lift_L8` step is the ride itself: both ends are inside
the same cabin, so it carries no event — RMF has already opened the session on
the way in and closes it on the way out.

### What had to change in the fleet adapter

`LionsbotRobot._prepare_map` treated any RMF level change as a robot map
switch. It now compares the resolved robot map name and level first and, when
they match, just updates the RMF level and returns.

That short-circuit does **not** fire on this site any more: L8 and L9 are
genuinely different vendor maps, so riding the lift really does switch the
robot's map and hot-localize it on the far floor. The guard still matters for
the case it was written for — several RMF levels sharing one robot map — and it
keeps a redundant `change_map` off the wire, which is worth having because that
call has been seen to answer 500.

Expect a lift ride to take noticeably longer now, and to fail if the robot
cannot localize on the destination floor's map.

`navigate()` also logs `destination.map` and `destination.inside_lift`, without
which a ride is three indistinguishable navigate calls in the log.

Nothing else was needed: EasyFullControl runs the lift session itself, so the
adapter only has to accept a destination on another level.

## Swapping in the real lift

`MockLiftAPI` and `LiftAPI` expose the same six methods, and the adapter picks
between them on `mock.enabled`. Porting means filling in the `IMPLEMENT YOUR
CODE HERE` blocks in `LiftAPI.py` and setting `mock.enabled: false`. The mock is
written as a REST client on purpose — `_request` is the only method that has to
become `requests.get` / `requests.post`.

Keep the mock working after that. It is the only way to exercise a floor
transition without a building.

## Caveats for this site

- **Lift rides need the adapter's `feat/lift-support-and-map-reconciliation`
  branch.** Older adapter branches never read `Destination::inside_lift()` or
  switch the robot's map mid-task, so RMF calls and holds the lift while the
  robot fails to ride it. On that branch the adapter watches `lift_states`, and
  when the cabin stops on another floor it switches the robot's map and
  hot-localizes it there. So far this has been exercised with the simulated
  lift only.
- **Every floor needs a `lift` marker.** `bootstrap_site.py --maps` aligns the
  levels on it and exits if a floor has none. Each floor is still its own SLAM
  map with its own scale, bridged to its RMF level by `robot_maps` in the fleet
  config.
- The production path runs the supervisor as its own pod
  (`charts/rmf-deployment/templates/rmf-core-modules.yaml`); the lift adapter
  needs an equivalent pod there, modelled on this compose service.
