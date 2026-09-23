# Local Open-RMF with the LionsBot fleet adapter

Runs Open-RMF locally against a **LionsBot site** with the
[LionsBot fleet adapter](https://github.com/lionsbot-official/fleet_adapter_lionsbot)
talking to `api.lionsbot.io`. No Gazebo, no Keycloak, no Kubernetes.

This is a different world from [`../docker-compose-local.yaml`](../docker-compose-local.yaml),
which runs the `rmf_demos` office map with simulated `tinyRobot`s. The two are not
interchangeable — waypoint names, level names and the `map_transform` scale all differ,
so a LionsBot adapter cannot be pointed at the demo simulation.

## Layout

| Service | Image | Network | What it does |
| --- | --- | --- | --- |
| `rmf-core` | template `rmf` | host | building map server, traffic schedule, task dispatcher, trajectory server (`:8006`) |
| `api-server` | template `api-server` | host | rmf-web REST/websocket API (`:8000`) |
| `dashboard` | template `dashboard-no-auth` | bridge | web UI, published on `DASHBOARD_PORT` |
| `fleet-adapter-1` | built here | host | LionsBot adapter for `FLEET_1_CONFIG` |
| `fleet-adapter-2` | built here | host | second fleet, behind the `fleet2` profile |

Everything ROS-facing runs on the host network namespace, so DDS discovery needs no
configuration. `rmf-core` runs entirely out of the stock `rmf` image — the site's map
and nav graphs are bind-mounted from the adapter repo at `/site`, so editing a map or
a fleet config only needs a service restart, not a rebuild.

The adapter repo's own `docker/docker-compose.yaml` starts the adapter alone. It does
not provide `rmf-core`, the api-server or the dashboard, and it hard-requires an
`RMF_WEB_API_TOKEN` you supply yourself. That is what this directory adds.

## Prerequisites

- Docker with Compose v2
- A local clone of `fleet_adapter_lionsbot` (default: a sibling of this repo)
- LionsBot cloud credentials and a robot registered on that account
- `uv` (only for `bootstrap_site.py`, which needs PyYAML)

## Required: patch the adapter for hyphenated robot IDs

**Apply this before the first run.** RMF composes a per-robot ROS topic,
`rmf/dynamic_event/begin/<fleet>/<robot>`, from the robot name. ROS 2 topic segments
allow only alphanumerics and underscores, but LionsBot encoding IDs contain hyphens
(`R3-2200888-SCR`), and the adapter uses the encoding ID as the RMF robot name. The
result is an immediate abort and a crash loop:

```
terminate called after throwing an instance of 'rclcpp::exceptions::InvalidTopicNameError'
  what():  Invalid topic name: 'rmf/dynamic_event/begin/r3/R3-2200888-SCR'
                                                            ^
```

The fix adds `rmf_robot_name()` to `fleet_adapter/utils/robot_config.py` and uses it at
the three places where the name crosses into RMF (`fleet_adapter.py`: known-robot
registration, `get_known_robot_configuration`, and `add_robot`), plus an `rmf_name`
field on `LionsbotRobot`. RMF then sees `R3_2200888_SCR` while every LionsBot API call
still uses `R3-2200888-SCR`. Set `robot_config.rmf_name` to choose the name yourself.

This is not specific to building RMF from `main`: `dynamic_event` landed in `rmf_ros2`
on 2025-04-28 (PR #410) and is present in every release from 2.11.0 onward, so the apt
packages need the patch too.

## Run a site that is already set up

```bash
cp .env.example .env
```

Fill in `LIONSBOT_USER` and `LIONSBOT_PASSWORD` (the file is gitignored), then:

```bash
docker compose up -d --build
```

Open <http://localhost:3001/dashboard>. To add the second fleet:

```bash
docker compose --profile fleet2 up -d
```

Tear down with `docker compose down`.

## Setting up a new site

### 1. On the robot's touchscreen

SLAM the floor, then place **a localization point** (without one there is no
`localization_starting_point` and the adapter cannot localize) and **the cleaning
zones** (without them there is nothing to dispatch).

Do not go looking for a dock-point tool. Every marker observed so far comes back as
`type: loc_point` whatever you name it, and `/markers/dock` has been empty on every
map. The generator falls back to the home point as the charger waypoint and warns.

### 2. Run one cleaning job in every zone

This step looks optional and is not. The zone endpoint returns no geometry, so the
coverage path in the job report is the only source of two things: the metres-per-pixel
scale, and where each zone actually is. Without a completed job the generator exits
rather than guess.

On one real site the difference was:

| | guessed `--scale 0.05` | derived from one job |
| --- | --- | --- |
| scale | 0.05 | 0.051991 (4% out, ≈1.5 m at the far edge of a 37 m map) |
| zone waypoint | placeholder ring, invented | real path centroid |
| `dock_summary` footprint | 1 point | 25-point convex hull |

Then wait for the map to sync. It needs `synced: true` **and**
`equalizersSyncStatus: SUCCESS` — these are independent, and a map can be synced while
its cleaning configs are not.

### 3. Generate the site files

```bash
uv run --with pyyaml devel/lionsbot/bootstrap_site.py --user you@example.com --list-robots
```

```bash
uv run --with pyyaml devel/lionsbot/bootstrap_site.py --robot R3-2200888-SCR --site mysite --fleet r3
```

It prompts for the password, or reads `LIONSBOT_USER` / `LIONSBOT_PASSWORD` from the
environment, and writes into the adapter repo:

```
configs/<site>/config_<fleet>.yaml    fleet + robot config
maps/<site>/0.yaml                    nav graph
maps/<site>/dock_summary.yaml         cleaning zone footprints
maps/<site>/rmf_<site>.building.yaml  floor plan for the dashboard
maps/<site>/rmf_<site>.png            the robot's occupancy grid
```

Omit `--scale` so the scale is derived; passing a value silently skips the job report.
It defaults to the robot's currently selected map — use `--map-name` for another. If
the RMF level name must differ from the vendor's, pass `--level` and a `robot_maps`
bridge is written, which the adapter supports natively.

### 4. Point `.env` at it

The generator prints these four values when it finishes:

```
FOLDER_NAME=mysite
FLOOR_NAME=L8            # must match --level exactly
FLEET_1_CONFIG=config_r3.yaml
FLEET_1_NAV_GRAPH=0.yaml
```

`FOLDER_NAME` also determines the building filename (`rmf_<site>.building.yaml`), so it
is not free-form.

### 5. Start and verify

```bash
docker compose up -d --build
```

The adapter changes the robot's selected map on startup, so a robot on a different map
will be switched. Three lines in `docker compose logs -f fleet-adapter-1` say it worked:

- `Advertised clean zones: [...]` — empty means dispatch is dead, see Troubleshooting
- `Successfully added robot [...]`
- a restart count of 0 (`docker inspect rmf-lionsbot-fleet-adapter-1-1 --format '{{.RestartCount}}'`)

Then check the dashboard. If the robot is offset the scale is wrong; if it is rotated,
the floor plan is not the robot's own grid image.

### 6. Draw the lanes

The generated nav graph joins every waypoint to a single hub in straight lines,
ignoring walls. It is enough to prove the stack end to end and must be replaced before
real traffic — LionsBot has no lane concept, so nothing can derive this.

The [rmf_site web build](https://open-rmf.github.io/rmf_site/) needs no install and
reads `.building.yaml` via its legacy importer. For a handful of waypoints, editing
`0.yaml` by hand is quicker.

### Multiple robots

Map **once** and point the other robots at that map. If each robot SLAMs the same floor
separately you get N maps, N transforms and N single-robot fleets, because
`map_transform` follows the robot's own map rather than the building. Getting a shared
map onto a second robot is a touchscreen sync on that robot: the API can only select a
map a robot already holds (`PUT /robot/{robotId}/map/{mapId}`), not push one to it.

## How coordinates work

The generator uses the robot's own grid PNG as the RMF floor plan, so the two share a
pixel frame and the transform reduces to a single scale:

```
rmf_x =  pixel_x * scale
rmf_y = -pixel_y * scale
```

That is why generated configs have `rotation_degrees: 0` and no translation — correct
by construction, not a placeholder. A config with a real rotation and offset (as in the
older `config_r3scp.yaml`) means someone aligned the robot's map against a separate
architectural drawing instead.

Two unit traps: marker `angle` from the REST API is **radians** and is copied straight
into `localization_starting_point.heading`, while pose feedback over the websocket is
**degrees** and the adapter converts it.

## Notes on Apple Silicon

The template's images are published for `linux/amd64` only, so on an arm64 Mac they run
under Docker Desktop's emulation. That works, and the `platform` mismatch warning at
startup is expected. Two things follow:

- Do not pass `--platform linux/amd64` to `docker pull` for these images; the registry
  serves a single-arch manifest and the flag makes the pull fail with `denied`.
- Docker Desktop's host networking must be on (Settings → Resources → Network). With it
  enabled, ports bound inside host-network containers are reachable from macOS
  `localhost`, including ones bound to `127.0.0.1`.

## Auth

The `api-server` runs on its stock `default_config`, which verifies JWTs with the
upstream stub secret (`rmfisawesome`, audience `rmf_api_server`, issuer `stub`). The
adapter entrypoint mints a matching token for alert posting unless `RMF_WEB_API_TOKEN`
is set. This is a local development arrangement only — the production path is Keycloak
via `charts/rmf-deployment`.

To call the API by hand:

```bash
TOKEN=$(python3 -c '
import base64, hashlib, hmac, json, time
seg = lambda r: base64.urlsafe_b64encode(r).rstrip(b"=")
now = int(time.time())
head = seg(json.dumps({"alg": "HS256", "typ": "JWT"}, separators=(",", ":")).encode())
body = seg(json.dumps({"iss": "stub", "aud": "rmf_api_server",
                       "preferred_username": "admin",
                       "iat": now, "exp": now + 3600},
                      separators=(",", ":")).encode())
signed = head + b"." + body
print((signed + b"." + seg(hmac.new(b"rmfisawesome", signed, hashlib.sha256).digest())).decode())
')
curl -s -H "Authorization: Bearer $TOKEN" http://localhost:8000/fleets
```

## Troubleshooting

**`docker compose` refuses to start and never mentions your email** — the credential
key in `.env` must be `LIONSBOT_USER`. `docker-compose.yaml` uses
`${LIONSBOT_USER:?...}`, so a key named `LIONSBOT_EMAIL` fails with a message about a
missing variable.

**Adapter crash-loops with `InvalidTopicNameError`** — the hyphen patch above is not
applied. Each loop also re-sends a change-map command to the robot, so stop the service
while you fix it.

**`Advertised clean zones: []`** — `dock_summary.yaml` is empty for this fleet. The
advertised list is built from that file alone (`fleet_adapter.py`, `make_dock_paths`),
not from the nav graph, so an empty dock summary blocks cleaning dispatch entirely.
Re-run the generator after a cleaning job has completed.

**Dashboard loads but the map is blank** — two separate causes:

1. The building has no walls. The dashboard frames the view from the wall graph, so
   `walls: []` renders an empty canvas with no floor plan, waypoints or robot even when
   those layers are enabled. `bootstrap_site.py` emits a perimeter wall to prevent this;
   check `wall_graph.vertices` is non-zero in `/building_map`.
2. `rmf-core` publishes `building_map`, which [`rmf-core.launch.xml`](rmf-core.launch.xml)
   remaps to `map` because that is what rmf-web's api-server subscribes to. Confirm with
   `curl -H "Authorization: Bearer $TOKEN" http://localhost:8000/building_map`.

**The dashboard shows an older map** — `rmf-core` reads `building.yaml` once at startup.
Restart `rmf-core` and `api-server` after regenerating.

**Robot appears in the wrong place** — the scale is wrong; re-run without `--scale` once
a cleaning job exists. If it is rotated instead, the floor plan is not the robot's grid.

**Robot never appears, no obvious error** — the key under `robots:` must be the
`robotEncodingId`; it is passed verbatim as `robot_encoding_id` to every API call.

**`Timed out waiting for /rmf_traffic_schedule`** — `rmf-core` is not up, or DDS is not
reaching it. Check `docker compose logs rmf-core`, and that both services share
`ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION`.

**`401 Unauthorized` from `api.lionsbot.io`** — the credentials in `.env` are wrong or
still the placeholders. Everything else in the stack is independent of this; the fleet
still registers with rmf-web, just with no robots.

**Port already in use** — `DASHBOARD_PORT` defaults to 3001. The api-server (`8000`) and
trajectory server (`8006`) ports are fixed by the images and cannot be remapped, because
host-network containers do not publish ports.

## Why the adapter image is built on the template's `rmf` image

The adapter repo's own `docker/Dockerfile` starts from `ros:jazzy` and installs RMF from
apt. This template builds RMF from source at `main`. Mixing the two puts different
`rmf_internal_msgs` / `rmf_api_msgs` definitions on either side of the same DDS domain,
which shows up as silent non-communication. [`Dockerfile.fleet-adapter`](Dockerfile.fleet-adapter)
therefore layers the adapter package onto the same `rmf` image `rmf-core` uses.

Switching to the adapter's apt-based image does not avoid the hyphen patch — see that
section above.
