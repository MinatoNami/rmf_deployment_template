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
| `rmf-core` | template `rmf` | host | building map server, traffic schedule, task dispatcher, lift supervisor, trajectory server (`:8006`) |
| `api-server` | template `api-server` | host | rmf-web REST/websocket API (`:8000`) |
| `dashboard` | template `dashboard-no-auth` | bridge | web UI, published on `DASHBOARD_PORT` |
| `fleet-adapter-1` | built here | host | LionsBot adapter for `FLEET_1_CONFIG` |
| `fleet-adapter-2` | built here | host | second fleet, behind the `fleet2` profile |
| `lift-adapter-1` | built here | host | lift adapter for `LIFT_1_NAME`, behind the `lift` profile |

Everything ROS-facing runs on the host network namespace, so DDS discovery needs no
configuration. `rmf-core` runs entirely out of the stock `rmf` image — the site's map
and nav graphs are bind-mounted from the adapter repo at `/site`, so editing a map or
a fleet config only needs a service restart, not a rebuild.

The adapter repo's own `docker/docker-compose.yaml` starts the adapter alone. It does
not provide `rmf-core`, the api-server or the dashboard, and it hard-requires an
`RMF_WEB_API_TOKEN` you supply yourself. That is what this directory adds.

## Prerequisites

- Docker with Compose v2 (Docker Desktop on macOS: turn on host networking, see
  [Notes on Apple Silicon](#notes-on-apple-silicon))
- `git`, and Python 3.10+
- [`uv`](https://docs.astral.sh/uv/) for the helper scripts that need PyYAML or
  websocket-client (`bootstrap_site.py`, `derive_zone_paths.py`)
- LionsBot cloud credentials (`api.lionsbot.io`) and a robot registered on that account

## Getting the code

The stack builds two other repos into its images, and by default expects them
checked out **beside** this one:

```
Open-rmf/                              any parent directory
├── rmf_deployment_template/           this repo
│   └── devel/lionsbot/                you are here; run every command from here
├── fleet_adapter_lionsbot/            LIONSBOT_ADAPTER_PATH → .../fleet_adapter
└── lift_adapter_template/             LIFT_ADAPTER_PATH → .../lift_adapter_template
```

```bash
git clone -b feat/lionsbot-local-stack https://github.com/MinatoNami/rmf_deployment_template.git
```

```bash
git clone -b feat/lift-support-and-map-reconciliation https://github.com/MinatoNami/fleet_adapter_lionsbot.git
```

```bash
git clone -b feat/mock-lift https://github.com/MinatoNami/lift_adapter_template.git
```

The lift adapter is only needed for `--profile lift`. If you put the repos
somewhere else, set `LIONSBOT_ADAPTER_PATH` and `LIFT_ADAPTER_PATH` in `.env`
to the directory holding each one's `package.xml`.

Every command below assumes the working directory is `devel/lionsbot`:

```bash
cd rmf_deployment_template/devel/lionsbot
```

## Quick start

For a site whose maps and configs already exist in the adapter repo (the
shipped `office_new` site, for example):

1. `cp .env.example .env`, then set `LIONSBOT_USER` and `LIONSBOT_PASSWORD`.
2. Check the site selection in `.env` (`FOLDER_NAME`, `FLOOR_NAME`,
   `FLEET_1_CONFIG`, `FLEET_1_NAV_GRAPH`) names files that exist under
   `fleet_adapter_lionsbot/fleet_adapter/{configs,maps}/<FOLDER_NAME>/`.
3. `docker compose up -d --build` (add `--profile lift` for the simulated lift).
   The first build pulls the template images and takes several minutes.
4. Open <http://localhost:3001/dashboard> and `docker compose logs -f fleet-adapter-1`.
   See [step 5](#5-start-and-verify) for what a healthy start looks like.

For a new site, work through [Setting up a new site](#setting-up-a-new-site) first.

## Tools in this directory

| File | Touches the robot? | What it does |
| --- | --- | --- |
| [`bootstrap_site.py`](bootstrap_site.py) | read-only | generates a site's fleet config, nav graph, dock summary and building map from the LionsBot API, one or several floors |
| [`derive_zone_paths.py`](derive_zone_paths.py) | **moves it** (`--yes`) | captures a zone's planned path so a never-cleaned zone gets real geometry; see [`lifts/README.md`](lifts/README.md#how-the-floors-are-tied-together) |
| [`sweep_heading.py`](sweep_heading.py) | **re-localizes it** (`--yes`) | scores hot-localize across headings to find the right one; see [Localization](#localization) |
| [`dispatch.py`](dispatch.py) | via RMF | dispatches `go` / `clean` tasks and prints status without the dashboard |
| [`lifts/`](lifts/) | — | lift adapter config and the [lift guide](lifts/README.md) |
| `start-adapter.sh`, `start-lift-adapter.sh` | — | container entrypoints; not run by hand |

Every script takes `--help`. The ones that log in read `LIONSBOT_USER` /
`LIONSBOT_PASSWORD` from the environment and prompt for the password otherwise.

## Which adapter branch to use

Check out the adapter's `feat/lift-support-and-map-reconciliation` branch
([fork](https://github.com/MinatoNami/fleet_adapter_lionsbot/tree/feat/lift-support-and-map-reconciliation)).
It contains everything this stack relies on: the hyphenated-robot-ID fix below,
lift rides between floors, and following a robot that was relocalised by hand.
Its README has a step-by-step quick start for this stack.

### Hyphenated robot IDs (already fixed on that branch)

Older adapter branches need this fix. RMF composes a per-robot ROS topic,
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

To add a lift — simulated by default, so it needs no hardware:

```bash
docker compose --profile lift up -d --build
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

Find the robot's encoding ID:

```bash
uv run --with pyyaml bootstrap_site.py --list-robots
```

Then generate the site. For a single floor (the robot's currently selected map):

```bash
uv run --with pyyaml bootstrap_site.py --robot R3-2200888-SCR --site mysite --fleet r3
```

For several floors joined by a lift, list the vendor maps in order as
`NAME=LEVEL`; the first is the reference level, and every floor needs a
`lift` marker (see [`lifts/README.md`](lifts/README.md#the-two-level-office_new-site)):

```bash
uv run --with pyyaml bootstrap_site.py --robot R3-2200888-SCR --site mysite --fleet r3 \
  --maps office_new=L8,office_L9=L9
```

It reads `LIONSBOT_USER` / `LIONSBOT_PASSWORD` from the environment (or `--user`,
and prompts for the password), and writes into the adapter repo — by default
`../../../fleet_adapter_lionsbot/fleet_adapter` relative to the script, or
`--output`:

```
configs/<site>/config_<fleet>.yaml          fleet + robot config
maps/<site>/0.yaml                          nav graph (--nav-graph to rename)
maps/<site>/dock_summary.yaml               cleaning zone footprints
maps/<site>/rmf_<site>.building.yaml        floor plan for the dashboard
maps/<site>/rmf_<site>.png                  the robot's occupancy grid
                                            (rmf_<site>_<level>.png per floor with --maps)
```

Options worth knowing (`--help` lists them all):

| Option | Use |
| --- | --- |
| `--map-level L9` | single-floor mode, but pick the vendor map by its level instead of the current one |
| `--scale 0.052` or `--scale L8=0.052,L9=0.051` | fix metres-per-pixel; skips job-report derivation for those levels, so omit it whenever a cleaning job exists |
| `--zone-paths zone_paths.json` | merge paths captured by `derive_zone_paths.py`, for zones never cleaned |
| `--start-level L8` | floor the robot stands on when the adapter starts (default: the charger's floor) |
| `--lift-name`, `--lift-marker`, `--landing-marker`, `--lift-dims` | lift naming and cabin size; defaults `lift_1`, `lift`, `lift_waiting`, `1.5x1.5` |

### 4. Point `.env` at it

The generator prints these four values when it finishes:

```
FOLDER_NAME=mysite
FLOOR_NAME=L8            # the first (reference) level
FLEET_1_CONFIG=config_r3.yaml
FLEET_1_NAV_GRAPH=0.yaml
```

`FOLDER_NAME` also determines the building filename (`rmf_<site>.building.yaml`), so it
is not free-form. If the site has a lift, also check `LIFT_1_NAME` matches
`--lift-name` and that `floors` in [`lifts/mock_lift.yaml`](lifts/mock_lift.yaml)
lists every level.

### 5. Start and verify

```bash
docker compose up -d --build
```

A robot that is already localized on a map listed in the fleet config's `robot_maps`
keeps that map and starts on the matching level. Only an unlocalized robot is switched
to the `start` map and hot-localized there. Three lines in `docker compose logs -f fleet-adapter-1` say it worked:

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

## Lifts

[`lifts/README.md`](lifts/README.md) covers the whole subject: the contract a
lift adapter has to satisfy, how to configure lift waypoints in the nav graph
and the building map, how to drive the lift by hand without a robot, and what
the simulated lift does.

The short version. `rmf-core` runs the lift supervisor, which turns a fleet's
`adapter_lift_requests` into the `lift_requests` a lift adapter subscribes to.
`lift-adapter-1` runs [`lift_adapter_template`](https://github.com/MinatoNami/lift_adapter_template/tree/feat/mock-lift)
against [`lifts/mock_lift.yaml`](lifts/mock_lift.yaml), which by default drives a
lift that exists only inside the adapter process and logs both halves of every
exchange:

```
--> POST /lift/mock_lift_1/command {"floor": "L9"}
<-- 200 {"accepted": true, "restarted": true, "from_floor": "L8", "to_floor": "L9", "eta_seconds": 8.0}
```

Set `mock.enabled: false` and fill in `LiftAPI.py` to talk to a real lift; the
adapter picks between the two implementations and nothing else changes.

The simulated lift, the `--mock` switch and the session-release fix are on the
`feat/mock-lift` branch of [the fork](https://github.com/MinatoNami/lift_adapter_template/tree/feat/mock-lift), not upstream. Upstream
`lift_adapter_template` has none of them and exits at startup with `Failed
initilize lift status`, so point `LIFT_ADAPTER_PATH` at a checkout of that branch.

`bootstrap_site.py --maps` generates every floor, aligns them on the `lift`
marker and writes the `lifts:` block into both the nav graph and the building
map (see [`lifts/README.md`](lifts/README.md#the-two-level-office_new-site)).
Without `--maps` it generates only the robot's current map, and a lift is a
hand edit on top. The fleet adapter rides lifts on its
`feat/lift-support-and-map-reconciliation` branch: when the cabin stops on another
floor it switches the robot's map and hot-localizes it there (see *Multi-level sites
and lifts* in the adapter README).

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

One unit trap, and it only bites in one place: marker `angle` from the REST API is
**radians** and is copied straight into `localization_starting_point.heading`, and
hot-localize takes radians too (the API docs say so explicitly, and the websocket
`HotLocalizeCommand` repeats it). Only pose feedback over the `robotpose` websocket is
**degrees**, which is why the adapter converts that one and nothing else.

## Localization

Hot-localize is a **confirmation, not a teleport**. The LionsCloud guide ("Tier 2
Feature: Localization") spells the procedure out:

1. Create a localization point on the map with the touchscreen map editor.
2. Physically push the robot onto it, as accurately as possible.
3. Only then call `PUT /robot/command/hot-localize/{robotId}` with that point's
   coordinates.

So a failing hot-localize is usually not a bad heading — it is a robot that is not
standing where the call says it is, or a target that was never a localization point in
the first place. `GET /robot/map/{mapId}/markers/locpoint` returns *only* real
localization points; the generic `/markers` endpoint also returns POIs and dock points,
and a POI is not a valid target however good its coordinates look.

The call answers `{"success": bool, "percentage": int}`, where percentage is the
robot's confidence in the match. That makes heading measurable rather than guessable,
which is what [`sweep_heading.py`](sweep_heading.py) is for:

```bash
# read-only: what localization points exist on the L9 map?
python3 sweep_heading.py --robot R3-2200888-SCR --map office_L9 --list

# walk the heading around the circle and score each one
python3 sweep_heading.py --robot R3-2200888-SCR --map office_L9 \
    --point lift_waiting --steps 24 --yes
```

Read the result like this:

| Shape | Means |
| --- | --- |
| one clear peak | the heading was wrong; take the argmax |
| low everywhere | wrong x/y or map, or the robot is not on the point |
| `400 RobotStateNotRight` | nothing to do with heading; the robot refused the command |

It prints a `localization_starting_point` block ready to paste into the fleet config,
and re-applies the best heading on the way out so the robot is not left believing
whatever the last step said. `--yes` is required because every step overwrites the pose
estimate; `--list` commands nothing. The account is limited to 100 requests per minute,
so `--delay` is clamped to keep a sweep inside that.

Two things the adapter does not do, worth knowing when reading its logs:

- **It never sends `accuracy`.** The request body accepts one and every marker carries
  one; `sweep_heading.py` defaults to the point's own value and `--accuracy -1` omits
  the field, so the difference can be measured.
- **It sweeps only after a lift ride.** The adapter logs every hot-localize with its
  percentage or `errorCode` (`hot-localize ... :` lines), and after a lift ride it
  sweeps the heading itself when the derived one scores low. The startup localize
  still uses `localization_starting_point` as given, so measure that heading with
  `sweep_heading.py`.

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

**Adapter crash-loops with `InvalidTopicNameError`** — the adapter checkout predates the
hyphen fix; switch to the branch named above. Each loop also re-sends a change-map command to the robot, so stop the service
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

**Lift adapter exits with `Timed out waiting for /rmf_lift_supervisor`** —
`rmf-core` was started before the supervisor was added to
[`rmf-core.launch.xml`](rmf-core.launch.xml). Restart it with
`docker compose up -d --force-recreate rmf-core`.

**`GET /lifts` returns `[]` but `GET /lifts/<name>/state` works** — rmf-web
builds its lift list from the building map, not from the state stream. The lift
is missing from the `lifts:` block of `rmf_<site>.building.yaml`; RMF drives it
correctly either way, it just has no card in the dashboard.

**A second level opens absurdly zoomed in and its camera buttons do nothing** —
that level has no fiducials, so it kept a scale of 1.0 and is published in
pixels while the reference level is in metres. Nothing errors. See
[`lifts/README.md`](lifts/README.md#cloning-a-level-needs-fiducials-not-just-a-copy).

**Dashboard map goes blank after restarting `rmf-core` or `api-server`** — the
three.js camera keeps its old position while the scene is rebuilt, so the floor
plan ends up off screen. The canvas is still there and `/building_map` is fine;
click the fit-to-view button (the top icon on the map's left rail). This is a
dashboard quirk, not a broken map — check `/building_map` before chasing it as
one.

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
