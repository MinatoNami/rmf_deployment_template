#!/bin/bash
# Entrypoint for one LionsBot fleet adapter.
#
# Mirrors the adapter repo's docker/start_adapter.sh, but reads configs and maps
# from /site (a read-only bind mount of the adapter repo) so they can be edited
# without rebuilding the image.
set -euo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# ROS setup scripts reference unset variables; source them with nounset off.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/rmf/setup.bash
source /opt/lionsbot/setup.bash
set -u

: "${LIONSBOT_USER:?LIONSBOT_USER is required}"
: "${LIONSBOT_PASSWORD:?LIONSBOT_PASSWORD is required}"
: "${SERVER_URI:?SERVER_URI is required}"

FOLDER_NAME="${FOLDER_NAME:-office_new}"
FLEET_CONFIG="${FLEET_CONFIG:-config_r5.yaml}"
NAV_GRAPH="${NAV_GRAPH:-0.yaml}"

CONFIG_DIR="/site/configs/${FOLDER_NAME}"
MAP_DIR="/site/maps/${FOLDER_NAME}"
CONFIG_FILE="${CONFIG_DIR}/${FLEET_CONFIG}"
NAV_GRAPH_FILE="${MAP_DIR}/${NAV_GRAPH}"
DOCK_SUMMARY_FILE="${MAP_DIR}/dock_summary.yaml"

for f in "$CONFIG_FILE" "$NAV_GRAPH_FILE" "$DOCK_SUMMARY_FILE"; do
  if [[ ! -f "$f" ]]; then
    echo "Not found: $f" >&2
    echo "Check LIONSBOT_ADAPTER_PATH, FOLDER_NAME, FLEET_CONFIG and NAV_GRAPH." >&2
    exit 1
  fi
done

# The adapter posts alerts to rmf-web. In this no-auth local stack the api-server
# runs on its default config, which signs with the upstream stub secret, so mint a
# matching short-lived token unless one was supplied.
if [[ -z "${RMF_WEB_API_TOKEN:-}" ]]; then
  RMF_WEB_API_TOKEN="$(python3 - <<'PY'
import base64, hashlib, hmac, json, os, time

def seg(raw):
    return base64.urlsafe_b64encode(raw).rstrip(b"=")

now = int(time.time())
secret = os.environ.get("RMF_WEB_JWT_SECRET", "rmfisawesome")
header = seg(json.dumps({"alg": "HS256", "typ": "JWT"}, separators=(",", ":")).encode())
payload = seg(json.dumps({
    "iss": "stub",
    "aud": "rmf_api_server",
    "preferred_username": "admin",
    "iat": now,
    "exp": now + 7 * 24 * 3600,
}, separators=(",", ":")).encode())
signing_input = header + b"." + payload
signature = seg(hmac.new(secret.encode(), signing_input, hashlib.sha256).digest())
print((signing_input + b"." + signature).decode())
PY
)"
  export RMF_WEB_API_TOKEN
  echo "Minted a local RMF_WEB_API_TOKEN for alerts (valid 7 days)."
fi

# Adapter.make() needs the schedule node up before it starts.
SCHEDULE_NODE="${SCHEDULE_NODE:-/rmf_traffic_schedule}"
SCHEDULE_TIMEOUT="${SCHEDULE_TIMEOUT:-120}"
echo "Waiting up to ${SCHEDULE_TIMEOUT}s for ${SCHEDULE_NODE}..."
for ((attempt = 1; attempt <= SCHEDULE_TIMEOUT; attempt++)); do
  if ros2 node list 2>/dev/null | grep -qx "$SCHEDULE_NODE"; then
    echo "Found ${SCHEDULE_NODE}."
    break
  fi
  if (( attempt == SCHEDULE_TIMEOUT )); then
    echo "Timed out waiting for ${SCHEDULE_NODE}." >&2
    echo "rmf-core may not be up, or DDS discovery is not reaching it." >&2
    exit 1
  fi
  sleep 1
done

# Credentials live in the config as $LIONSBOT_USER / $LIONSBOT_PASSWORD; expand
# them into a temp copy rather than editing the mounted (read-only) config.
EXPANDED_CONFIG="$(mktemp /tmp/fleet_adapter_config.XXXXXX.yaml)"
trap 'rm -f "$EXPANDED_CONFIG"' EXIT

python3 -c '
import os
import sys
import yaml

source_path, target_path = sys.argv[1], sys.argv[2]

def expand(value):
    if isinstance(value, dict):
        return {key: expand(child) for key, child in value.items()}
    if isinstance(value, list):
        return [expand(item) for item in value]
    if isinstance(value, str):
        return os.path.expandvars(value)
    return value

with open(source_path) as source_file:
    config = expand(yaml.safe_load(source_file))
with open(target_path, "w") as target_file:
    yaml.safe_dump(config, target_file, sort_keys=False)
' "$CONFIG_FILE" "$EXPANDED_CONFIG"

echo "Starting fleet adapter: ${FLEET_CONFIG} on ${NAV_GRAPH} (${FOLDER_NAME})"
exec ros2 run fleet_adapter fleet_adapter \
  -c "$EXPANDED_CONFIG" \
  -n "$NAV_GRAPH_FILE" \
  -d "$DOCK_SUMMARY_FILE" \
  --server_uri "$SERVER_URI" \
  --ros-args -p use_sim_time:="${USE_SIM_TIME:-false}"
