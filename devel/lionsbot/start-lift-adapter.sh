#!/bin/bash
# Entrypoint for one lift adapter.
#
# The config is read from /lifts, a read-only bind mount of ./lifts, so a floor
# list or a travel time can be changed with a restart rather than a rebuild.
set -euo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# ROS setup scripts reference unset variables; source them with nounset off.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/rmf/setup.bash
source /opt/lift_adapter/setup.bash
set -u

: "${LIFT_NAME:?LIFT_NAME is required, and must match the key under \`lifts:\` in the building map}"

LIFT_CONFIG="${LIFT_CONFIG:-mock_lift.yaml}"
CONFIG_FILE="/lifts/${LIFT_CONFIG}"

if [[ ! -f "$CONFIG_FILE" ]]; then
  echo "Not found: $CONFIG_FILE" >&2
  echo "LIFT_CONFIG names a file in devel/lionsbot/lifts/." >&2
  exit 1
fi

# The adapter exits if its first state query fails, and a real lift API is
# usually reachable before rmf-core is. Waiting on the supervisor instead keeps
# the two ordered: until it is up, `lift_requests` has no publisher and the
# adapter would publish states nobody consumes.
SUPERVISOR_NODE="${SUPERVISOR_NODE:-/rmf_lift_supervisor}"
SUPERVISOR_TIMEOUT="${SUPERVISOR_TIMEOUT:-120}"
echo "Waiting up to ${SUPERVISOR_TIMEOUT}s for ${SUPERVISOR_NODE}..."
# A deadline rather than an attempt count: each `ros2 node list` spends a
# second or more on discovery, so counting attempts overshoots the timeout.
deadline=$((SECONDS + SUPERVISOR_TIMEOUT))
until ros2 node list 2>/dev/null | grep -qx "$SUPERVISOR_NODE"; do
  if (( SECONDS >= deadline )); then
    echo "Timed out waiting for ${SUPERVISOR_NODE}." >&2
    echo "rmf-core may not be up, or DDS discovery is not reaching it." >&2
    exit 1
  fi
  sleep 1
done
echo "Found ${SUPERVISOR_NODE}."

# Credentials for a real lift API belong in the environment, not in the config
# file. Expand them in memory and hand the result to the adapter on stdin, so
# the expanded credentials are never written to disk. A variable that is not
# set would otherwise be left in the value as literal `$NAME`, so fail instead.
EXPANDED_CONFIG="$(python3 -c '
import os
import re
import sys
import yaml

UNEXPANDED = re.compile(r"\$(\w+|\{\w+\})")
missing = set()

def expand(value):
    if isinstance(value, dict):
        return {key: expand(child) for key, child in value.items()}
    if isinstance(value, list):
        return [expand(item) for item in value]
    if isinstance(value, str):
        expanded = os.path.expandvars(value)
        missing.update(name.strip("{}")
                       for name in UNEXPANDED.findall(expanded))
        return expanded
    return value

with open(sys.argv[1]) as source_file:
    config = expand(yaml.safe_load(source_file))
if missing:
    names = ", ".join(sorted(missing))
    sys.exit(f"{sys.argv[1]} references unset environment variable(s): "
             f"{names}")
yaml.safe_dump(config, sys.stdout, sort_keys=False)
' "$CONFIG_FILE")"

MOCK_ARGS=()
if [[ "${LIFT_MOCK:-false}" == "true" ]]; then
  MOCK_ARGS+=(--mock)
fi

echo "Starting lift adapter: ${LIFT_NAME} (${LIFT_CONFIG})"
exec ros2 run lift_adapter_template lift_adapter_template \
  -n "$LIFT_NAME" \
  -c /dev/stdin \
  ${MOCK_ARGS[@]+"${MOCK_ARGS[@]}"} \
  --ros-args -p use_sim_time:="${USE_SIM_TIME:-false}" \
  <<< "$EXPANDED_CONFIG"
