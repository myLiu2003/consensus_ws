#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTO_WS="$(cd "$SCRIPT_DIR/../../.." && pwd)"
PROFILE_FILE="$AUTO_WS/config/workspace_profile.env"
if [[ -f "$PROFILE_FILE" ]]; then
    # shellcheck disable=SC1090
    source "$PROFILE_FILE"
fi

WS="${WS:-${MAP_CONSENSUS_WS:-$AUTO_WS}}"
export MAP_CONSENSUS_WS="$WS"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${WS}/install/setup.bash" ]]; then
    source "${WS}/install/setup.bash"
fi
set -u

if command -v micro-xrce-dds-agent >/dev/null 2>&1; then
    exec micro-xrce-dds-agent udp4 -p 8888
fi

if command -v MicroXRCEAgent >/dev/null 2>&1; then
    exec MicroXRCEAgent udp4 -p 8888
fi

echo "未找到 Micro XRCE-DDS Agent 可执行文件。" >&2
exit 1
