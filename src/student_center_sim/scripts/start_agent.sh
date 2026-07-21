#!/usr/bin/env bash
set -Eeuo pipefail

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${HOME}/consensus_ws/install/setup.bash" ]]; then
    source "${HOME}/consensus_ws/install/setup.bash"
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
