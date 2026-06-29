#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

DOOSAN_SETUP=""
for candidate in \
    /root/ros2_ws/doosan_ws/install/setup.bash \
    /root/ros2_ws/doosanrobot_humble/install/setup.bash
do
    if [[ -f "${candidate}" ]]; then
        DOOSAN_SETUP="${candidate}"
        break
    fi
done

# 이미지마다 워크스페이스 디렉터리 이름이 다른 경우 dsr_msgs2를 기준으로 찾는다.
if [[ -z "${DOOSAN_SETUP}" ]]; then
    while IFS= read -r candidate; do
        install_dir="$(dirname "${candidate}")"
        if [[ -d "${install_dir}/dsr_msgs2" ]]; then
            DOOSAN_SETUP="${candidate}"
            break
        fi
    done < <(find /root/ros2_ws -maxdepth 4 -path '*/install/setup.bash' -type f 2>/dev/null)
fi

if [[ -z "${DOOSAN_SETUP}" ]]; then
    echo "[ERROR] Doosan 워크스페이스의 install/setup.bash를 찾지 못했습니다." >&2
    exit 1
fi
source "${DOOSAN_SETUP}"

DOOSAN_INSTALL="$(dirname "${DOOSAN_SETUP}")"
DSR_IMP="${DOOSAN_INSTALL}/dsr_common2/lib/dsr_common2/imp"
if [[ -d "${DSR_IMP}" ]]; then
    export PYTHONPATH="${PYTHONPATH:+${PYTHONPATH}:}${DSR_IMP}"
fi

ROBOT_WS=/root/ros2_ws/auto-dump-bot/code/robot_ws
cd "${ROBOT_WS}"

# 개발 중 bind mount된 최신 소스를 컨테이너 시작 시 다시 빌드한다.
if [[ "${BUILD_ON_START:-1}" == "1" ]]; then
    echo "[INFO] auto_dump_robot_pkg colcon build 시작"
    colcon build --symlink-install
fi

if [[ -f "${ROBOT_WS}/install/setup.bash" ]]; then
    source "${ROBOT_WS}/install/setup.bash"
fi

exec "$@"
