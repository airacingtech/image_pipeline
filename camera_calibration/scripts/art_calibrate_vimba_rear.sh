#!/usr/bin/env bash
set -Eeuo pipefail

CAMERA="vimba_rear"
TOPIC="/vimba_rear/image"
RESULT_ROOT="${ART_CALIBRATION_ROOT:-/home/autera-admin/ART/camera_calibration_sessions}"
SESSION="${RESULT_ROOT}/$(date +%Y%m%d_%H%M%S_%N)/${CAMERA}"

pause_on_exit() {
  local return_code=$?
  if (( return_code == 0 )); then
    printf '\n标定任务已结束。结果：%s\n' "${SESSION}"
  else
    printf '\n标定任务失败或被中止（退出码 %d）。数据：%s\n' "${return_code}" "${SESSION}" >&2
  fi
  if [[ -t 0 ]]; then
    read -r -p '按 Enter 关闭终端...' _
  fi
  exit "${return_code}"
}
trap pause_on_exit EXIT

mkdir -p "${SESSION}"
exec > >(tee -a "${SESSION}/launcher.log") 2>&1

source /opt/ros/jazzy/setup.bash
source /home/autera-admin/ART/race_common/install/setup.bash
source /home/autera-admin/ART/image_pipeline/install/setup.bash
export PYTHONUNBUFFERED=1
export DISPLAY="${DISPLAY:-:0}"
if [[ -z "${XAUTHORITY:-}" && -r /run/user/1000/gdm/Xauthority ]]; then
  export XAUTHORITY=/run/user/1000/gdm/Xauthority
fi

topic_info="$(timeout 15 ros2 topic info "${TOPIC}" 2>&1)" || {
  printf '无法读取 topic：%s\n%s\n' "${TOPIC}" "${topic_info}" >&2
  exit 2
}
if ! grep -Eq 'Publisher count: [1-9][0-9]*' <<<"${topic_info}"; then
  printf 'topic 没有 publisher：%s\n%s\n请先启动既有相机系统；本脚本不会启动 Vimba。\n' \
    "${TOPIC}" "${topic_info}" >&2
  exit 3
fi

printf '开始 %s fisheye 自动标定。\n结果目录：%s\n' "${CAMERA}" "${SESSION}"
ros2 run camera_calibration art_camera_calibrator "${CAMERA}" \
  --image-topic "${TOPIC}" \
  --auto-save "${SESSION}/calibrationdata.tar.gz" \
  --auto-progress "${SESSION}/progress.json" \
  --auto-exit
