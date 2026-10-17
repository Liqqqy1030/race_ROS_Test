#!/usr/bin/env bash
set -e
SOURCE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname -- "$SOURCE_DIR")"
if [[ ! -f "$WORKSPACE_DIR/devel/setup.bash" ]]; then
  echo "Workspace is not built. Run catkin_make in $WORKSPACE_DIR first." >&2
  exit 1
fi
source "$WORKSPACE_DIR/devel/setup.bash"
if [[ -f "$WORKSPACE_DIR/competition_env.sh" ]]; then
  source "$WORKSPACE_DIR/competition_env.sh"
fi
AI_PYTHON="${AI_PYTHON:-$WORKSPACE_DIR/.venv_ai/bin/python3}"
if [[ ! -x "$AI_PYTHON" ]]; then
  echo "Independent AI Python not found: $AI_PYTHON" >&2
  echo "Follow 00_总流程_只看这一份.md first." >&2
  exit 1
fi
if command -v python >/dev/null && python -c 'import rospy' >/dev/null 2>&1; then
  ROS_PYTHON="$(command -v python)"
elif command -v python2 >/dev/null && python2 -c 'import rospy' >/dev/null 2>&1; then
  ROS_PYTHON="$(command -v python2)"
elif command -v python3 >/dev/null && python3 -c 'import rospy' >/dev/null 2>&1; then
  ROS_PYTHON="$(command -v python3)"
else
  echo "Cannot find the ROS-native Python interpreter with rospy." >&2
  exit 1
fi
export AI_PYTHON ROS_PYTHON
"$ROS_PYTHON" -c 'import rospy; from TTS_audio.srv import StringService'
env -u PYTHONPATH -u PYTHONHOME "$AI_PYTHON" -c 'import cv2,numpy,requests,websocket,pyaudio'
: "${VOLC_TTS_APPID:?Set VOLC_TTS_APPID first}"
: "${VOLC_TTS_TOKEN:?Set VOLC_TTS_TOKEN first}"
: "${ARK_API_KEY:?Set ARK_API_KEY first}"
command -v mplayer >/dev/null || { echo "mplayer is not installed" >&2; exit 1; }
exec roslaunch robot_slam patrol_all.launch "$@"
