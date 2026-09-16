#!/bin/bash
# ROS 2 replacement for the ROS 1 `launch/run_lodestar_odom` script.
# No roscore is needed any more; RViz2 is started in the background and shut
# down again when the odometry run finishes.
set -euo pipefail

# Source the ROS 2 environment if the caller has not done so already.
if [ -z "${ROS_DISTRO:-}" ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$HOME/ros2_ws/install/setup.bash"
fi

########### Bagfile Data Name #############
SEQUENCE="${SEQUENCE:-test_sequence3}"
BAG_BASE_PATH="${BAG_BASE_PATH:-$HOME/ros2_data/odom_test}"

# A ROS 2 bag is a DIRECTORY containing metadata.yaml (or a single .mcap file).
BAG_FILE_PATH="${BAG_BASE_PATH}/${SEQUENCE}"
echo "${BAG_FILE_PATH}"

EVAL_BASE_DIR="${EVAL_BASE_DIR:-${BAG_BASE_PATH}/${SEQUENCE}_eval}"
est_dir="${EVAL_BASE_DIR}/"
mkdir -p "${est_dir}"

if [ ! -e "${BAG_FILE_PATH}" ]; then
  echo "ERROR: bag not found at ${BAG_FILE_PATH}" >&2
  echo "Convert a ROS 1 bag first, e.g.:" >&2
  echo "  rosbags-convert --src <sequence>.bag --dst ${BAG_FILE_PATH}" >&2
  exit 1
fi

#PARAMETERS for point normal matcher (CFEAR)
cost_type="P2P"
submap_scan_size="1"
registered_min_keyframe_dist="1.5"
res="3"
zmin="60"
weight_option="4"
weight_intensity="true"
range_resolution="0.05" # for better optimization performance, we first generate small pointcloud and recover the real resolution later.
soft_constraint="false"
disable_compensate="true"

#LodeStar Parameters
radar_topic="/radar_image_inrange"
contour_threshold="214" #If you want to use radar from pohang canal dataset, you should change source code(RCS Inversion required)
k_nearest="20"

pars=(
  --range-res "${range_resolution}"
  --sequence "${SEQUENCE}"
  --soft_constraint "${soft_constraint}"
  --disable_compensate "${disable_compensate}"
  --cost_type "${cost_type}"
  --submap_scan_size "${submap_scan_size}"
  --registered_min_keyframe_dist "${registered_min_keyframe_dist}"
  --res "${res}"
  --bag_path "${BAG_FILE_PATH}"
  --est_directory "${est_dir}"
  --job_nr 1
  --z-min "${zmin}"
  --weight_option "${weight_option}"
  --weight_intensity "${weight_intensity}"
  --dataset marine
  --radar_topic "${radar_topic}"
  --contour_threshold "${contour_threshold}"
  --k_nearest "${k_nearest}"
)

RVIZ_PID=""
cleanup() {
  if [ -n "${RVIZ_PID}" ] && kill -0 "${RVIZ_PID}" 2>/dev/null; then
    kill "${RVIZ_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

ros2 launch lodestar_odometry vis.launch.py &
RVIZ_PID=$!
sleep 2

ros2 run lodestar_odometry lodestar_odom "${pars[@]}"
