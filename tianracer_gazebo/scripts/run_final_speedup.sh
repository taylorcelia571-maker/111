#!/usr/bin/env bash
# One isolated experiment, automatic evidence folder/bag; never calls start.
set -euo pipefail
version="${1:-baseline}"
if (($#)); then shift; fi
case "$version" in baseline|v1|v2|v3) ;; *) echo "Use baseline, v1, v2 or v3" >&2; exit 2;; esac
declare -A seen=()
budget_trial=false
launch_overrides=()
for argument in "$@"; do
  key="${argument%%:=*}"
  case "$key" in
    startup_speed|startup_acceleration|startup_connector_length|steering_preview_time|steering_feedback_gain|heading_feedback_gain|cross_track_gain|acceleration|lateral_accel|braking|yaw_rate|line_search_level|predict_pose|route_control_scoring|speed_recovery_hold|legacy_control|runtime_path_cache|steering_preview_mode|route_joint_polish|route_polish_strength|route_speed_refine|route_speed_iterations|budget_trial) ;;
    *) echo "Unsupported/protected override: $argument" >&2; exit 2;;
  esac
  if [[ "$argument" != *":="* || -n "${seen[$key]:-}" ]]; then
    echo "Use each parameter once as name:=value: $argument" >&2; exit 2
  fi
  seen[$key]=1
  if [[ "$key" == budget_trial ]]; then
    case "${argument#*:=}" in true|false) budget_trial="${argument#*:=}";; *) echo 'budget_trial must be true or false' >&2; exit 2;; esac
  else
    launch_overrides+=("$argument")
  fi
done
for argument in "${launch_overrides[@]}"; do
  key="${argument%%:=*}"
  if [[ "$key" == lateral_accel || "$key" == braking ]]; then
    limit=9.94
    if [[ "$key" == lateral_accel && "$budget_trial" == true ]]; then limit=10.2; fi
    if [[ "$key" == braking ]]; then limit=3.7; fi
    if ! python3 -c 'import sys,math;v=float(sys.argv[1]);limit=float(sys.argv[2]);sys.exit(0 if math.isfinite(v) and 0<v<=limit else 1)' "${argument#*:=}" "$limit"; then
      echo "$key exceeds the selected experiment bound ($limit); rejected." >&2
      exit 2
    fi
  fi
done
if [[ "$budget_trial" == true ]]; then
  echo 'EXPERIMENTAL lateral budget opt-in: <=10.2, not a measured safe range; braking stays <=3.7.' >&2
fi
set +u
source /opt/ros/noetic/setup.bash
race_workspace="${RACE_WORKSPACE:-$HOME/ican_race_ws}"
source "$race_workspace/devel/setup.bash"
set -u
package="$(rospack find tianracer_gazebo)"
run_dir="$HOME/ican_race_logs/final71/$(date +%Y%m%d_%H%M%S)_${version}_$$"
mkdir -p "$run_dir"
printf '%s\n' "$run_dir" > "$HOME/ican_race_logs/final_last_session.txt"
git -C "$package/.." rev-parse HEAD > "$run_dir/commit.txt"
python3 - "$run_dir/library_versions.json" <<'PY'
import json,sys
import numpy,scipy
with open(sys.argv[1], 'w') as stream:
    json.dump(dict(python=sys.version, numpy=numpy.__version__, scipy=scipy.__version__), stream, indent=2)
    stream.write('\n')
PY
printf '%s\n' "$budget_trial" > "$run_dir/budget_trial.txt"
if [[ -n "${RACE_TRIAL_RECIPE_FILE:-}" ]]; then
  cp -- "$RACE_TRIAL_RECIPE_FILE" "$run_dir/trial_recipe.json"
fi
command=(roslaunch tianracer_gazebo final_speedup_demo.launch
         "version:=$version" "log_dir:=$run_dir" "${launch_overrides[@]}")
printf '%q ' "${command[@]}" > "$run_dir/command.txt"
printf '\n' >> "$run_dir/command.txt"
topics=(/clock /tf /tf_static /tianracer/cmd_vel /tianracer/ackermann_cmd_stamped
        /tianracer/odom /tianracer/joint_states /tianracer/scan
        /tianracer/amcl_pose /tianracer/imu /rosout_agg)
rosbag record --buffsize=256 -O "$run_dir/control.bag" "${topics[@]}" > "$run_dir/record.log" 2>&1 &
bag_pid=$!
capture_pid=''
cleanup() {
  if [[ -n "$capture_pid" ]]; then kill "$capture_pid" 2>/dev/null || true; fi
  kill -INT "$bag_pid" 2>/dev/null || true
  wait "$bag_pid" 2>/dev/null || true
  echo "Evidence folder: $run_dir"
  echo "Analyze: rosrun tianracer_gazebo analyze_final_race.py --session '$run_dir' --bag '$run_dir/control.bag'"
}
trap cleanup EXIT
(
  for ((attempt=0; attempt<120; attempt++)); do
    if timeout 3 rosservice list 2>/dev/null | grep -qx /tianracer/continuous_race/status; then
      rosparam get /tianracer/continuous_race > "$run_dir/params.yaml"
      timeout 5 rosservice call /tianracer/continuous_race/status '{}' > "$run_dir/status_ready.txt" 2>&1 || true
      exit 0
    fi
    sleep 1
  done
) &
capture_pid=$!
echo "Experiment $version; manual prepare/status/start required."
echo "Evidence folder: $run_dir"
set +e
"${command[@]}" 2>&1 | tee "$run_dir/C.log"
launch_status=${PIPESTATUS[0]}
exit "$launch_status"
