#!/bin/zsh
set -euo pipefail

script_dir=${0:A:h}
repo_dir=${script_dir:h:h}
build_dir=$(mktemp -d /tmp/rescue-rover-host.XXXXXX)
trap 'rm -rf "$build_dir"' EXIT

/usr/bin/clang++ -std=c++17 -Wall -Wextra \
  -I"$repo_dir/tests/host/include" -I"$repo_dir/include" \
  "$repo_dir/tests/host/host_fake.cpp" \
  "$repo_dir/src/sensors.cpp" \
  "$repo_dir/src/main.cpp" \
  "$repo_dir/tests/host/rover_host.cpp" \
  -o "$build_dir/rover_host"

typeset -a scenarios=(
  reversal_and_cancel
  reversal_completes_after_deadtime
  no_unnecessary_deadtime_and_pivot
  reversal_emergency_cancel
  reversal_disconnect_cancel
  reversal_timeout_cancel
  reversal_critical_safety_cancel
  event_priority_and_drain
  critical_beep_restarts_after_expiry
  simultaneous_rollover_and_tilt
  sound_and_pir
  sound_pir_obstacle_suppressed
  speed_ack_and_telemetry
  ultrasonic_failure_and_recovery
  dht_hysteresis
  tilt_hysteresis
  tilt_invalid_retention_and_rollover
)

overall_status=0
for scenario in $scenarios; do
  "$build_dir/rover_host" "$scenario" || overall_status=1
done
exit $overall_status
