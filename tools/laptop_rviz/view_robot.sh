#!/bin/bash
# RViz on this laptop, subscribed to the robot at ROS domain 26.
set -euo pipefail
dir=$(cd "$(dirname "$0")" && pwd)
shopt -s nullglob
setups=(/opt/ros/*/setup.bash)
if [[ ${#setups[@]} -eq 0 ]]; then
  echo "В /opt/ros нет ROS 2. Установите дистрибутив и пакеты rviz2 и rmw-cyclonedds-cpp." >&2
  exit 1
fi
chosen=${setups[0]}
for candidate in "${setups[@]}"; do
  [[ $candidate == */humble/* ]] && chosen=$candidate
done
# shellcheck disable=SC1090
source "$chosen"
distro=$(basename "$(dirname "$(dirname "$chosen")")")
if ! ros2 pkg prefix rviz2 >/dev/null 2>&1 || [[ -z $(find "/opt/ros/$distro" -name 'librmw_cyclonedds_cpp.so' -print -quit) ]]; then
  echo "Не хватает пакетов. Выполните:" >&2
  echo "sudo apt-get install -y ros-${distro}-rviz2 ros-${distro}-rmw-cyclonedds-cpp" >&2
  exit 1
fi
export ROS_DOMAIN_ID=26
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${dir}/cyclonedds.xml"
exec ros2 launch "$dir/view_robot.launch.py"
