#!/usr/bin/env bash
# Бэг с реального Mid-360 -> карта (.pcd, .pgm + .yaml) и мир Gazebo.
#
#   tools/bag_to_world/bag_to_world.sh <бэг> <каталог> <имя> [x0,y0,x1,y1 ...]
#
# В бэге нужны /livox/lidar (PointCloud2 от livox_ros_driver2) и /livox/imu.
# Прямоугольники в конце -- что стереть из карты (координаты карты, см.
# build_map.py). Мир пишется в <каталог>/<имя>.world; чтобы запустить его в
# симуляторе, скопируйте в src/sim_kobuki/worlds/.
#
# FAST-LIO2 (fast_lio) и livox_ros_driver2 берутся из FASTLIO_SETUP.
set -euo pipefail

if [ $# -lt 3 ]; then
    sed -n 2,11p "$0"
    exit 1
fi

BAG=$(realpath "$1")
OUT=$(realpath -m "$2")
NAME=$3
shift 3

HERE=$(dirname "$(realpath "$0")")
FASTLIO_SETUP=${FASTLIO_SETUP:-$HOME/ws/localization_benchmark/ws/install/setup.bash}
WORK=$(mktemp -d)
trap 'kill $(jobs -p) 2>/dev/null || true; rm -rf "$WORK"' EXIT

set +u
source /opt/ros/humble/setup.bash
source "$FASTLIO_SETUP"
set -u
# отдельный домен: не мешать живым топикам и не ловить чужие
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_BAG:-77}

mkdir -p "$OUT"

echo "1/4 PointCloud2 -> CustomMsg"
python3 "$HERE/to_custom.py" "$BAG" "$WORK/custom"

echo "2/4 FAST-LIO2 (в реальном времени, сколько длится бэг)"
# бинарь напрямую, а не через ros2 run: тогда сигнал доходит до самой ноды
"$(ros2 pkg prefix fast_lio)/lib/fast_lio/fastlio_mapping" --ros-args \
    --params-file "$HERE/fastlio_mid360.yaml" -p use_sim_time:=true \
    > "$OUT/fastlio.log" 2>&1 &
sleep 3
ros2 bag record -s sqlite3 -o "$WORK/fastlio" /cloud_registered /Odometry \
    > "$WORK/record.log" 2>&1 &
RECORD=$!
sleep 3
ros2 bag play "$WORK/custom" --clock 200 > "$WORK/play.log" 2>&1
sleep 3
kill -INT $RECORD
wait $RECORD || true

kill -INT %1 2>/dev/null || true

echo "3/4 карта"
python3 "$HERE/build_map.py" "$WORK/fastlio" "$BAG" "$OUT" "$NAME" "$@"

echo "4/4 мир Gazebo"
python3 "$HERE/gen_world.py" "$OUT" "$NAME" "$NAME" "$OUT/$NAME.world"
