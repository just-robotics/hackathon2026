# hackathon2026

Стек автопилота Autoware / ROS 2 Humble и симуляция Kobuki с многослойным
лидаром в Gazebo Harmonic. Все компоненты запускаются в Docker через CLI `helm`
из [helm_launch/](helm_launch/).

## Требования

- Docker с плагином `docker compose` v2
- Python 3 и `pip`
- NVIDIA Container Toolkit для GPU-лидара
- X11, если нужен GUI Gazebo

## Установка

1. Склонировать репозиторий:

   ```bash
   git clone git@github.com:just-robotics/hackathon2026.git
   cd hackathon2026
   ```

2. Поставить `helm`. Команда зависит от версии дистрибутива:

   ```bash
   # Ubuntu 24.04, Debian 12+
   sudo pip install helm_launch/. --break-system-packages

   # Ubuntu 22.04
   sudo pip install helm_launch/.
   ```

   На 24.04 без `--break-system-packages` установка упирается в PEP 668
   (`error: externally-managed-environment`). На 22.04 наоборот: там pip 22.0.2,
   который такого флага ещё не знает, и с ним команда падает с
   `no such option: --break-system-packages`. Проверить свою версию — `pip --version`,
   флаг появился в pip 23.0.1.

3. Проверить:

   ```bash
   helm -h
   ```

4. Автодополнение по Tab:

   ```bash
   sudo activate-global-python-argcomplete # or activate-global-python-argcomplete3
   ```

## Сборка

Основной образ содержит Autoware, ROS 2 Humble, Gazebo Harmonic из репозитория
OSRF и пакет `sim_kobuki`:

```bash
helm build gazebo
```

Humble официально работает с Gazebo Fortress. Для Harmonic используется
предоставляемый OSRF пакет `ros-humble-ros-gzharmonic`; он не должен быть
установлен вместе с `ros-humble-ros-gz` для Fortress.

## Запуск Gazebo

Только симулятор:

```bash
helm up gazebo
helm flogs gazebo
helm down gazebo
```

Gazebo вместе с модулями автопилота:

```bash
helm up simulation
```

Окно симулятора открывается автоматически в обоих профилях. Compose передаёт в
контейнер `DISPLAY` и cookie X-сервера из `XAUTHORITY`, а процесс внутри идёт от
uid 1000, как и пользователь хоста, поэтому `xhost` не нужен.

Запуск без графики (например, на машине без дисплея) — через [.env](.env):

```bash
GAZEBO_HEADLESS=true
```

Либо разово, переменной окружения:

```bash
helm clean gazebo && GAZEBO_HEADLESS=true helm up gazebo
```

После изменения Dockerfile, compose или launch-файлов пересоберите образ и
пересоздайте контейнер:

```bash
helm clean gazebo
helm build gazebo
helm up gazebo
```

## Модель и ROS-интерфейс

Пакет [src/sim_kobuki/](src/sim_kobuki/) содержит:

```text
sim_kobuki/
├── config/gz_bridge.yaml
├── description/kobuki.urdf.xacro
├── launch/launch_sim.launch.py
├── meshes/kobuki/
└── worlds/empty.sdf
```

Launch-файл запускает:

1. `robot_state_publisher`;
2. Gazebo Harmonic;
3. спавн Kobuki из `robot_description`;
4. `ros_gz_bridge`.

Интерфейс совместим с прежним симуляционным контуром:

| Топик | Тип | Направление |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → Gazebo |
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | Gazebo → ROS |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo → ROS |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo → ROS |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS |
| `/tf`, `/tf_static` | TF | симулятор / robot_state_publisher |

Лидар Livox Mid-360 установлен в `livox_frame`, работает на 10 Гц через
[RGLGazeboPlugin](https://github.com/RobotecAI/RGLGazeboPlugin) (GPU OptiX)
с пресетом `Livox Mid360` — non-repetitive паттерн, ~200k pts/s,
диапазон 0.1–40 м. PointCloud2 идёт через `ros_gz_bridge` на `/livox/lidar`.
В GUI Gazebo облако смотрите плагином **RGLVisualize** (топик с суффиксом
`/world`), не Visualize Lidar.
Статические трансформы модели публикует
`robot_state_publisher`, а Gazebo публикует динамическую цепочку
`odom → base_footprint → base_link`.

Пластины TurtleBot 2 держатся на шестигранных стойках. Стойки заданы как
visual-геометрия самих пластин, поэтому не создают отдельных тел и коллизий, но
лидар их видит.

Опорные ролики стоят на 0.5 мм выше пятна контакта колёс, из-за чего вес несут
приводные колёса. При контактах на одной высоте решатель нагружал почти
безфрикционные ролики, и робот после команды «стоп» продолжал скользить вперёд
примерно 17 см. Сейчас выбег около 0.5 см, полная остановка за 0.13 с, а корпус
опирается на передний ролик с наклоном примерно 0.25°.

## Проверка

```bash
helm enter gazebo
source /autoware/install/setup.bash

ros2 topic list
ros2 topic hz /livox/lidar
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link livox_frame
```

Управление с клавиатуры:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Просмотр облака:

```bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

В RViz выберите `Fixed Frame: odom` и добавьте PointCloud2
`/livox/lidar`. В настройках дисплея поставьте **Reliability: Best Effort**.

## Структура репозитория

| Путь | Назначение |
| --- | --- |
| [docker/Dockerfile](docker/Dockerfile) | образ Autoware + Humble + Harmonic |
| [docker/docker-compose.yaml](docker/docker-compose.yaml) | сервисы автопилота и `gazebo` |
| [docker/launch.yaml](docker/launch.yaml) | команды `helm` |
| [src/sim_kobuki/](src/sim_kobuki/) | модель, мир, bridge и launch симуляции |
| [helm_launch/](helm_launch/) | CLI `helm` |

## Диагностика

**Нет `/clock` или `/livox/lidar`.** Проверьте `helm flogs gazebo`. Нужны
GPU (NVIDIA), `RGL_PATTERNS_DIR=/opt/rgl/lidar_patterns` и
`GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/rgl/plugins`. В мире должен быть
`RGLServerPluginManager`.

**GUI не открывается.** Установите `GAZEBO_HEADLESS=false`, выполните
`xhost +local:root` и проверьте переменную `DISPLAY`.

**Конфликт Gazebo-пакетов при сборке.** В образе не должны одновременно
присутствовать `ros-humble-ros-gz*` для Fortress и
`ros-humble-ros-gzharmonic`.
