# hackathon2026

Стек автопилота Autoware / ROS 2 Humble и симуляция Kobuki с многослойным
лидаром в Gazebo Classic 11. Все компоненты запускаются в Docker через CLI `helm`
из [helm_launch/](helm_launch/).

## Требования

- Docker с плагином `docker compose` v2
- Python 3 и `pip`
- NVIDIA Container Toolkit для контейнера Autoware
- X11, если нужен GUI Gazebo

## Установка

1. Склонировать репозиторий:

   ```bash
   git clone git@github.com:just-robotics/hackathon2026.git
   cd hackathon2026
   ```

   Пакеты из подмодулей (например MPC-контроллер) подключаются отдельной
   командой после установки `helm` — см. [Подмодули](#подмодули).

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

## Подмодули

Внешние ROS-пакеты подключаются git-подмодулями в [src/](src/). Список описан в
[docker/submodules.yaml](docker/submodules.yaml), подключение автоматизировано:

```bash
helm submodules                     # все подмодули из конфига
helm submodules mpc_motion_control  # только указанные
```

Команда идемпотентна: если подмодуль уже подключен, она его не добавляет
повторно, а доподтягивает (`git submodule update --init`). Поэтому одной и той
же командой закрываются оба случая — первое подключение и свежий клон
репозитория.

Сейчас подключаются два подмодуля:

| Подмодуль | Ветка | Пакеты | Назначение |
| --- | --- | --- | --- |
| [mpc_motion_control](https://github.com/artem-kondratew/mpc_motion_control/tree/hackathon2026) | `hackathon2026` | `swarm_msgs`, `swarm_controller` | MPC: круиз-контроль, ACC, удержание в полосе |
| [lio_sam](https://github.com/artem-kondratew/mpc_motion_control/tree/main) | `main` | `lio_sam` | локализация по лидару и IMU |

Это один и тот же репозиторий, но разные ветки: в `hackathon2026` LIO-SAM
вырезан как относящийся к реальному железу, поэтому локализация берется из
`main` и подключается вторым подмодулем.

В корне подмодуля лежит `COLCON_IGNORE`, поэтому автообход colcon его
пропускает, а пакеты собираются явными путями — они перечислены в поле
`packages` конфига и продублированы аргументом `SUBMODULE_PACKAGES` в
[docker/Dockerfile](docker/Dockerfile). Там же ставятся решатель QP
(`osqp`, `scipy`) и GTSAM для `lio_sam`, которых нет в базовом образе. Если
подмодуль не подключен, шаг сборки пропускается и образ остается собираемым.

После подключения подмодуля пересоберите образ, иначе пакетов в контейнере
не будет:

```bash
helm build gazebo
```

Контроллер запускается вместе с симуляцией — команды прописаны в сервисах
`planning` и `control`, отдельно ничего запускать не нужно:

```bash
helm up simulation
```

Это демонстрационный профиль `main` для **одного** Kobuki и заранее заданной
траектории. Двухроботный контур decision manager запускается профилем `duel`
ниже. Профили используют общие ROS-топики и должны запускаться по отдельности.

`planning` поднимает `sim_planning.launch.py` (строит опорную траекторию в
`/planning/trajectory`), `control` — `sim_control.launch.py`
(`/planning/trajectory` + `/odom` → `/cmd_vel`). Параметры вынесены в
[.env](.env):

| Переменная | По умолчанию | Значения |
| --- | --- | --- |
| `MPC_TRAJECTORY` | `lanelet` | `line`, `circle`, `lanelet` |
| `MPC_TRAJECTORY_FILE` | `my_trajectory5.yaml` | waypoints для `lanelet` |
| `MPC_LATERAL` | `true` | удержание в полосе |
| `MPC_LONGITUDINAL` | `cc` | `cc` — профиль скорости, `acc` — зазор за лидером |

После правки `.env` пересоздайте контейнеры: `helm clean simulation && helm up simulation`.

Контроллер стартует в режиме ожидания и не двигает робота, пока не выставлен
параметр `start` — это страховка от самопроизвольного старта:

```bash
helm start   # поехали
helm stop    # стоп
```

Имя продольной ноды зависит от режима (`swarm_cc_mpc_node` для `cc`,
`swarm_acc_mpc_node` для `acc`), поэтому команда не зашивает его, а находит
среди запущенных нод ту, у которой есть параметр `start`. Если контроллер не
поднят, команда сообщает об этом и возвращает ненулевой код — `ros2 param set`
сам по себе в этом случае молча завершается успехом.

Траекторию можно менять на ходу, без перезапуска:

```bash
helm exec planning ros2 param set /planning/trajectory_planner trajectory line
helm exec planning ros2 param set /planning/trajectory_planner circle_radius 3.0
```

## Локализация

Позу робота даёт сам симулятор: плагин `gazebo_ros_p3d` в URDF берёт её прямо
из физического движка и публикует в `/localization/pose` как
`nav_msgs/Odometry`. Это ground truth — без дрейфа, без накопления ошибки и без
SLAM.

```bash
helm exec tools ros2 topic echo /localization/pose --once
```

Отличие от `/odom`: в Gazebo Classic `/odom` тоже задан в мировых координатах,
но считается по колёсам плагином `diff_drive` и может накапливать ошибку при
пробуксовке. `/localization/pose` берётся прямо из физического движка.
Поэтому `/localization/pose` удобно брать как эталон для сравнения.

В TF плагин ничего не публикует — иначе у `base_footprint` было бы два
родителя (`odom` от `diff_drive` и фрейм p3d) и дерево развалилось бы.

Лидарная одометрия (LIO-SAM) пока не включена. Заготовка под неё есть:
подмодуль `lio_sam`, launch-файл
[jr_localization.launch.xml](src/jr_launch/launch/components/jr_localization.launch.xml)
и конфиг
[lio_sam.param.yaml](src/jr_launch/config/localization/lio_sam.param.yaml).
Чтобы её включить, нужен плагин Livox с настоящим паттерном: LIO-SAM требует
в облаке поля `ring` и `time` для деskew'а, а штатный `ray`-сенсор их не даёт.

## Сборка

Основной образ содержит Autoware, ROS 2 Humble, Gazebo Classic 11 и пакет
`sim_kobuki`:

```bash
helm build gazebo
```

Gazebo Classic формально EOL с января 2025, но выбран сознательно: только под
него существуют плагины Livox, воспроизводящие настоящий non-repetitive паттерн
Mid-360 вместе с полями `tag`/`line` и `offset_time`, которые нужны алгоритмам
лидарной одометрии для деskew'а. Плата за это — растеризация лучей на CPU
силами ODE вместо GPU.

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

Двухроботный сценарий с выбором поведения и построением путей:

```bash
helm submodules mpc_motion_control
helm build duel
helm clean duel
GAZEBO_HEADLESS=true helm up duel
```

Профиль `duel` запускает два Kobuki, карту `/map`, адаптер наблюдений,
decision manager, планировщик, сценарного соперника и MPC. Подмодуль
`mpc_motion_control` обязателен для этого профиля. Роль задаётся в `.env`
через `HSL_ROLE=explorer` или `HSL_ROLE=guardian`; после её смены пересоздайте
контейнеры (`helm clean duel && helm up duel`). Второй робот спавнится в
`OPPONENT_X`, `OPPONENT_Y`, `OPPONENT_Z`, `OPPONENT_YAW`. Контуры площадок
задаются в `src/hsl_decision/config/decision.yaml` в системе `map` и сейчас
служат только примером для мира `maze`.

До разрешения старта решение публикует `WAIT`, а защитный шлюз — нулевую
скорость. После истечения `freeze time` разрешите движение:

```bash
helm exec hsl-decision ros2 service call /match/allow_motion std_srvs/srv/SetBool "{data: true}"
```

Для остановки передайте `{data: false}`. `helm start` и `helm stop` управляют
штатным сервисом `control` профиля `simulation`; для `duel` используется
`/match/allow_motion`.

В `duel` `/navigation/self` и `/navigation/opponent` — позы `Odometry` в `map`
из симуляционного ground truth; `/navigation/scan` и
`/navigation/map_points` — текущие и накопленные 3D-точки `PointCloud2`;
`/navigation/known_grid` — доступная только в симуляции карта занятости.
В `duel` начало `map` совпадает с точкой спавна своего робота: адаптер сдвигает
в неё ground truth, а TF связывает мировые кадры одометрии с `map`.
В штатном `simulation` карта и `/odom` остаются в мировых координатах Gazebo.
Decision manager публикует `/navigation/intent`, планировщик —
`/navigation/global_path` и `/navigation/local_path` (`nav_msgs/Path`). MPC
следует по локальному пути, а единственный издатель `/cmd_vel` в этом профиле
останавливает робота при `WAIT`, `STOP`, пустом пути и устаревших данных.

Если нужно проверить путь без подмодуля MPC, остановите сервис `hsl-control`
и запустите `ros2 run hsl_debug_control debug_follower --ros-args -p
use_sim_time:=true` в том же ROS-домене. Этот контроллер — отладочная замена,
одновременно с MPC его запускать нельзя.

Окно симулятора открывается автоматически при выключенном headless-режиме. Compose передаёт в
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
├── description/kobuki.urdf.xacro
├── launch/launch_sim.launch.py
├── launch/launch_duel.launch.py
├── meshes/kobuki/
└── worlds/
    └── maze.world   # лабиринт
```

Launch-файл запускает:

1. `robot_state_publisher`;
2. Gazebo Classic (`gzserver`, при GUI — `gzclient`);
3. спавн Kobuki из `robot_description`;
4. плагины `gazebo_ros` публикуют топики напрямую, мост не нужен.

### Карта и точка спавна

Карта — это мир Gazebo из `worlds/`. Выбирается переменной `MAP` в
[.env](.env) по имени файла без расширения:

```bash
MAP=maze     # лабиринт (по умолчанию)
```

Чтобы добавить свой мир, положите `<имя>.world` в
[src/sim_kobuki/worlds/](src/sim_kobuki/worlds/) и укажите `MAP=<имя>`.
Стены лабиринта заданы **box**-коллизиями.

Точка спавна робота тоже в `.env` и по умолчанию соответствует старту
лабиринта:

```bash
SPAWN_X=-0.34
SPAWN_Y=-0.18
SPAWN_Z=0.23
```

После правки `.env` пересоздайте контейнер: `helm clean gazebo && helm up gazebo`.

Интерфейс совместим с прежним симуляционным контуром:

| Топик | Тип | Направление |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → Gazebo |
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | Gazebo → ROS |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Gazebo → ROS |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo → ROS |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo → ROS |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS |
| `/tf`, `/tf_static` | TF | симулятор / robot_state_publisher |

Лидар установлен в `livox_frame` и работает на 10 Гц, диапазон 0.1–40 м,
сектор по вертикали от −7° до +52° как у Mid-360. Пока это **штатный
вращательный `ray`-сенсор** Gazebo Classic (900×40 лучей), а не
non-repetitive паттерн Livox: плагин с настоящим паттерном подключается
отдельным шагом. PointCloud2 идёт на `/livox/lidar`.
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
| [docker/Dockerfile](docker/Dockerfile) | образ Autoware + Humble + Gazebo Classic |
| [docker/docker-compose.yaml](docker/docker-compose.yaml) | сервисы автопилота и `gazebo` |
| [docker/launch.yaml](docker/launch.yaml) | команды `helm` |
| [docker/submodules.yaml](docker/submodules.yaml) | список подмодулей с ROS-пакетами |
| [src/sim_kobuki/](src/sim_kobuki/) | модель, мир и launch симуляции |
| [src/hsl_decision/](src/hsl_decision/), [src/hsl_planning/](src/hsl_planning/) | выбор поведения и построение глобального/локального путей |
| [src/hsl_sim_adapter/](src/hsl_sim_adapter/), [src/hsl_debug_control/](src/hsl_debug_control/) | временные симуляционные входы и контроллер/шлюз |
| `src/mpc_motion_control/` | подмодуль с MPC-контроллером (`helm submodules`) |
| [helm_launch/](helm_launch/) | CLI `helm` |
| [docs/PROJECT_GOAL.md](docs/PROJECT_GOAL.md), [docs/PROJECT_STATUS.md](docs/PROJECT_STATUS.md), [docs/AGENTS.md](docs/AGENTS.md) | цель и регламент, состояние, инструкции для агентов |

## Диагностика

**Нет `/clock` или `/livox/lidar`.** Проверьте `helm flogs gazebo`. `/clock`
публикует `libgazebo_ros_init.so`, который грузит `gzserver` (аргумент
`init:=true`); топики сенсоров дают плагины `gazebo_ros` из URDF.

**GUI не открывается.** Установите `GAZEBO_HEADLESS=false`, выполните
`xhost +local:root` и проверьте переменную `DISPLAY`.

**Модель не спавнится.** `spawn_entity.py` ждёт готовности `gzserver`;
смотрите, поднялся ли он в `helm flogs gazebo`, и проверьте пути
`GAZEBO_MODEL_PATH` / `GAZEBO_RESOURCE_PATH`.
