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

   Основной MPPI устанавливается при сборке образа; подмодуль контроллера
   не требуется. Необязательный LIO-SAM подключается отдельно.

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

## Сборка и запуск

```bash
helm build duel              # после изменений исходников
helm start_match             # свежий матч из config/match.yaml
helm stop_match              # остановить обоих
```

Настройки матча находятся в **`config/match.yaml`**. Редактирование этого
файла не требует пересборки: он читается на хосте при каждом старте.
`robot.start` и `opponent.start` — `[x, y, yaw]` в системе `map`,
метры и радианы. `robot.role` задаёт нашу роль, роль соперника противоположная.
`start_area_half_size` задаёт половину стороны квадратной стартовой площадки.
`motion.allow_reverse` разрешает равноправное движение передом/задом;
страж в CAPTURE всё равно ориентируется передом на исследователя.
`max_speed` и `max_angular_speed` — ограничения штатного MPPI, а не порог
средней скорости. `match` задаёт seed и длительность активной части.

`simulation` задаёт мир, смещение `map` относительно world, границы арены,
GUI/headless и RViz. Смещение карты не зависит от стартовой позы.
При отсутствии DISPLAY запуск автоматически headless. Положение внутри стен
и свободное место вокруг стартов нужно проверить по карте: YAML не проверяет
пересечение с геометрией стен. Mission-секции предназначены и для хакатона;
источники реальной карты, локализации и наблюдений ещё предстоит подключить.

Каждый `helm start_match` пересоздаёт мир и сохраняет копию конфига и
эффективные настройки в `results/manual-*/`. Подготовить мир без движения:

```bash
python3 benchmarks/start_match.py --prepare-only
python3 benchmarks/start_match.py --print-env     # проверить конфиг без запуска
HSL_MATCH_CONFIG=/absolute/path/match.yaml helm start_match
python3 benchmarks/run_duel_series.py --config config/match.yaml --runs 20 --trace
```

Benchmark сохраняет YAML и параметры каждого заезда; CLI seed, длительность
и назначение ролей служат явными overrides для серии.

`helm start` и `helm stop` — алиасы start_match/stop_match. До общего старта
и после первого исхода оба робота получают нулевые финальные команды.
`helm up gazebo` запускает одиночный сенсорный стенд; профиль `simulation`
содержит Gazebo, карту и инструменты, без старой демонстрации MPC/траектории.
Автономное движение проверяется в `duel`.

RViz2 автоматически открывается при наличии DISPLAY и показывает карту,
оба RobotModel и пути. Для headless: `HSL_RVIZ_ENABLED=false`.
Второй RobotModel использует TF Prefix `opponent`; названия First/Second
обозначают физические namespace, а не неизменные роли.

## Контроллер и безопасность

**MPPI — единственный контроллер рабочего стека.** Исходный C++ плагин
Nav2 Humble MPPIController1.1.20 используется собственной обёрткой,
без controller_server. MPC-ноды, подмодуль, параметры и fallback удалены;
вернуться к прежнему варианту можно через историю Git/checkpoint.
`HSL_CONTROL_MODE` и `HSL_MPC_PATH_SOURCE` больше не выбирают контроллер.

A* строит глобальный маршрут по текущим наблюдениям и карте. Каждый робот
имеет отдельные decision manager, планировщик, rolling costmap, MPPI,
шлюз и метрики. MPPI выдаёт прямую команду `navigation/mppi_cmd_vel` и
показывает ближайшие1,2м оптимизированной траектории в `local_path`.
Перед выдачей команды проверяется весь3-секундный перемещаемый контур,
а не только этот визуальный префикс. Защитный радиус0,23м соответствует
глобальному планировщику; физический collision cylinder имеет радиус0,178м.

Перед/зад равнозначны: `vx_min=-0.5`, `vx_max=0.5`, PreferForward выключен,
PathAngle.forward_preference=false. GoalAngle включается обёрткой только
у стража в CAPTURE, чтобы поимка выполнялась передом. GoalCritic вес15
у стража и5 у исследователя. В baseline серии seeds12–14 получены две цели и одна поимка, контактов0,
минимальные средние скорости explorer/guardian0.325/0.377м/с. После очистки
Python legacy повторная проверка seeds12–14 дала две поимки и одну цель,
контактов0, минимальные средние скорости0.256/0.318м/с. Итоговые20 заездов
ещё выполняются; эти короткие серии не доказывают устойчивость алгоритмов.

Единственный издатель каждого финального cmd_vel — `hsl_motion_gate`.
Он пропускает MPPI только со статусом OK/RECOVERY_MPPI и свежими pose,
scan, intent, path, command и общим match/active. При отсутствии безопасной
команды остановка сохраняется; планирование альтернативы продолжается.
Шлюз не добавляет доворот на месте или резервного исполнителя траектории.

Используется только штатный C++ MPPI Nav2. Python-адаптация `mppi.py`,
переключатель backend, debug follower и сценарный патруль удалены.
Описание границ интеграции — [NAV2_MPPI_ADAPTATION.md](docs/NAV2_MPPI_ADAPTATION.md).

## ROS-интерфейс duel

Первый стек использует `/navigation/...`, второй — `/opponent/navigation/...`:

| Интерфейс | Тип и назначение |
| --- | --- |
| self / opponent | Odometry: собственная локализация и наблюдаемый трек соперника |
| scan / map_points / known_grid | PointCloud2 / OccupancyGrid: препятствия и карта |
| intent | PlanningIntent: задача от decision manager |
| global_path / nav2_reference / local_path | Path: A*, ссылка MPPI, визуальный префикс rollout |
| mppi_cmd_vel | Twist: проверенная команда MPPI |
| planner_status / global_status | String: состояние локального/глобального планирования |
| planning_diagnostics / mppi_diagnostics | String JSON: входы/выбор пути и результат MPPI |
| control_cycle_ms / native_mppi_cycle_ms | Float32: вычислительные задержки |

Общие `/match/active`, `/match/outcome` задают активное окно и первый исход.
Разрешения `/match/allow_motion` и `/opponent/match/allow_motion` (SetBool)
нужны обоим. Финальные `/cmd_vel` и `/opponent/cmd_vel` раздельны.
Отчёты referee и обоих роботов сохраняются в `results/` и относятся к
одному активному окну; разрешение только одному не запускает движение.

## Оценка и диагностика

```bash
python3 -m pytest -q tests helm_launch/tests/tests.py
python3 benchmarks/run_duel_series.py --runs 3 --start-seed 19 --first-role guardian --active-s 90 --scenario 3 --trace --audit-start
python3 benchmarks/report_motion.py results/series-YYYYMMDDTHHMMSSZ
```

Финальная оценка — не менее20 независимых заездов до первого события или
360с активного симуляционного времени. Скорость измеряется за весь матч,
без исключения разворотов/остановок из знаменателя. Начальные пороги0,2м/с,
далее0,3м/с — критерии средней скорости, не потолки команд. Нужны повторные
цели и поимки текущей версии, отсутствие контактов и приемлемые ошибки и
плавность. Геометрия поимки: дистанция<0,45м, направление переда≤45°,
нет препятствия. Требования и остающиеся ограничения — в
[PROJECT_GOAL.md](docs/PROJECT_GOAL.md) и [PROJECT_STATUS.md](docs/PROJECT_STATUS.md).

Runner сохраняет seed, роли, source hashes, image IDs, эффективные параметры,
версии Nav2, парные метрики и trace. `--rviz` включает GUI при DISPLAY.
В trace measured_omega_radps — собственная измеренная угловая скорость;
`mppi_capture_heading_required` показывает требование ориентации при поимке.
Сравнивайте ускорения на общих временных окнах при разных длительностях.

Отдельная проверка native MPPI на синтетической свободной карте:

```bash
docker run --rm --network host -e ROS_DOMAIN_ID=73 -v "$PWD/benchmarks/audit_native_bidirectional.py:/tmp/audit.py:ro" --entrypoint bash jr_image:latest -lc 'source /autoware/install/setup.bash && python3 /tmp/audit.py'
```

Она использует отдельный ROS-домен73, не запускает Gazebo и не публикует
финальный cmd_vel. Это проверка интеграции, не оценочный матч.

## Необязательная локализация

```bash
helm submodules lio_sam
```

LIO-SAM хранится в `src/lio_sam_src`, пока исключён из сборки основного образа.
В Gazebo собственная локализация берётся из p3d. Трек соперника оценивается
по LiDAR-кластерам и их истории, ground truth используется только referee
и измерениями. Статическая карта из SDF разрешена текущей архитектурой;
переносимость на другие лабиринты/реальное оборудование ещё не доказана.

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

## Изолированная оценка рядом с ручным миром

```bash
python3 benchmarks/run_duel_series.py --isolated-project hsl-eval --ros-domain-id 73 --gazebo-port 11418 --runs 3 --start-seed 12 --active-s 90 --trace --audit-start
```

Отдельный проект Compose, ROS domain и Gazebo master исключают обмен
командами/наблюдениями с обычным docker-проектом. Результаты находятся в
`results/isolated/hsl-eval/`; ручные latest.json не перезаписываются.
Навигация и YAML остаются прежними. Дополнительная нагрузка может снизить
RTF: учитывайте её при сравнении. Не используйте тот же domain/порт для
других экспериментов одновременно. После серии удаляйте только её проект:
`COMPOSE_PROJECT_NAME=hsl-eval ROS_DOMAIN_ID=73 GAZEBO_MASTER_URI=http://127.0.0.1:11418 helm clean duel`.
