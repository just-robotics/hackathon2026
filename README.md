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
`motion.allow_reverse` разрешает равноправное движение передом/задом
только исследователю, независимо от namespace. Страж всегда едет вперёд;
вращение на месте остаётся доступным обеим ролям.
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

У исследователя при allow_reverse=true диапазон `[-0.5, 0.5]`,
PathAngle.forward_preference=false; у стража `[0, 0.5]` и true.
PreferForward выключен. GoalAngle включается обёрткой только
у стража в CAPTURE, чтобы поимка выполнялась передом. GoalCritic вес15
у стража и5 у исследователя. До изменения стража на движение только вперёд проверены20 независимых матчей
на seeds0–19:14 целей и6 поимок, контактов и тайм-аутов0. Средние скорости
explorer/guardian0.363/0.392м/с, минимумы0.315/0.289м/с. Порог0.2 выполнен
во всех матчах,0.3 у стража в одном не выполнен. Полная таблица и ограничения —
в [оценке MPPI](docs/DUEL_EVALUATION_20261001.md). Переключение режима поимки
и неизвестные карте препятствия ещё требуют проверки; это базовая серия.

Единственный издатель каждого финального cmd_vel — `hsl_motion_gate`.
Он пропускает MPPI только со статусом OK/RECOVERY_MPPI и свежими pose,
scan, intent, path, command и общим match/active. При отсутствии безопасной
команды остановка сохраняется; планирование альтернативы продолжается.
Шлюз не добавляет доворот на месте или резервного исполнителя траектории.

Используется только штатный C++ MPPI Nav2. Python-адаптация `mppi.py`,
переключатель backend, debug follower и сценарный патруль удалены.
Описание границ интеграции — [NAV2_MPPI_ADAPTATION.md](docs/NAV2_MPPI_ADAPTATION.md).

Для отдельного испытания нового препятствия используйте изолированный runner:

```bash
python3 benchmarks/run_duel_series.py --isolated-project hsl-eval --runs 1 --start-seed 0 --active-s 360 --trace --audit-start --unknown-obstacle
```

Он добавляет ящик в Gazebo на фактически опубликованном пути первого робота,
проверив свободное место и существование альтернативы. Статическая карта
не меняется. `NN-obstacle.json` сохраняет положение, LiDAR hits, пути и позы;
поимка в этом опыте дополнительно требует отсутствия ящика на линии между
роботами. Это опция испытаний, обычный `helm start_match` ящик не добавляет.

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

Изолированный runner автоматически удаляет свой подтверждённый duel после
каждого заезда, включая последний: завершённый матч не должен оставлять
нагружающий CPU Gazebo. Совмещение двух миров снижает RTF; сравнивайте
производительность при одном работающем мире. Чужой заменённый runtime
runner не удаляет.

### Детектор соперника

`hsl_perception` — C++-узел `opponent_detector` для каждого робота. Получает
собственные `navigation/scan` (PointCloud2 в map), `navigation/self` и
`navigation/known_grid`; публикует прежние `navigation/opponent`,
`navigation/opponent_visible` и время вычисления `navigation/detector_cycle_ms`.
Оба детектора запускаются через `hsl_sim_adapter/observations.launch.py`
в контейнерах адаптеров; второй имеет namespace `/opponent`.
Python-адаптер сохраняет только собственную симуляционную локализацию,
TF-преобразование облака, удаление своего корпуса и map_points.
Детектор не подписывается на симуляционную позу соперника.
Кандидаты проверяются по верхнему90-му квантилю высоты целого кластера.
`perception.opponent_max_height` в config/match.yaml задаёт допустимую
высоту над плоскостью map (по умолчанию0.46м: текущий корпус0.4098м
плюс~0.05м на шум). Параметр применяется к обоим детекторам при новом старте,
без пересборки. Старые конфиги без perception получают этот default.
Размеры/силуэт похожего неизвестного предмета всё ещё могут дать ложный
кандидат; высота — геометрический фильтр, не семантическое распознавание.
Для реального корпуса и высоты плоскости карты параметр требует калибровки.

Дополнительно проверяется горизонтальный диаметр XY-кластера: расстояние
между любой парой точек не должно превышать0.476м. Основа — диаметр корпуса
0.356м и запас по0.06м на каждую сторону (3σ шума симуляционного LiDAR).
Диагональ XY bounding box может быть больше диаметра даже у круглого робота;
для спорных кластеров проверяются вершины выпуклой оболочки. Это необходимая
проверка размера, а не распознавание класса: небольшой видимый фрагмент
неизвестного предмета всё ещё может оказаться ложным соперником. Перенос
данного шумового запаса на реальные данные пока экспериментально не проверен.

Для записи реальных облаков обоих детекторов добавьте к изолированной
проверке `--unknown-obstacle --record-detector-scans`. В `NN-obstacle.json`
сохраняются height-filtered map-frame points, карта и truth-метки только
для offline-оценки. Они не подаются в навигацию.

Offline-повтор записанных облаков через текущий C++ core:
```bash
python3 benchmarks/replay_detector_clouds.py results/isolated/hsl-eval/series-20261001T191042Z/00-obstacle.json --height 0.46 --output /tmp/detector-replay.json
```
Нужен host g++ с C++17. Truth используется только для labels результата,
не для выбора кандидата. Это дополнительная проверка на записанных входах,
а не замена новой дуэли или измерения recall при всех условиях видимости.

Для отдельной проверки низкого препятствия после освобождения
изолированного стенда:
```bash
python3 benchmarks/run_duel_series.py --isolated-project hsl-eval --ros-domain-id 73 --gazebo-port 11418 --runs 1 --start-seed 0 --active-s 90 --trace --audit-start --unknown-obstacle --obstacle-height 0.15 --record-detector-scans
```
`--obstacle-height` меняет физическую высоту fixture, XY остаётся0.6×0.6м.
Default0.8м, минимум0.15м; размеры сохраняются в отчёте. Новый низкий
вариант подготовлен, но ещё не проверен физически; два успешных заезда
с высокой коробкой не доказывают его обработку.
