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
   не требуется. Реальный робот использует отдельный образ на базе контейнера HSL25.

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
реальный стек подключает статическую карту и AMCL; физическая проверка остаётся открытой.

Каждый `helm start_match` пересоздаёт мир и сохраняет копию конфига и
эффективные настройки в `results/manual-*/`. Исследователь должен довести
центр робота до центра стартовой площадки стража с допуском 0,08 м;
касание границы теперь не завершает матч. Это более строгое условие, чем
регламентное касание контура. Путь заканчивается в точном центре площадки.
Запуск печатает этапы готовности/проверки/разрешения с временем и сохраняет
`startup-timing.json`. Проверка параметров выполняется одним ROS-клиентом через сервисы;
движение открывается только после готовности обоих MPPI и проверки образа.
Подготовить мир без движения:

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

## Реальный робот

Драйверы Kobuki/Mid-360 из HSL25 находятся в `drivers/src`, интеграция —
в `src/hsl_real`. Отдельный CPU-запуск без Gazebo и без второго робота:

```bash
helm build_real       # собрать текущий стек и драйверы
helm start_real       # драйверы + навигация, движение закрыто
helm enable_real      # проверить готовность и разрешить движение
helm pause_real       # остановить движение, сохранить работающие драйверы
helm stop_real        # остановить движение и контейнер
helm logs_real        # логи
helm status_real      # состояние
helm enter_real       # терминал
```

Настройки оборудования — `config/real.yaml` и `config/livox_mid360.json`,
реальная миссия — `config/real_match.yaml` (отдельно от симуляции).
Статическая `maps/maze_bag_v1.yaml` и локализация AMCL запускаются внутри
`start_real`. Livox → высотный срез LaserScan → AMCL; одометрия остаётся
`/odom`, положение на карте — `/amcl_pose` и скорректированный
`/navigation/self` для decision/planner/MPPI. Настройки — `localization.yaml`.
Перед включением движения задайте фактические стартовые позы в real_match;
в RViz доступен **2D Pose Estimate**. При потере актуальной локализации
движение закрывается, повторное разрешение — `enable_real`.
`start_real` автоматически пишет диагностический MCAP в
`recordings/<UTC>-autonomous/bag`; `stop_real` завершает запись и проверяет metadata.
`/navigation/indication` показывает роль, условия достижения цели или поимки.
В `real_match.yaml` активное время — 600 с; симуляционный конфиг независим.
Физическая езда этой интеграции ещё не подтверждена.
Подробная настройка и проверка: [docs/REAL_ROBOT.md](docs/REAL_ROBOT.md).

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
python3 benchmarks/report_straight_motion.py results/series-YYYYMMDDTHHMMSSZ > /tmp/straight-motion.json
```

`report_straight_motion.py` отдельно оценивает измеренные коррекции на
прямой reference: минимум0,45м впереди, изменение направления сегментов
≤0,05rad; скорость≥0,05м/с. Смена знака omega считается за пределами
deadband±0,05rad/s; смена поведения/направления reference, остановка или
gap>0,3s сбрасывают последовательность. Время/RMS взвешены по sim-интервалам
внутри referee window. Нехватка геометрии исключается, её покрытие явно
показано. Прямая reference не доказывает свободный коридор; коррекция
не автоматически означает дефект. Доли дугового движения также считаются
по времени, по порогам speed≥0,05 и |omega|≥0,1 на обоих концах интервала.

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

## Локализация

В Gazebo поза берётся из p3d. На реальном роботе AMCL сопоставляет Livox
скан со статической картой и исправляет одометрию Kobuki через map→odom.
SLAM и сценарий картографирования в аппаратный запуск не входят.
Статическая карта из SDF используется только в симуляции; переносимость на
реальные лабиринты ещё не подтверждена.

## Модель и ROS-интерфейс

Пакет [src/sim_kobuki/](src/sim_kobuki/) содержит:

```text
sim_kobuki/
├── description/kobuki.urdf.xacro
├── launch/launch_sim.launch.py
├── launch/launch_duel.launch.py
├── meshes/kobuki/
└── worlds/
    └── polygon_rosbag.world   # реконструкция реального лабиринта
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
MAP=polygon_rosbag     # реальный лабиринт (по умолчанию)
```

Чтобы добавить свой мир, положите `<имя>.world` в
[src/sim_kobuki/worlds/](src/sim_kobuki/worlds/) и укажите `MAP=<имя>`.
Стены лабиринта заданы **box**-коллизиями.

Точка спавна робота тоже в `.env` и по умолчанию соответствует старту
лабиринта:

```bash
SPAWN_X=0.032
SPAWN_Y=-0.082
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

`hsl_perception/opponent_detector` использует алгоритм из `feature/detector`
(b985515): сегментация, подгонка окружности корпуса Kobuki и несколько треков
с фильтром Калмана постоянной скорости. Реализация Python/NumPy; прежний C++
детектор сохранён в Git checkpoint 4f9168c. Полная ветка не сливается:
симуляционный мир, MPPI и запуск остаются текущими.

Входы каждого детектора: свои `navigation/scan` (map, timestamped TF),
`navigation/self`, статический `navigation/known_grid`. Неизвестные коробки
не входят в фон. Выходы: `navigation/opponent` (Odometry, скорость в его
локальном frame), `opponent_visible`, `detector_cycle_ms`, `detector_diagnostics`
(JSON с причинами отказа, кандидатами и состоянием треков). Соперник из Gazebo
не подаётся алгоритму. Прогноз без новой детекции не обновляет Odometry/видимость.
Стационарный робот может открывать трек после трёх сильных детекций.

`perception.opponent_max_height` в match YAML ограничивает максимальную высоту
кластера (default0.46м); остальные параметры формы `robot.*` и трекера `tracker.*`
объявлены ROS-параметрами. В симуляции включена проверка зазора между
пластинами `robot.max_gap_share=0.12` и более строгий fit окружности
`robot.line_ratio=0.35`; real сохраняет исходные defaults1.0/0.7
до проверки на оборудовании. Для открытия трека simulation также требует
дугу≥90° и≥95% согласованных точек обода; из большого смешанного кластера
допускается только продолжение уже известного трека. Это уменьшает ложные
открытия на коробках, но затрудняет первое обнаружение при сильном перекрытии.
В адаптации фон вычитается по occupancy grid с
запасом0.08м вместо аналитических боксов SDF из исходной ветки. Точность
реальных данных и переносимость требуется проверять отдельно.

Для записи реальных облаков обоих детекторов добавьте к изолированной
проверке `--unknown-obstacle --record-detector-scans`. В `NN-obstacle.json`
сохраняются height-filtered map-frame points, карта и truth-метки только
для offline-оценки. Они не подаются в навигацию.

Offline-повтор записанных облаков через текущий NumPy core:
```bash
python3 benchmarks/replay_detector_clouds.py results/isolated/hsl-eval/series-20261001T191042Z/00-obstacle.json --height 0.46 --output /tmp/detector-replay.json
```
Нужен Python3 с NumPy. Truth используется только для labels результата,
не для выбора кандидата. Это дополнительная проверка на записанных входах,
а не замена новой дуэли или измерения recall при всех условиях видимости.
Полные облака с eval-позами записывает `benchmarks/capture_detector_clouds.py`
в stdout; replay поддерживает этот формат. `benchmarks/report_detector.py <series>`
сравнивает парные trace только внутри активного окна referee.

Для отдельной проверки низкого препятствия после освобождения
изолированного стенда:
```bash
python3 benchmarks/run_duel_series.py --isolated-project hsl-eval --ros-domain-id 73 --gazebo-port 11418 --runs 1 --start-seed 0 --active-s 90 --trace --audit-start --unknown-obstacle --obstacle-height 0.15 --record-detector-scans
```
`--obstacle-height` меняет физическую высоту fixture, XY остаётся0.6×0.6м.
Default0.8м, минимум0.15м; размеры сохраняются в отчёте. Новый низкий
вариант подготовлен, но ещё не проверен физически; два успешных заезда
с высокой коробкой не доказывают его обработку.

### Собственная скорость в модели перехвата

`motion.max_speed` из внешнего match YAML передаётся MPPI, глобальному
planner и обоим decision manager как `own_max_speed`. PURSUE использует
это значение как собственную возможность движения в модели встречи;
фактическая скорость на повороте может быть ниже. Это не дополнительный
предел команд. Скорость соперника берётся из LiDAR-трека и не ограничивается
значением из нашего конфига. Runtime audit сверяет `own_max_speed` у обеих
ролей перед разрешением движения.

## Запись данных реального робота

Симуляция использует прежний `jr_image` и `helm start_match`.
Реальный стек собирается независимо, на базе контейнера HSL25
`nickodema/kobuki:humble-22.04-100625`:

```bash
helm build_real             # драйверы + наше решение в jr_real_image
helm start_real             # автономный стек, движение закрыто
helm enable_real            # разрешить после проверки готовности
helm stop_real
helm start_real_bag_record  # драйверы + запись MCAP + клавиатура
helm stop_real              # остановить движение и сохранить bag
```

Bag находится в `recordings/<session>/bag/`, список топиков и timeout
клавиатуры — в `config/recording.yaml`. Сценария построения карты нет.
Подробности — [REAL_ROBOT.md](docs/REAL_ROBOT.md#7-запись-bag-с-клавиатурой).

## Офлайн-карта по bag

Из maze_bag_v1 построена статическая Nav2-карта:
[config/maps/README.md](config/maps/README.md). YAML/PGM загружены map_server;
карта выровнена по стенам на +1,8° и включена в real.yaml вместе с AMCL.
Нужны фактические стартовые позы в её системе координат. Локализация
проверяется воспроизведением записи; физическая проверка остаётся открытой.

Офлайн-аудит, повтор обработки и ограничения —
[OFFLINE_MAPPING.md](docs/OFFLINE_MAPPING.md).

### Симуляция реконструированного реального лабиринта

`helm start_match` использует `polygon_rosbag.world` из ветки
`feature/rosbag_map`. Старый `maze.world` удалён; восстановить можно из Git.
В `config/match.yaml` заданы старты `[0.5,0.5,0]` и `[0.5,3.5,0]`, площадки 0.5×0.5 м.
Начало map находится в world[-0.468,-0.582]; это внутренний левый нижний угол стен. Эта система отличается от
выбранного углового начала реальной `maze_bag_v1`. Настройки реального робота
остаются в `real.yaml`/`real_match.yaml`.

```bash
helm build duel
helm start_match
```

Статическая `/map` строится из коллизий этого же мира. Готовая Nav2-пара
находится в `config/maps/polygon_rosbag.yaml` и `.pgm`; для обновления после
правок мира: `python3 tools/export_sdf_map.py`. Экспорт и ROS используют общий
растеризатор с разрешением 0.05 м. После изменения мира нужна пересборка образа.

### Неизвестные коробки в симуляции

`config/simulation_obstacles.yaml` задаёт коробки, которые `start_match`
добавляет через Gazebo SpawnEntity. Они отсутствуют в `.world` и статической
`/map`: робот получает их только через LiDAR. Сейчас включены три коробки
0.15×0.15×0.40 м и одна 0.40×0.60×0.20 м, с различными поворотами.
`pose: [x,y,yaw]` — координаты map, угол в радианах; `size` — метры.
Чтобы оставить две узкие коробки, удалите одну запись; чтобы отключить все,
установите `enabled: false`. Изменения YAML подхватываются после свежего
старта без пересборки. Движение разрешается только после успешного спавна.

Подготовить сцену **без разрешения движения**:

```bash
python3 benchmarks/start_match.py --prepare-only
```

Обычный автономный запуск: `helm start_match`. Для новой реализации детектора
сначала пересоберите `helm build duel`. Текущий детектор в этой работе не
изменялся; его ложные треки коробок остаются известным ограничением. Совместные
испытания двух ролей отложены до подключения нового детектора пользователем.

`/navigation/obstacle_grid` и `/opponent/navigation/obstacle_grid` — отдельные
карты статических стен + недавних LiDAR-препятствий (обновление5Гц). Их используют
A* и статический слой MPPI; текущий скан также используется напрямую. Видимые
исчезнувшие препятствия очищаются лучами, невидимые записи истекают через8с.
Исходная `/map` и `navigation/known_grid` остаются статическими. Это слой
препятствий при готовой локализации, не SLAM. В RViz добавлен observed obstacle map.

Для контролируемого одиночного заезда имеется
`benchmarks/obstacle_navigation_trial.py`: выполнять внутри **изолированного**
Gazebo-проекта после отключения его decision/referee контейнеров и отключения
штатных коробок в YAML. Скрипт сам размещает коробки на маршруте и убирает
второго робота с целевой площадки; это навигационный тест, не оценочная дуэль.
`--remove-after` позволяет проверить очистку карты после исчезновения коробок.
