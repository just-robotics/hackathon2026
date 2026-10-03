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

После обновления `helm_launch/` повторите установку той же командой: установленная CLI
содержит копию Python-кода и путь к этому checkout.

4. Автодополнение по Tab:

   ```bash
   sudo activate-global-python-argcomplete # or activate-global-python-argcomplete3
   ```

## Сборка и запуск

```bash
helm build duel              # после изменений исходников: один общий jr_image
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
`start_real`. Livox → фильтр собственных возвратов → высотный срез LaserScan → AMCL; одометрия остаётся
`/odom`, положение на карте — `/amcl_pose` и скорректированный
`/navigation/self` для decision/planner/MPPI. Настройки — `localization.yaml`.
Перед включением движения задайте фактические стартовые позы в real_match;
в RViz доступен **2D Pose Estimate**. При потере актуальной локализации
движение закрывается, повторное разрешение — `enable_real`.
`start_real` автоматически пишет диагностический MCAP в
`recordings/<UTC>-autonomous/bag`; `stop_real` завершает запись и проверяет metadata.
`/navigation/indication` показывает роль, условия достижения цели или поимки.
В `real_match.yaml` активное время — 600 с; симуляционный конфиг независим.
На роботе проведены заезды; остаются проблемы recovery и блокировки движения.
Последние исправления требуют повторной проверки на оборудовании.
Подробная настройка и проверка: [docs/REAL_ROBOT.md](docs/REAL_ROBOT.md).

## Архитектура

На каждого робота приходится независимый контур:

`наблюдения → детектор/decision manager → A* → Nav2 C++ MPPI → motion_gate → cmd_vel`

MPPI — единственный контроллер движения. MPC и Python MPPI удалены;
прежние варианты доступны в истории Git. Детектор — Python-пакет
`jr_perception` из `feature/detector`: фон из `/map` строится по арене
вокруг робота с запасом 15 см у стен, отладочные маркеры разнесены по трём
топикам. Старые `hsl_perception`,
`hsl_perception_cpp` и фильтр/классификатор коробок удалены.
В симуляции точная поза используется для собственного робота;
ground truth соперника доступен referee и оценке.

| Данные | Кто использует |
| --- | --- |
| Сырое `/livox/lidar` | Детектор `jr_perception` напрямую |
| `navigation/self` | Детектор, decision manager, планирование |
| `/opponent/odom` | Выбранный трек робота; decision manager и A* |
| `/opponent/markers` | Все кластеры по классам, у отклонённых — причина |
| `/opponent/robot_markers` | То, что считается роботом: детекции, треки, курс |
| `/opponent/box_markers` | Предметы на сцене: повёрнутые боксы с центром и размерами |
| `/map`, `navigation/known_grid` | Статический фон детектора, A*, StaticLayer MPPI |
| `navigation/scan` | Свежесть LiDAR для планирования и диагностика |
| `navigation/obstacle_grid` | Диагностическая копия статической карты |

Для второго симуляционного стека выходы `/opponent/opponent/odom` и
`/opponent/opponent/markers`; его колёсная одометрия — `/opponent/wheel/odom`.
Костмапа MPPI состоит из StaticLayer и InflationLayer. Объекты из детектора
не меняют occupancy. В `feature/egor-detector-fix` минимум точек обода поднят
до 12, кластеры с меньшим числом точек не обновляют трек. Центры в occupied-клетках
и кандидаты с закрытой картой поверхностью обода отклоняются. Для повторного
захвата нужны три последовательные сильные детекции; ограничение старой
позицией истекает через 1,5 с. Дальнейшие изменения требуют согласования. Маркеры показывают
коробки как кластеры, но не классифицируют их на маленькие/большие.
Исходный трекер может временно публиковать прогноз потерянного соперника.

В `start_real` независимый от TF фильтр стоит сразу после LiDAR:
`/livox/lidar → /sensing/lidar/points_filtered`. Очищенное облако получают AMCL
и адаптер наблюдений. Детектор получает raw `/livox/lidar` отдельным входом.
Raw сохраняется для диагностики. `start_real_bag_record` записывает raw;
фильтр реальных штанг в симуляции не запускается.

Финальный `cmd_vel` публикует только шлюз. При устаревших данных,
отсутствии безопасной команды или разрешения движение закрыто. MPPI проверяет
полный движущийся контур радиусом из `config/planning.yaml`; `local_path` — лишь визуальный
префикс оптимизированной траектории, не отдельный вход другого контроллера.

## Мир и неизвестные коробки

Текущий мир — `polygon_rosbag.world`, статическая карта получена из его
коллизий. Старты в `map`: `[0.5,0.5,0]` и `[0.5,3.5,0]`; yaw 0 направлен
вправо. Начало map — внутренний левый нижний угол стен,
в world это `[-0.468,-0.582]`. Real-карта имеет собственную привязку;
не переносите смещение из симуляции в real-конфиг.

`config/simulation_obstacles.yaml` добавляет неизвестные карте коробки:
три 15×15×40 см и одну 40×60×20 см с разными ориентациями.
Все четыре коробки подвижны и сохраняют физические коллизии.
Масса большой — 0,30 кг, маленьких — 0,10 кг, трение — 0,30; это
приблизительная модель. При текущей статической костмапе коробки вне карты
не учитываются планировщиком; детектор отображает кластеры в MarkerArray.

YAML коробок не требует сборки; после изменения мира или кода —
`helm build duel`. Перегенерация YAML/PGM: `python3 tools/export_sdf_map.py`.
Одиночный сенсорный стенд `helm up gazebo` читает мир/спавн из `.env`;
автономный матч читает `config/match.yaml`.

## Запись bag с клавиатурой

```bash
helm start_real_bag_record  # Kobuki + Mid-360 + MCAP + клавиатура
helm stop_real              # закрыть движение и штатно завершить запись
```

Записи — `recordings/<session>/bag/`, топики и watchdog клавиатуры —
`config/recording.yaml`. Сценария картографирования нет. Автономный `start_real`
также автоматически пишет диагностический bag. Подробности:
[REAL_ROBOT.md](docs/REAL_ROBOT.md).

## Детектор на бэгах

Пакеты собираются в контейнере поверх образа, бэг играет с часами бэга
(`--clock`). Сначала бэг на паузе (`--start-paused`), потом launch, потом
пробел в терминале бэга: иначе у узлов на времени бэга прыжок времени назад
стирает статический TF. Каталог `config` — `config/` репозитория или папка
сессии (`lidar_filter.yaml`, `localization.yaml`, `maps/`).

**Неподвижный лидар** (фон записан `record_background.py`):

```bash
ros2 launch jr_launch jr_perception_bag.launch.xml background_file:=<фон.npz>
```

**Едущий робот, поза от AMCL.** В autonomous-записях AMCL уже в `/tf`:

```bash
ros2 bag play <сессия>/bag --clock --start-paused \
  --topics /livox/lidar /sensing/lidar/points_filtered /odom /tf /tf_static /map
ros2 launch jr_launch jr_detector_bag.launch.xml
```

В ручных записях (`*-bag`: только лидар, IMU, одометрия, TF) AMCL
поднимается заново — фильтр лидара, срез в скан, `map_server`, `nav2_amcl`
(пакеты `ros-humble-nav2-amcl`, `-nav2-map-server`, `-nav2-lifecycle-manager`,
`-pointcloud-to-laserscan`) — со стартом по первым сканам:

```bash
ros2 run jr_perception initial_pose.py <сессия>/bag config/maps/maze_bag_v1.yaml  # -> x y yaw
ros2 bag play <сессия>/bag --clock --start-paused --topics /livox/lidar /odom /tf /tf_static
ros2 launch jr_launch jr_raw_localization.launch.xml config:=config x:=… y:=… yaw:=…
ros2 launch jr_launch jr_detector_bag.launch.xml
```

**Едущий робот, FAST-LIO2 с коррекцией по статической карте.**
FAST-LIO2 оценивает движение по raw LiDAR и IMU; AMCL сопоставляет очищенный
скан с картой. `fastlio_bridge` публикует `/localization/lio_odometry` и
TF `lio_odom → odom`, AMCL — `map → lio_odom`. Итоговая поза —
`/localization/kinematic_state`. Старт `x y yaw` — начальное приближение.
При replay исключите записанные TF локализации (map→odom/map→lio_odom),
оставив колёсные TF и tf_static; иначе появляются конкурирующие родители.
FAST-LIO2 уже находится в `src/fast_lio` и собирается реальным образом.

```bash
helm build_real
ros2 bag play <сессия>/bag --clock --start-paused --topics /livox/lidar /livox/imu /tf /tf_static
ros2 launch jr_launch jr_detector_fastlio.launch.xml config:=config x:=… y:=… yaw:=… \
  map:=config/maps/maze_bag_v1.yaml
```

В autonomous-записях `/map` есть в бэге (добавьте в `--topics`, `map:=` не
нужен). Записанные рёбра локализации нужно исключить из `/tf`; простой
выбор топика `/tf` этого не делает. `publish_tf:=true` требуется для новой
цепочки AMCL. Повтор бэга
по кругу не годится — FAST-LIO2 продолжит с конца прошлого прохода.
`config/perception/fastlio_view.rviz` в `jr_launch` показывает сам FAST-LIO2:
облако, сшитое по его позе, и траекторию.

В исторической проверке прежнего моста на `20261003T094119` его поза отличалась от AMCL робота в среднем на 4 см
(максимум 8 см), курс — на 1.3°.

## Проверки и текущее состояние

```bash
python3 -m pytest -q tests
(cd helm_launch/tests && python3 -m pytest -q tests.py)
python3 benchmarks/run_duel_series.py --runs 3 --start-seed 0 --active-s 90 --trace --audit-start
```

Средняя скорость измеряется за весь активный матч; текущий ориентир —
не менее 0,2 м/с при малом латеральном отклонении, плавности и отсутствии
столкновений со стенами. Максимум команд задаётся в motion YAML.
Исторические серии относятся к своим версиям кода и полигона и не подтверждают
текущую версию. Остаются ложные треки коробок, блокировка движения на real
и точность финиша при толкании коробки; подробности — в статусе.

| Документ | Назначение |
| --- | --- |
| [PROJECT_STATUS.md](docs/PROJECT_STATUS.md) | Сводка текущего дерева, результаты, ограничения и история циклов |
| [PROJECT_GOAL.md](docs/PROJECT_GOAL.md) | Цель и требования регламента |
| [AGENTS.md](docs/AGENTS.md) | Правила разработки и передачи состояния |
| [REAL_ROBOT.md](docs/REAL_ROBOT.md) | Оборудование, конфиги, сборка, запуск, остановка и диагностика |
| [DIAGNOSTICS.md](docs/DIAGNOSTICS.md) | ROS-интерфейсы, серии, графики и replay детектора |
| [NAV2_MPPI_ADAPTATION.md](docs/NAV2_MPPI_ADAPTATION.md) | Границы интеграции штатного MPPI |
| [OFFLINE_MAPPING.md](docs/OFFLINE_MAPPING.md) | Получение статической карты из bag |
| [config/maps/README.md](config/maps/README.md) | Карты и их системы координат |

Параметры A*/запаса контура и мягких штрафов MPPI вынесены в
[`config/planning.yaml`](config/planning.yaml), общие для sim/real. После
редактирования перезапусти стек; пересборка для YAML не нужна. Фильтр LiDAR
в основном запуске — C++ (`hsl_lidar_filter`), детектор — `jr_perception`.
Текущие проверки/неустранённые отказы — в `docs/PROJECT_STATUS.md`.
`local.MPPI.time_steps` задаёт длину прогноза: по умолчанию60шагов по0,05с
(3с). Проверка движущегося контура охватывает весь выбранный прогноз.

## Основная локализация реального стека: FAST-LIO2 + карта

`config/real.yaml`: `localization: fastlio`, `fastlio_file: fastlio.yaml`,
`localization_file: localization.yaml`, `map_file: maps/maze_bag_v1.yaml`.
`robot.start` в `real_match.yaml` задаёт начальное приближение на карте.

FAST-LIO2 получает raw LiDAR + IMU. AMCL получает очищенный 2D-скан и
одометрию FAST-LIO через TF, исправляя привязку к статической карте.
TF: `map → lio_odom → odom → base_footprint`; AMCL и мост имеют разные рёбра.
Колёсный `/odom` и драйверный TF сохраняются.

`/Odometry` → bridge → `/localization/lio_odometry` → коррекция AMCL →
**`/localization/kinematic_state`** (`nav_msgs/Odometry`, frame `map`,
child `base_footprint`) → детектор и real_observations → `/navigation/self`
→ decision manager, планировщик и MPPI.
Скорость в итоговом сообщении берётся из FAST-LIO, а не из скачков коррекции.

Начальная поза подаётся однократно в `/initialpose` после активации AMCL
с ненулевой covariance: `initial_position_std` и `initial_yaw_std` в
`localization.yaml`. Это позволяет уточнять старт даже без движения.

Готовность требует свежих итоговой позы, LiDAR, IMU, колёсной одометрии и TF,
а также свежей оценки AMCL с допустимой неопределённостью. Без схождения
AMCL итоговая поза не публикуется, движение остаётся закрытым.
Covariance — приближённая оценка AMCL с локальной добавкой, не гарантия точности.
Режим `localization: amcl` использует прежнюю колёсную одометрию.
Симуляция по-прежнему использует точную позу Gazebo.

Пересборка: `helm build_real`; запуск: `helm start_real`, затем
`helm enable_real`; пауза: `helm pause_real`; остановка: `helm stop_real`.

### Проверка плоских фрагментов в детекторе

В feature/egor-detector-fix грани сравниваются с окружностью по одному ободу
после удаления выбросов. Параметры `robot.plane_ratio: 0.8`,
`plane_min_improvement: 0.015` м и `plane_min_arc: 1.5707963267948966` рад
заданы в профилях `jr_perception` и bag-конфигах `jr_launch`.
Для короткой видимой дуги этот дополнительный запрет не применяется.
`plane_ratio: 0` выключает проверку. Наши 12 точек, видимость и tracker сохранены.
Прямой порог1.5 из main отклонён после проверки на реальных bags.
Сравнение: `results/detector-plane-adapted-20261003/report.md`;
оно использует записанную AMCL-позу через PoseRelay и не оценивает FAST-LIO2.
