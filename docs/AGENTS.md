# Инструкции для агентов

## Перед работой

- Прочитай [PROJECT_GOAL.md](PROJECT_GOAL.md), текущую сводку и последние циклы
  в [PROJECT_STATUS.md](PROJECT_STATUS.md), затем относящийся к задаче код.
- Проверь ветку, `git status`, работающие контейнеры. Сохраняй чужие
  незавершённые изменения. Записи прежних проверок не доказывают текущий код.
- После каждого завершённого цикла обновляй PROJECT_STATUS: гипотеза,
  изменение, команды/условия/seed, результаты, вывод и следующий шаг.
- При изменении запуска, архитектуры, зависимостей или ROS-интерфейсов
  синхронно обновляй README и соответствующее руководство.
- Не отправляй изменения в origin без запроса. Не записывай секреты в документы.
  Не удаляй чужие данные и записи. Удаляй подтверждённо неиспользуемый код
  и его настройки; сохраняй необходимые заготовки реальных интерфейсов.
- Требования соревнования сверяй с PROJECT_GOAL и PDF-регламентом.
  Рабочие выводы проекта не являются дополнительными правилами организаторов.

## Запуск и сборка

Симуляция: Docker/ROS 2 Humble/Gazebo Classic 11, образ `jr_image`.
`helm build duel` собирает один общий образ сервисов профиля.
После изменения `helm_launch/` переустанови CLI по инструкции README.
`jr_launch` устанавливает только tracked-каталог launch, config у пакета нет.

Основной запуск — `helm start_match`, настройки — внешний `config/match.yaml`.
Других start_matchN нет. Перед разрешением движения проверяются оба стека,
MPPI, параметры, исходники и готовность коробок. `--prepare-only` не открывает
движение. GUI/RViz при DISPLAY, иначе headless. `simulation`/`gazebo` —
сенсорный стенд, не оценочная автономная дуэль.

Real: отдельный CPU-образ `jr_real_image` на базе контейнера HSL25,
драйверы `drivers/src` в /workspace, решение `src` в /solution.
`helm build_real` не собирает Gazebo; sim build не обновляет real-образ.
`start_real` оставляет движение закрытым, `enable_real` проверяет готовность,
`pause_real` закрывает движение, `stop_real` завершает контейнер и bag штатно.
Конфиги оборудования `real.yaml`/Livox JSON, миссии `real_match.yaml`,
локализации `localization.yaml`, фильтра `lidar_filter.yaml`, планирования
`planning.yaml` (общий для sim/real, читается при запуске).
`start_real_bag_record` — драйверы, raw MCAP и клавиатура через watchdog;
автономных издателей команд в этом режиме нет. Сценария построения карты,
LIO-SAM и submodules нет; FAST-LIO2 vendored в src/fast_lio. [REAL_ROBOT.md](REAL_ROBOT.md) — руководство.

## Приоритет проверок на реальных данных

Основной критерий фильтра и детектора — воспроизведение реального raw LiDAR
из bag с настоящими объектами. Проверять всю цепочку до наблюдений и
препятствий планировщика. Unit fixtures, совпадение Python/C++ и Gazebo
дополняют проверку, но не подтверждают правильное распознавание реальных
объектов. Число свежих tracks не считать точностью/recall без разметки.
Для разбора robot/box конфликта дополнительно считать box hypothesis без
защиты текущим track: иначе сама защита скрывает возможную ошибку детектора.

## Архитектурный контракт

Каждый автономный робот имеет собственные наблюдения, detector, decision,
A*, costmap, штатный C++ Nav2 Humble MPPI 1.1.20, motion_gate и метрики.
MPC, Python MPPI, debug follower и scripted opponent удалены.
Не возвращай скрытые backend/fallback. Единственный издатель финального
cmd_vel — motion_gate. Отсутствие безопасного MPPI означает остановку.
До общего старта/после первого исхода команды обоих роботов закрыты.
Актуальность pose/scan/path/intent/command обязательна.

- MPPI выдаёт `navigation/mppi_cmd_vel`; `local_path` — визуальный префикс,
  вся оптимизированная траектория проходит swept-check.
- Радиус планирования и costmap задаётся planning.yaml: тело Kobuki 0,178 м + запас.
  Пользователь разрешил настройку запаса через planning.yaml; тело 0,178 м
  должно полностью помещаться в footprint. A* проверяет swept edges, включая
  диагонали; дополнительный local_safety_margin номинально 0.
- `motion.allow_reverse` касается только исследователя. Страж vx_min=0,
  PathAngle.forward_preference=true. PreferForward выключен. GoalAngle
  включается у стража только в CAPTURE. Пределы команд — motion YAML.
- PURSUE получает собственный `own_max_speed` из motion.max_speed.
  Скорость соперника измеряется по треку; не ограничивай её нашей скоростью.
- Цель исследователя — центр площадки с допуском 0,08 м (строже регламента).
  Завершение защёлкивается и не снимается шумом локализации. Размер площадки
  0,5×0,5 м, half_size=0,25. Конфиги sim/real независимы.

## Карта, наблюдения и неизвестные препятствия

Симуляционный мир `polygon_rosbag.world`, карта из его коллизий через общий
`jr_map/sdf_geometry.py`. Экспорт `tools/export_sdf_map.py`.
Старты `[0.5,0.5,0]`/`[0.5,3.5,0]`, map_origin_world `[-0.468,-0.582]`.
Ноль map внутри нижнего левого угла стен, yaw 0 вправо.
Real использует maze_bag_v1 для планирования и FAST-LIO2 для позы:
raw LiDAR + IMU → /Odometry → fastlio_bridge → /localization/lio_odometry
→ AMCL map correction → /localization/kinematic_state → real_observations
→ navigation/self. AMCL публикует map→lio_odom, мост lio_odom→odom.
Старт robot.start — начальное приближение; очищенный скан сопоставляется
со статической картой AMCL. Режим amcl использует колёсную одометрию. Потеря localization/ready закрывает движение.
Не переносить sim truth/map/referee или смещение мира в hardware-стек.

- `/map` и `navigation/known_grid` статические, входы детектора и StaticLayer.
- По текущему указанию пользователя костмапа **только статическая**:
  A* читает occupied known_grid, MPPI использует StaticLayer `/map` и
  InflationLayer. ObstacleLayer отключён. map_points, scan, obstacle_scan,
  ignored_obstacles и результаты классификации не меняют occupancy.
- obstacle_grid — копия статической карты для совместимости диагностики.
  Нет памяти/обновления динамических препятствий в активном планировании.
- В реальном launch активен `jr_perception/opponent_detector_cpp`: входы
  `/sensing/lidar/points_filtered`, `/map`, stamped TF сенсора и map-поза
  `/localization/kinematic_state` при FAST-LIO2 или `/navigation/self` иначе.
  Профиль `src/jr_perception/config/real_cpp.yaml`; один издатель
  `/opponent/odom`, только при подтверждённом измерении текущего скана.
  Ориентация корпуса неизвестна, tracking frame совмещён с осями `map`;
  надёжная скорость задаётся в этих осях. Прогноз только диагностический.
  Маркеры и `/navigation/detector_diagnostics` показывают кандидатов,
  причины решений и потери/замещение сканов. Алгоритмы и параметры
  распознавания не менять без предварительного согласования пользователя.
- Следующие пункты описывают старый Python-детектор `jr_perception` из
  feature/detector b985515, оставшийся в симуляционных сценариях.
- Вход: сырое `/livox/lidar`, timestamped `navigation/self`, TF базы/лидара,
  latched `/map` как background. Карта должна иметь нулевой поворот origin;
  фон стен в occupied cells >=50 до 0,70 м, пол z=0.
- Выходы: `opponent/odom` — один выбранный robot track с исходными
  twist/covariance; `opponent/markers` — MarkerArray кластеров, включая
  коробки, и треков. Decision и A* подписаны на `opponent/odom`.
- В namespace opponent выходы `/opponent/opponent/odom` и markers;
  колёсная одометрия второго робота `/opponent/wheel/odom`. Не смешивать
  её с детекцией первого робота `/opponent/odom`.
- Исходный tracker допускает coasting: свежий Odometry/visible не означает
  новое измерение. `navigation/detector_diagnostics.stamp_s` показывает
  успешную обработку скана, даже без выбранного соперника. Без карты,
  timestamped собственной позы или sensor TF обработка не готова.
- В feature/egor-detector-fix минимальная поддержка обода — 12 точек;
  недостаточный обод не обновляет трек слабой детекцией. Центры кандидатов
  и публикуемых треков проверяются по occupied-клеткам /map (>=50), без
  инфляции. Unknown/outside не блокируются. Проверяется видимая поверхность
  обода (>=50%, запас 8 см); закрытая линия до центра сама по себе допустима.
  Повторный захват требует 3 последовательных strong; память места — 1,5 с.
- Старые C++ detector, SmallBoxFilter, obstacle memory и semantic layer
  удалены. Маркеры новой ветки не классифицируют размеры коробок.
- `config/simulation_obstacles.yaml`: 3 узкие и 1 широкая подвижные коробки.
  Физические коллизии сохранены, mass/friction приблизительные.
  Ни одна коробка вне статической карты не обновляет occupancy.

Real FAST-LIO2: raw `/livox/lidar` → livox_custom → FAST-LIO2 вместе с IMU.
Детектор получает очищенное облако и `/localization/kinematic_state`.
Параллельно raw → C++ hsl_lidar_filter → `/sensing/lidar/points_filtered`
→ AMCL и real_observations → navigation/scan. Сырое облако не подавать в AMCL.
Python фильтр сохранён как offline oracle; фильтр отдельно не замерять.
`jr_perception/SOURCE.md` описывает происхождение и ограничения адаптации.
Для текущей задачи проверка детектора проводится только на реальных bag:
не использовать симуляцию, оборудование или SSH. Изменения
алгоритма/параметров распознавания согласованы 03.10.2026 для нового C++ подхода.

## Испытания и передача

[DIAGNOSTICS.md](DIAGNOSTICS.md) — команды, ROS-топики, replay и графики.
Для CLI есть helm_launch/tests, для алгоритмов tests; после изменения Docker
проверь Compose и реальную сборку. Для поведения нужны ROS/Gazebo-прогоны.

Runner сохраняет image IDs, source hashes, параметры и YAML каждого заезда.
Не отключай проверку устаревших образов. Отчёты двух ролей должны относиться
к одному referee active window; ручной останов не полный оценочный матч.
Ground truth соперника разрешён только в referee/метриках/offline labels.
Средняя скорость — за всё активное время. Ориентир ≥0,2 м/с, далее ≥0,3;
максимальные команды этим порогом не ограничены. Финальная серия ≥20 матчей,
360 с active, разными записанными seed при одинаковых стартах; не подбирать
seed ради исхода и не подгонять поведение под 50/50.

Изолированные серии используют отдельные project/domain/Gazebo port.
Удаляй только свой подтверждённый runtime; не измеряй RTF рядом с другим миром.
MCAP в hsl2026Extra read-only, артефакты в ignored results. Нет разметки
коробок/соперника — число гипотез не precision/recall. Offline и модульные
проверки не подтверждают физическое достижение цели/поимку. Исторические
серии старого мира/контроллера/детектора не оценивают текущую версию.

FAST-LIO2 vendored в src/fast_lio; COLCON_IGNORE исключает его из sim.
Dockerfile.real удаляет маркер и собирает пакет. Параметры config/fastlio.yaml.
Монитор готовности FAST-LIO2 проверяет свежесть позы/TF/IMU/LiDAR/odom,
требует свежую AMCL-оценку с допустимой covariance и итоговый kinematic_state;
после активации однократно подаёт robot.start через initialpose с ненулевой
covariance, а set_initial_pose=false избегает нулевой поддержки старта;
номинальная covariance нескорректированного моста не означает точность карты.
Последний перенос в main-based дерево сделан без проверок по просьбе пользователя.

Проверка граней после согласования адаптирована только к ободу/inliers:
plane_ratio=.8, plane_min_improvement=.015м, plane_min_arc=pi/2.
Важны реальные bags: исходный прямой перенос1.5 дал0выходов на4bags.
Длинные паузы сохраняются, нет semantic labels; не считать уменьшение
числа кандидатов доказанным ростом precision. BoxTracker/linefit не перенесены.
