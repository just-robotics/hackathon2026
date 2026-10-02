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
локализации `localization.yaml`, фильтра `lidar_filter.yaml`.
`start_real_bag_record` — драйверы, raw MCAP и клавиатура через watchdog;
автономных издателей команд в этом режиме нет. Сценария построения карты,
LIO-SAM/FAST-LIO и submodules нет. [REAL_ROBOT.md](REAL_ROBOT.md) — руководство.

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
- Радиус планирования и costmap 0,23 м: тело Kobuki 0,178 м + запас 0,052 м.
  Не уменьшай footprint для обхода отказа. A* проверяет swept edges, включая
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
Real использует отдельную maze_bag_v1 и AMCL: wheel odom + LiDAR LaserScan
→ map→odom → navigation/self. Потеря localization/ready закрывает движение.
Не переносить sim truth/map/referee или смещение мира в hardware-стек.

- `/map` и `navigation/known_grid` статические, входы детектора и StaticLayer.
- Planner фильтрует неизвестные препятствия, публикует obstacle_scan.
  Тот же набор обновляет ObstacleMemory/A* и штатный cloud ObstacleLayer
  MPPI с marking+clearing. **Не подавать obstacle_grid в StaticLayer MPPI.**
- obstacle_grid — представление статических стен и памяти LiDAR до 8 с
  для глобального планирования/диагностики. Видимые свободные лучи очищают
  память, невидимые записи истекают. Это не SLAM.
- По указанию пользователя коробки 15×15×40 см можно толкать и игнорировать
  после распознавания по форме, без fixture poses. 40×60×20 см учитывать.
  Стены и неоднозначные кластеры сохраняются. Fresh measured opponent≤0,3 с
  защищается от классификации как коробка; устаревший прогноз — нет.
- `config/simulation_obstacles.yaml`: 3 узкие подвижные коробки, 1 широкая
  неподвижная. Физическая коллизия сохранена, mass/friction приблизительные.
  small_box_contacts отдельно; общий collisions всё ещё включает толкания.
- Guardian исключает текущего соперника из препятствий для поимки. Ложный
  трек коробки при этом опасен; не считать проблему решённой.

Real LiDAR: raw `/livox/lidar` → независимый от TF real_lidar_filter →
`/sensing/lidar/points_filtered` → AMCL и real_observations → scan →
detector/planner → obstacle_scan → MPPI. Сырой cloud сохраняется.
Узкие угловые маски с ограниченным ближним диапазоном в lidar_filter.yaml;
не увеличивай общий радиальный blind zone. Фильтр помог на оборудовании
лишь частично по отзыву пользователя; нового post-filter bag нет.

## Детектор

Python/NumPy из feature/detector b985515; segmentation.py/tracker.py
не менялись, ROS-адаптация в node.py/core.py. Входы: scan в map, own pose,
static known_grid и TF сенсора на stamp. Observed grid и fixture poses
не входы. Стационарный соперник может открывать трек. Prediction не освежает
Odometry/visible. Скорость публикуется в child frame; курс движения при
reverse не равен направлению корпуса.

Real sensor_frame=livox, simulation=livox_frame. Профили в profiles.py;
real-профиль пока проверен offline без semantic labels. Simulation
strong_rectangle_ratio=0,70 понижает прямоугольные strong-кандидаты до weak;
для real выключен. Одна ложная strong + weak подтверждения всё ещё могут
создать ложный трек. Происхождение и ограничения — hsl_perception/SOURCE.md.
Enable real требует свежей обработки detector, а не присутствия соперника.
Без карты detector не готов и автономное разрешение остаётся закрытым.

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
