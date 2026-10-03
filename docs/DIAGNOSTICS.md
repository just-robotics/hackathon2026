# Диагностика и испытания

Команды выполняются из корня репозитория. Текущее состояние — в [PROJECT_STATUS.md](PROJECT_STATUS.md).

## ROS-интерфейс duel

Первый стек использует `/navigation/...`, второй — `/opponent/navigation/...`:

| Интерфейс | Тип и назначение |
| --- | --- |
| navigation/self / opponent/odom | Odometry: собственная локализация и наблюдаемый трек соперника |
| scan / map_points / known_grid | PointCloud2 / OccupancyGrid: наблюдения и статическая карта |
| opponent/markers | MarkerArray: кластеры объектов, включая коробки, и треки |
| intent | PlanningIntent: задача от decision manager |
| global_path / nav2_reference / local_path | Path: A*, ссылка MPPI, визуальный префикс rollout |
| mppi_cmd_vel | Twist: проверенная команда MPPI |
| planner_status / global_status | String: состояние локального/глобального планирования |
| planning_diagnostics / mppi_diagnostics | String JSON: входы/выбор пути и результат MPPI |
| control_cycle_ms / native_mppi_cycle_ms | Float32: вычислительные задержки |

При `mppi_diagnostics.result=swept_collision` сохраняются отклонённая поза,
индекс rollout и `rejected_cells`: до8 уникальных запрещающих клеток на
растеризованном контуре, координаты центров, master cost и costs отдельных
слоёв. `layers` показывает StaticLayer; динамический ObstacleLayer отключён; `null`
означает недоступный в этот момент mutex/координаты слоя, не свободную клетку.
За пределами costmap список может быть пустым. Поле диагностическое:
решение о столкновении по-прежнему принимает штатный footprint checker.

Общие `/match/active`, `/match/outcome` задают активное окно и первый исход.
Разрешения `/match/allow_motion` и `/opponent/match/allow_motion` (SetBool)
нужны обоим. Финальные `/cmd_vel` и `/opponent/cmd_vel` раздельны.
Отчёты referee и обоих роботов сохраняются в `results/` и относятся к
одному активному окну; разрешение только одному не запускает движение.

## Оценка и диагностика

```bash
python3 -m pytest -q tests
(cd helm_launch/tests && python3 -m pytest -q tests.py)
python3 benchmarks/run_duel_series.py --runs 3 --start-seed 19 --first-role guardian --active-s 90 --trace --audit-start
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
[PROJECT_GOAL.md](PROJECT_GOAL.md) и [PROJECT_STATUS.md](PROJECT_STATUS.md).

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

В реальном launch активен `jr_perception/opponent_detector_cpp`. Он читает
`/sensing/lidar/points_filtered`, `/map` и собственную позу с исходной меткой
времени. `/opponent/odom` содержит только подтверждённое измерение текущего
облака; `/navigation/opponent_visible` означает то же событие. Предсказание
публикуется только как отдельный синий маркер и JSON-поле `prediction`.
`/navigation/detector_diagnostics` содержит `stamp_s` последнего успешно
обработанного скана, `pose_selection`, `pose_gap_s`, статус, число заменённых
и отброшенных сканов, счёт кандидатов и время `total_ms`. Если `stamp_s`
перестаёт обновляться, проверьте карту, timestamped собственную позу и TF
сенсора. В реальном launch один издатель `/opponent/odom`.

`map_alignment_inconsistent` означает консервативный отказ при низкой доле
точек, совпавших с картой; `map_alignment_share` показывает эту долю.
Порог задают `min_map_alignment_share` и `min_map_alignment_points` в
`real_cpp.yaml`. Этот тест сам по себе не доказывает ошибку позы; карта
может быть перекрыта объектами. Цель и история подтверждения при отказе
сбрасываются. `temporal_change` и `complete_body_viable` в кандидатах
помогают разбирать дополнительную ветку движущихся объектов.

Просмотр одного real bag с текущим детектором и обычным RViz:

```bash
helm replay_detector             # консольный выбор
helm replay_detector --list
helm replay_detector --bag 20261003T095806.834495Z-bag --rate 0.5
```

Настройки и различие map/local-odom записей — в
[REAL_ROBOT.md](REAL_ROBOT.md#просмотр-детектора-на-реальном-bag).

Описание ниже относится к старому Python-детектору, который остаётся в
симуляционных сценариях и исторических проверках.

`jr_perception/robot_detector.py` — исходный детектор feature/detector b985515.
Входы: raw `/livox/lidar`, `navigation/self`, TF сенсора и `/map` (статический
фон). Выходы: `opponent/odom`, `opponent/markers` (MarkerArray),
`opponent/foreground`, `navigation/opponent_visible`,
`navigation/detector_diagnostics` (stamp_s, backend, costmap_source).
Второй стек использует namespace opponent. Колёсный odom второго робота
перенесён в `/opponent/wheel/odom`, чтобы не конкурировать с детекцией.

Odometry содержит только выбранный robot track. Маркеры содержат кластеры
объектов, включая коробки, без классификации маленькая/большая.
Исходный Kalman tracker допускает прогноз до max_coast; свежий выход/visible
не равнозначен новой измеренной детекции. Карта остаётся static-only.

Параметры распознавания находятся в `src/jr_perception/config/`, скопированы
из ветки; перед изменением необходимо спросить пользователя.
Старые detector/box replay и benchmark удалены вместе с заменённым кодом.

### Собственная скорость в модели перехвата

`motion.max_speed` из внешнего match YAML передаётся MPPI, глобальному
planner и обоим decision manager как `own_max_speed`. PURSUE использует
это значение как собственную возможность движения в модели встречи;
фактическая скорость на повороте может быть ниже. Это не дополнительный
предел команд. Скорость соперника берётся из LiDAR-трека и не ограничивается
значением из нашего конфига. Runtime audit сверяет `own_max_speed` у обеих
ролей перед разрешением движения.


## Новые реальные записи

Последние bag: `/home/eddyswens/ROS/hsl2026Extra/recordings_last8`.
Первая запись `20261003T093554.608208Z-autonomous` длится около259с,
содержит raw LiDAR, собственную map-позу, TF и статическую карту; по описанию
пользователя собственный робот ездит вокруг другого робота.
Bag-only записи содержат odom/TF/raw, но не записанную map-позу.
Нельзя объявлять их map-localized без отдельного восстановления локализации.

После подключения пользователь отменил прогоны. Детектор не проверен
на новых bag; сборка и проверки launch не подтверждают качество распознавания.
Записи read-only. Для следующего replay использовать raw LiDAR и исходные
позы/TF на stamp; не подменять детекцию truth и не менять пороги без согласования.

Для отдельного набора коробок, без правки основного YAML:

```bash
python3 benchmarks/run_duel_series.py --isolated-project hsl-joint-check --ros-domain-id 76 --gazebo-port 11421 --runs 1 --start-seed 3 --first-role explorer --active-s 90 --trace --record-detector-scans --config benchmarks/scenarios/narrow_diagonal_match.yaml --obstacles-config benchmarks/scenarios/narrow_diagonal_obstacles.yaml
```

Runner записывает SHA выбранного obstacles YAML; напрямую Compose использует
`HSL_SIM_OBSTACLES_FILE` (абсолютный путь). Этот fixture имеет обход: достижение
цели само по себе не подтверждает проезд через узкое место.
Текущий режим static-only: `obstacle_grid.data` совпадает с known_grid;
диагностические облака старого классификатора удалены. Проверки обхода
неизвестных коробок в этом режиме не подтверждают работу будущего динамического слоя.

Для private серии на одном компьютере при нестабильном Wi-Fi/VPN можно
задать DDS loopback только этой команде (обычный и hardware запуск не меняется):

```bash
export HSL_CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
# Затем run_duel_series.py; после серии:
unset HSL_CYCLONEDDS_URI
```
