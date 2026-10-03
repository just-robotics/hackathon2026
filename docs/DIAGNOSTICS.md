# Диагностика и испытания

Команды выполняются из корня репозитория. Текущее состояние — в [PROJECT_STATUS.md](PROJECT_STATUS.md).

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

При `mppi_diagnostics.result=swept_collision` сохраняются отклонённая поза,
индекс rollout и `rejected_cells`: до8 уникальных запрещающих клеток на
растеризованном контуре, координаты центров, master cost и costs отдельных
слоёв. `layers` включает StaticLayer и динамический ObstacleLayer; `null`
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

`hsl_perception_cpp/opponent_detector` использует алгоритм из `feature/detector`
(b985515): сегментация, подгонка окружности корпуса Kobuki и несколько треков
с фильтром Калмана постоянной скорости. Production-реализация C++/Eigen; прежний C++
детектор сохранён в Git checkpoint 4f9168c. Полная ветка не сливается:
симуляционный мир, MPPI и запуск остаются текущими.

Входы каждого детектора: свои `navigation/scan` (map, timestamped TF),
`navigation/self`, статический `navigation/known_grid`. Неизвестные коробки
не входят в фон. Выходы: `navigation/opponent` (Odometry, скорость в его
локальном frame), `opponent_visible`, `detector_cycle_ms`, `detector_diagnostics`
(JSON с причинами отказа, кандидатами и состоянием треков). Соперник из Gazebo
не подаётся алгоритму. Прогноз без новой детекции не обновляет Odometry/видимость.
Стационарный робот может открывать трек. Подтверждение требует трёх strong hits; weak-продолжения только поддерживают
трек. Причины перечислены в native JSON; Python detector/oracle удалён.
Ложные геометрические совпадения всё ещё требуют проверки на реальных данных.

`perception.opponent_max_height` в match YAML ограничивает максимальную высоту
кластера (default0.46м); остальные параметры формы `robot.*` и трекера `tracker.*`
объявлены ROS-параметрами. В симуляции включена проверка зазора между
пластинами `robot.max_gap_share=0.12` и более строгий fit окружности
`robot.line_ratio=0.35`; real-профиль задаёт max_gap_share=0.25,
line_ratio=0.50, strong extent≥0.25, arc≥75°, inlier≥0.80. Он проверен
offline на неразмеченных bags; устойчивость на оборудовании ещё не подтверждена. Для открытия трека simulation также требует
дугу≥90° и≥95% согласованных точек обода; из большого смешанного кластера
допускается только продолжение уже известного трека. Это уменьшает ложные
открытия на коробках, но затрудняет первое обнаружение при сильном перекрытии.
В адаптации фон вычитается по occupancy grid с
запасом0.08м вместо аналитических боксов SDF из исходной ветки. Точность
реальных данных и переносимость требуется проверять отдельно.

Для записи полных симуляционных облаков обоих детекторов добавьте к изолированной
проверке `--unknown-obstacle --record-detector-scans`. В `NN-obstacle.json`
сохраняются height-filtered map-frame points, карта и truth-метки только
для offline-оценки. Они не подаются в навигацию.

Offline-повтор записанных облаков через текущий NumPy core:
```bash
python3 benchmarks/replay_detector_clouds.py results/isolated/hsl-eval/series-20261001T191042Z/00-obstacle.json --height 0.46 --output /tmp/detector-replay.json
```
Нужны Python3 с NumPy и собранный C++ detector_replay (source workspace
или HSL_DETECTOR_REPLAY). Truth используется только для labels результата,
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


## Аудит реального bag без запуска робота

```bash
docker run --rm --network none \
  -v "$PWD:/work" -v /path/to/recordings:/bags:ro \
  --entrypoint bash jr_real_image:latest -lc \
  'source /solution/install/setup.bash; python3 /work/benchmarks/audit_real_bags.py /bags --output /work/results/real-bags-audit --period 0.2 --filter-lidar'
python3 benchmarks/report_real_bags.py results/real-bags-audit
```

`--filter-lidar` повторяет production-фильтр сырых облаков перед TF. Входы
read-only, DDS не используется. JSON/NPZ/PNG содержат измеренное движение,
recovery, гипотезы и восстановленные облака. Нет разметки соперника/коробок —
число гипотез не precision/recall; sampled replay не подтверждает трекер
на полной частоте. Сессии без LiDAR/TF пригодны лишь для частичной диагностики.

`benchmarks/replay_real_start.py` — неподвижный ROS-снимок запуска, не езда:
фиксированная записанная поза, нет драйверов/AMCL/финального cmd_vel.
`--costmap-source static` соответствует текущему MPPI, `memory` — вариант
для диагностического сравнения. Такой тест не подтверждает физическое
устранение застревания и не заменяет новую запись реального заезда.

Новые проверки переноса: `benchmarks/check_lidar_cpp.py` (byte parity),
`check_cpp_detector_ros.py`
(ROS contract/visibility expiry). `audit_lidar_pipeline.py` проверяет
свежесть записанных датчиков/TF. `replay_real_transport.py` запускает реальный
bringup с записанными raw LiDAR/odom, без драйверов и разрешения движения;
AMCL получает очищенное облако, записанный map→odom не воспроизводится.
`check_semantic_costmap.py` проверяет удаление ранее отмеченной маленькой
коробки и сохранение большой коробки/статической стены.

### Планирование на записанных датчиках без движения

`benchmarks/replay_real_transport.py` использует raw LiDAR, wheel odom и только
odom TF из bag, current AMCL/map/filter/planning. По умолчанию replay проверяет
свежесть наблюдений и согласованность верхних LiDAR-возвратов с картой.
`--planning-target X Y` дополнительно заменяет decision/referee фиксированной
целью только внутри стенда. Работает только в network-none контейнере без
драйверов; motion_gate слушает отдельный постоянно закрытый active, final
cmd_vel должен быть нулевым. Это вычислительная проверка, не езда по bag.

```bash
docker run --rm --network none -v "$PWD:/work" -v /path/to/recordings:/bags:ro   --entrypoint bash jr_real_image:latest -c   'source /solution/install/setup.bash; python3 /work/benchmarks/replay_real_transport.py /bags/SESSION --seconds 35 --planning-target 0.5 3.5 --output /work/results/planning-probe.json'
```

Смотрите diagnostics/global_paths/nonzero_mppi_commands и отдельно
nonzero_final_commands. Количество ненулевых MPPI-команд и их средняя величина
не являются измеренной скоростью робота. Без движения recorded pose не
реагирует на команды, поэтому recovery/выход на цель так не оцениваются.
`sensor_replay` содержит длительность исходных выбранных датчиков и поданного
окна, число исходных/поданных сообщений по топикам и `complete_sensor_window`.
Проверяйте покрытие записи отдельно от числа полученных наблюдений: очередь
может пропускать промежуточные сканы, сохраняя свежий последний скан.

При разборе формы объектов `audit_real_bags.py --cloud-points 0` сохраняет
все XYZ выбранных кадров; `--cloud-points 6000` воспроизводит прежнее
прореживание для сравнения. Это offline-контроль геометрии на recorded TF,
а не production bringup/AMCL или замер производительности фильтра.

## Производительность detector

Историческое сравнение до удаления Python detector: bag151820,761 полное
облако,5повторов на i7-10510U/одном ядре, mean Python4,188мс/C++0,268мс,
p95 5,609/0,355мс, mean speedup15,62×. Outputs совпали. Python comparator
и oracle удалены; эти числа относятся к сохранённому эксперименту.

Текущие benchmark_detector.py/.cpp подготавливают raw bag и измеряют только
native Detector.step. TF/фильтр/декодирование/DDS/I/O вне timer. Фильтр
отдельно не замерять. Результат не является пропускной способностью ROS стека
или измерением на роботе. Общая цель остаётся paused; новые заезды не запускать
без запроса пользователя.
