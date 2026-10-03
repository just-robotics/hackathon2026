# Текущее состояние проекта

Обновлено: 2026-10-03. Цель — в [PROJECT_GOAL.md](PROJECT_GOAL.md), правила — в [AGENTS.md](AGENTS.md), запуск — в [README.md](../README.md).

## Актуальная сводка дерева — 03.10.2026

HEAD и origin/feature/decision-manager: fb98475. Есть незакоммиченные
изменения; прежние результаты ниже относятся к своим версиям.

- Приоритет последнего запроса — реальная сборка, без новых прогонов.
- Подключён `jr_perception` из origin/feature/detector b985515 напрямую.
  Сегментация, tracker и recognition YAML не изменены; ROS-интеграция
  добавляет фон из `/map`, готовность и свежесть выходного трека.
- Raw `/livox/lidar` → detector → `/opponent/odom` → decision/A*/MPPI.
  `/opponent/markers` отображает кластеры объектов, включая коробки.
  Отдельного adapter node нет; старые hsl_perception(_cpp), SmallBoxFilter,
  obstacle memory, semantic layer и их специальные инструменты удалены.
- AMCL сохраняет очищенное облако от C++ hsl_lidar_filter.
- Костмапа static-only: A* known_grid, MPPI StaticLayer+InflationLayer.
  Параметры MPPI из локального planning.yaml сохранены (radius0,6/scaling2).
  Радиус контура пользователя0,20м сохранён; удалены только настройки старой
  памяти препятствий и подтверждения маленьких коробок.
- Реальный образ jr_real_image собран, установленный launch проверен без
  запуска ROS nodes. Новые bag/replay/физическая езда текущей версии
  не проверялись по последнему указанию пользователя.
- Детектор исходной ветки может coasting-предсказывать потерянный трек;
  fresh Odometry/visible не равно новой измеренной детекции. MarkerArray
  показывает коробки как кластеры, не классифицирует маленькая/большая.
  Перед изменением алгоритмов/порогов детектора спросить пользователя.

## 02.10.2026 — общий checkpoint

По запросу пользователя закоммичены все текущие исходники, конфиги, тесты,
инструменты replay и документация. Проверки — в разделе актуализации ниже;
новых изменений алгоритмов при создании checkpoint нет. Пуш не выполнялся.
Игнорируемые bag/results не включены в Git. Открытые дефекты сохранены
в актуальной сводке; checkpoint не объявляет их устранёнными.

## 02.10.2026 — актуализация проекта и перенос исправлений сборки

Источник: origin/feature/fail_fix ff8426f (пользователь назвал feature/fix-fail),
исправления из 0932993. После fetch проверены все три отличия:

1. helm build выбирает один сервис общего образа профиля вместо параллельного
   экспорта одного тега для каждого сервиса. Развёрнутый Compose подтверждает
   одинаковый image/build у всех build-сервисов duel/simulation/all.
2. prepare_xauthority не копирует файл в самого себя при вложенном вызове
   helm; samefile также обрабатывает symlink. Свежая сессия обновляет cookie.
3. jr_launch устанавливает только launch: config отсутствует в tracked-файлах
   и раньше мог мешать чистой сборке другого checkout.

README сокращён до запуска и актуальной архитектуры; команды диагностики
вынесены в DIAGNOSTICS.md. AGENTS/PROJECT_GOAL/промпт продолжения обновлены:
убраны старый мир/C++ detector/«real не подключён», прописаны владение слоями
costmap и открытые дефекты. История результатов сохранена в PROJECT_STATUS.
Удалён неиспользуемый USE_PERCEPTION из .env (empty_perception отсутствует).
Рабочие изменения до начала сохранены в /tmp/hsl-before-cleanup-saww6ik9
(tracked.patch + untracked.tar.gz); это временная локальная копия.

Проверки:
- helm build duel: успешная сборка jr_image,
  sha256:9e9c8642584072650c573deb4050700c0634970acb9cd1a1dc0eb632d2fda775;
  один export тега в логе /tmp/hsl-cleanup-build.log.
- python3 -m pytest -q tests: 169 passed, 1 skipped (host без ROS).
- cd helm_launch/tests && python3 -m pytest -q tests.py: 43 passed,
  включая повторный Xauthority/symlink и обновление cookie.
- Compose duel/simulation/all/real валиден. Исправления не меняют миссионные
  длительности и стартовые позы, реальный робот не запускался.

Проверка запуска:
```bash
python3 benchmarks/run_duel_series.py --isolated-project hsl-build-smoke --ros-domain-id 78 --gazebo-port 11423 --runs 1 --start-seed 0 --active-s 10 --trace --audit-start
```
Артефакты: results/isolated/hsl-build-smoke/series-20261002T201649Z.
Один старт потребовал автоматического retry готовности (см. retry1-up.log);
после повтора оба MPPI активированы, статическая карта и 4 коробки загружены,
runtime audit и запрет до общего старта пройдены. Итог — timeout 10 с,
не навигационный успех: скорости explorer/guardian 0,242/0,225 м/с,
контактов 0/0. Это только проверка собранного запуска, не оценочная серия.
Runner удалил свой duel; оставшиеся init/daemon этого проекта удалены вручную,
чужие контейнеры не затронуты. compileall, print-env и локальные Markdown-ссылки
обновлённых руководств проверены; git diff --check без ошибок.

Перенос выполнен точечно: полного merge ветки и подмены текущего дерева нет.
Сборка подтверждена на текущем компьютере с cache базовых Docker-слоёв;
чистая загрузка зависимостей и запуск на другом железе здесь не проверены.
Следующий шаг разработки — дефекты в актуальной сводке выше.

## 02.10.2026 — перенос feature/detector и отладка в Gazebo

Источник origin/feature/detector b985515fde5cceb13e8cf1901183d957ab7d773f.
Сегментация/Tracker CV Kalman перенесены без изменения исходных файлов;
новый ROS adapter hsl_perception/node.py использует прежние scan/self/known_grid,
static occupancy фон вместо SDF моделей, TF сенсора на stamp, local-frame twist.
Предыдущий CPP detector/test удалены, checkpoint4f9168c доступен в Git.
Нет полного merge чужой ветки/подмены world или truth оппонента в навигации.

Цикл1: queue TF из8 сканов вместо отбрасывания до прихода трансформации;
стационарный робот подтверждается3 strong hits, prediction без det не публикует
fresh Odometry/visible. Новые diagnostics + trace timing/причины. 145tests passed.
Первый предварительный стенд без движения остановлен до исхода, не оценивается.

Цикл2: source-профиль в hsl-detector-check/domain76/Gazebo11421, seed0,
4 штатные unknown коробки,90с active. Исход timeout; скорости explorer0.207,
guardian0.007м/с: страж выбирал коробку/ложный трек. Baseline серия прервана
после1 полного матча; следующий неполный не оценивать. Артефакты
results/isolated/hsl-detector-check/series-20261002T145425Z. Position error
p90 explorer1.882м/guardian2.291м; median0.087/0.049. Cycle median3.759/6.558мс,
p90 9.789/12.827мс. Не выдавать меньшую median за отсутствие ложных треков.

Записаны128 полных map-clouds обеих ролей + own/peer eval poses/sensor/map:
results/detector-integration-20261002/clouds-baseline.json. Evaluation only.
Replay baseline:29 correct/21 false/78 none (proximity<0.3м, не visibility recall).
Solid угол15см коробки давал strong fit окружности. Existing gap-test source
max_gap_share0.12:77 correct/0 false/51 none на этом же фрагменте. Min_extent0.22
ухудшает partial robot; оставлен0. Изменение профиля только simulation launch;
real default gap1.0 сохранён. Ограничен BLAS1thread для маленьких Kalman матриц.
146tests passed. Повторные реальные симуляционные результаты ниже после сборки;
replay сам по себе не доказывает устойчивую дуэль и hardware не проверяет.

### Продолжение циклов переноса detector

Промежуточная gap0.12 серия20261002T150206Z: seeds0/1/2,
поимки22.0/23.5/22.4с, скорости explorer0.211/0.278/0.207,
guardian0.264/0.250/0.264, контакты0,RTF0.515–0.528. Но paired active-window
report выявил ложные box-near samples; поимки не считаются доказательством
качества detector. Profile line_ratio0.35 серия20261002T151111Z тоже дала
поимки, но explorer удерживал prestart ложный трек крупной низкой коробки.
У seed0 guardian error p90=0.039м, ложных0; explorer ложных105 samples.
Report_detector.py сравнивает stamp с интерполированной peer own pose,
строго внутри referee window; новые цифры не смешивать со старым trace
error, включавшим подготовку. Не называть proximity error semantic recall.

Запись начала seed2:168 full clouds с9.821simsec, clouds-start.json.
Источник круговой fit допускал выбор подмножества rim points. После
confidence checks simulation (arc90°, fraction0.95, mergedstrong=false)
replay этого старта не выдаёт ни одного ложного трека; mid fragment
clouds-gap сохраняет корректные обнаружения. Одно replay/одна удачная дуэль
не завершает проверку. Исходные segmentation/tracker byte-identicalb985515;
confidence adaptation в core.py, real defaults не усилены без hardware данных.

### Итоговая проверенная версия detector

Simulation confidence profile: gap0.12, line_ratio0.35, strong_arc_min_span_deg90,
allow_merged_strongfalse, strong_min_inlier_fraction0.95. Все дополнительные
проверки делают сомнительный кандидат weak, не открывая новый трек; source
segmentation/tracker не переписаны. Real launch сохраняет source defaults,
на роботе ничего не развёртывалось и jr_real_image не пересобирался.

Команда:

```bash
python3 benchmarks/run_duel_series.py --isolated-project hsl-detector-check --ros-domain-id 76 --gazebo-port 11421 --runs 3 --start-seed 0 --active-s 90 --trace --audit-start
```

Точные команды/условия также сохранены в run logs и match.yaml.
Последовательные seeds0/1/2, одинаковые старты, штатные4 коробки с разными yaw
из simulation_obstacles.yaml. Никаких навигационных teleport/truth opponent.
Серия results/isolated/hsl-detector-check/series-20261002T152625Z:

| Seed | Первый исход | Активное время, с | Explorer, м/с | Guardian, м/с | Контакты обоих | Ошибочные треки коробок обоих |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 0 | guardian_capture | 18.1 | 0.246 | 0.295 | 0 | 0 |
| 1 | guardian_capture | 24.1 | 0.254 | 0.241 | 0 | 0 |
| 2 | guardian_capture | 23.8 | 0.241 | 0.247 | 0 | 0 |

767 aligned detections внутри общего активного referee window: ни одного
position error>0.3м, ни одного box-near false sample. P90 позиции по6 парам
role/run0.022–0.045м; вычисление detector p90 10.34–17.34мс. Это ошибка
выданных треков; visibility recall не измерен. Слабые/перекрытые первые
наблюдения намеренно не открывают трек, пока корпус недостаточно различим.
Weighted скорость за суммарные66с:explorer≈0.247м/с,guardian≈0.258м/с.
Planner OK explorer0.834–0.924,guardian0.972–0.983; RTF0.527–0.552.
Подтверждены3 автономные поимки, штатный запрет до старта и стоп послефиниша,
source/parameter audit в обеих ролях. Цель explorer в этих3 дуэлях не достигнута;
это не подтверждение баланса ролей и не20-серия устойчивости навигации.

Образ jr_image:latest sha256:75589886b4f0b02bb3a5d781e1504eb72864105a1c6fc2d8fb144891a5316c2b.
148pytest passed, compileall, Compose simulation/real, diffcheck и установленный
launch audit (4 role/reverse cases, один detector в каждом namespace) прошли.
Изолированный мир каждого заезда убран runner; собственные init containers
удалены. Другие проекты не остановлены. Артефакты capture/replay/final-summary
в ignored results/detector-integration-20261002. Инструменты воспроизведения
benchmarks/capture_detector_clouds.py, replay_detector_clouds.py,
report_detector.py; последний использует только активное окно и interpolation
≤0.2с. Изменения пока не закоммичены/не отправлены.

Следующий шаг: другие расстановки и ориентации коробок, более длинные заезды,
проверка reacquire после сильного перекрытия, отдельно stationary distant robot.
Неподвижный робот/короткая дуга/merged cluster/time reset проверены unit-тестами,
но далёкое первое обнаружение при occlusion не объявлять экспериментально
доказанным. Новый конкурентный баланс и прохождение цели explorer требуют
отдельной серии; hardware detection требует своего профиля и реальной записи.

## 02.10.2026 — слияние feature/real-diagnostics в feature/decision-manager

Слиты source3242169 и target7c71cee. Сохранены polygon_rosbag, неизвестные
коробки, obstacle_memory, диагональная проверка и исходный detector.
Добавлены журнал смены действий, navigation/indication, запись behavior/indication
в существующий единственный autonomous MCAP recorder, laptop RViz и RDP-скрипт.
Goal completion latch и high-rate остановка real_match сохранены.
Конфликты: объединена документация и список топиков recorder; реальные старты
взяты из source[0.5,0.5,0]/[0.5,3.5,0], active_seconds600. Симуляционный
match.yaml не менялся. Обновлено ожидание полигона в test_real_robot.

Проверки: python3 -m pytest tests -q — 142 passed; compileall decision/real/
laptop_rviz, bash -n обоих новых скриптов, git diff --check и Compose config
для simulation/real прошли. Simulation Compose предупреждает о незаданном
VEHICLE_ID. Живой ROS/Gazebo и реальный робот при слиянии не запускались,
образы не пересобирались; RDP-скрипт не выполнялся. Для использования нового
кода требуется пересборка соответствующего образа. Следующий шаг — совместный
тест коробок после пользовательского detector. Коммит слияния локальный,
push не выполняется по запросу пользователя.

## 02.10.2026 — индикация событий регламента

`navigation/indication` публикует одну строку для судьи: роль, разрешение
движения, для исследователя расстояние до центра площадки и допуск 0,08 м,
для стража дистанцию, курс, линию по карте и итог поимки. Строка обновляется
каждые 0,2 с и пишется в журнал узла при изменении. Это не касание контура
площадки из регламента: программный критерий остаётся центром. Проверены
модульные случаи цели и трёх условий поимки. Живой робот не запускался.

## 02.10.2026 — журнал смены действий ролей

Decision manager пишет в журнал одну строку info при смене поведения или
причины: роль, имя действия, reason, своя поза, цель и расстояние до свежего
соперника. Повтор каждые 0,2 с не логируется. Топики intent и behavior не
меняются. Проверено только чтением кода и py_compile; живой матч не запускался.

## 02.10.2026 — статическая карта maze_bag_v1 построена

Вход /home/eddyswens/ROS/hsl2026Extra/maze_bag_v1:140.81с,1408 облаков,
28163 IMU; odom/TF отсутствуют. Источник смонтирован ro и не менялся,
SHA256 сохранены в config/maps/maze_bag_v1_quality.json. Metadata9/Lyrical
несовместим с Humble QoS parser, исправлен прямой read MCAP без переписи.
Аудит подтверждает monotonic stamps, FLOAT64 absolute point timestamps,
~10Гц/200Гц, IMU≈1g; пользователь описал upside-down LiDAR под верхней
крышкой как в симуляции. Точная монтажная калибровка в bag отсутствует.

Использован отдельный offline snapshot FAST-LIO2 с адаптером времени точек:
hsl_offline_mapping:maze-v1 (6eab6677), исходники/лицензии сохранены в ignored
results/real-bag-check-maze-v1/fastlio-source-used.tar.gz. В реальные/duel
launch или jr_real_image backend не возвращён. Первый запуск встретил права
выходного volume; исправлен uid/gid. Cyclone+localhost-only при networknone
выбрал lo дважды; заменён FastDDS внутри сетевой изоляции. Первый полный
replay зарегистрировал1405 сканов, но shutdown имел fclose(NULL)/SIGSEGV
из-за неоткрытого source Log. Повтор с writable Log mount завершился exit0,
все1408/28163 input доставлены,1405 registered/poses, unpaired0. Траектории
двух проходов совпали в начале/середине/конце (difference0); финальная карта
взята из чистого прохода. Не выдавать1405 за обработку каждого input scan.

Map: robust floor plane median abs residual0.00931м; высоты0.10…0.40м над
полом, range8м,2D nearest angular rays + log odds, free/occupied/unknown.
Нет hand-drawn стен или inflation в static grid. Дальний фон ограничен
bbox крупнейшего observed wall component+0.25м; полный uncropped результат
и3D сохранились. Конечная resolution0.05м,75×97,origin[-0.45,-0.70,0],
493 occupied/4842 free/1940 unknown. Реальный Humble Nav2 map_server прошёл
configure/activate и опубликовал ровно эти классы/cell counts в frame map.

Качество по повторным наблюдениям: на80.3% поздних wall points, попавших
в0.15м от ранних поверхностей, median residual0.01490м,p90=0.05324м.
Это consistency без ground truth. Траектория10.854м, end-start0.415м:
точное возвращение в старт не задано, это не доказанный drift. Loop closure
не выполнялся. Визуально стены согласованы, открытые/неизвестные участки
не достраивались. Physical localization/навигация по карте не проверены.

Доставлены config/maps/maze_bag_v1.yaml/pgm/png/preview, параметры и quality
JSON, README системы координат. PCD/trajectory/registered scans/логи в
results/real-bag-check-maze-v1/. Команды воспроизведения в OFFLINE_MAPPING.md.
Текущий real.yaml автоматически не переключён: нужны реальные стартовые
координаты в новой системе и проверенная привязка/локализация.
Следующий шаг — оценить геометрию на реальном лабиринте, задать стартовые
позы/arena_bounds, подключить карту и проверить локализацию отдельно.

## 02.10.2026 — подготовка офлайн-карты по будущему bag

Пользователь позже передаст новую запись. Подготовлены read-only
`tools/inspect_mapping_bag.py` и инструкция OFFLINE_MAPPING.md: состав сессии,
аудит топиков/типов/полей облака, timestamps, TF, odom/IMU, восстановление
траектории, проверка дрейфа, occupancy ray tracing и Nav2 YAML/PGM экспорт.
Работа выполняется отдельно от робота: docker --network none, cap-drop ALL,
входные файлы ro; ROS publishers/driver launch отсутствуют. SLAM-пакеты и
сценарий картографирования не возвращены. Реальный старт не менялся.

Аудит проверен в актуальном jr_real_image на static_1m из HSL26-ros2_bags:
все99 облаков и1992 IMU прочитаны, найдены XYZ/line/timestamp и base_link→livox,
явно отмечены отсутствие odom и median acceleration norm1.002 (возможные
единицы g). Exit0 при валидном LiDAR; отдельный тест отсутствующего LiDAR
вернул exit2 с JSON ошибкой. Первая попытка смонтировать скрипт как /inspect.py
затенила стандартный Python inspect; исправлено именем /audit_mapping_bag.py.
Артефакты results/real-bag-check-20261002/mapping-*.json (ignored).
py_compile/diff-check успешны. Входные bags не изменены.

Новая карта не построена: новые данные ещё не предоставлены. Следующий шаг —
получить всю сессию (metadata+все data files+конфиги), провести аудит,
выбрать/проверить способ регистрации траектории по реальному дрейфу,
экспортировать map с отчётом качества. Накопление по wheel odom нельзя
выдавать за проверенную карту лабиринта. Карта и локализация — разные задачи;
текущий static map→odom anchor не устраняет дрейф.

## 02.10.2026 — пользовательский фикс kobuki_core

В drivers/src/kobuki_core перенесены два изменённых заголовка из папки
kobuki_core_ fix. Остальные файлы совпадали с текущим драйвером. Исходный
комплект сохранён в /tmp/kobuki-core-speed-fix-20261002; дубликат убран из
корня. Лимиты скорости0.4м/с/3рад/с, ускорения0.3м/с²/3.5рад/с²,
торможения0.7м/с²/5.2рад/с². Знак движения учитывается; смена направления
сначала тормозит до нуля, затем ускоряется в другую сторону.

Адаптация: std::clamp заменён эквивалентным min/max, поскольку kobuki_core
и kobuki_node собираются с C++14. Kobuki::init передаёт все6 лимитов из
Parameters вместо неявных defaults. В общем hardware launch включён
acceleration_limiter для обоих real режимов: прежний false обходил весь
ограничитель. Параметры MPPI/ручного шлюза сохраняются; физическая скорость
дополнительно ограничивается предоставленными драйверными лимитами.
Симуляция не менялась. Проверка расчёта — tests/kobuki_acceleration_limiter.cpp,
без доступа к USB; физическая работа на базе пока не подтверждена.

Проверено: real образ успешно пересобран (5 driver +8 solution packages),
sha256:866f1be9e7255ae59406b540fc6748534e7f422ff347d356c8c56b5e7207a43b.
Standalone C++14 CMake-проверка в этом образе прошла: оба знака скорости,
ускорение/торможение, смена направления внутри/после шага до нуля,
нулевой/отрицательный dt, caps0.4/3.0. Первый ручной g++ probe с общим
списком include-директорий ошибочно перекрыл системные заголовки; он заменён
CMake find_package/ament_target_dependencies, итоговая проверка успешна.
Модульные проверки real:16 passed; compileall/diff-check успешны.
Повтор C++: source /solution/install/setup.bash; смонтировать tests в /checks;
cmake -S /checks/kobuki_limiter -B /tmp/check-build;
cmake --build /tmp/check-build; /tmp/check-build/check.
Действующий real контейнер не запускался: для применения нового образа
вызвать stop_real, затем start_real или start_real_bag_record.


## 02.10.2026 — разделение симуляционного и организационного real образов

Последнее указание пользователя заменяет прежний план картографии:
на роботе остаются только автономный start_real и start_real_bag_record.
Сценарий карты и SLAM не требуются. Симуляционный start_match/duel сохранён.

Добавленный комплект HSL25 содержит Dockerfile с базой
nickodema/kobuki:humble-22.04-100625. Образ получен, digest
2034fe21fbe8aae8677191967ec9c4fc168a1337ada7bb1913a642b92f2b8619
закреплён. В базе есть ROS2 Humble и Livox SDK2, workspace отсутствует:
нужно собрать предоставленные драйверы. Сравнение kobuki_core/kobuki_node/
livox_ros_driver2 из нового архива с drivers/src не выявило различий.
Полный исходный архив сохранён вне дерева в
/tmp/hsl25-provided-20261002-container; лицензии нужных пакетов в drivers/src.

Dockerfile.real теперь собирает drivers/src в /workspace и наше решение
в /solution на организационной базе, без зависимости от jr_image/Autoware.
build_real собирает только real образ. Compose и control/teleop используют
/solution/install/setup.bash. hsl_sim_adapter включён ради общих cloud/grid
утилит; симуляционные узлы не запускаются. В start_real сохранены MPPI,
wheel odom, запрет до enable, проверки готовности и остановка по таймауту.

Удалены LIO-SAM gitlink/.gitmodules, команда и скрипт submodules, конфиг
подмодулей, GTSAM/PPA из симуляционного Dockerfile, старые LIO launch/config,
FAST-LIO, map_session, map_record и их настройки. recording.yaml теперь
содержит только bag_topics и keyboard_timeout_s. История ниже описывает
прежние версии и не является инструкцией текущего запуска.

Проверки на новой базе (domain95, drivers_disabled=true, без оборудования):
- финальный jr_real_image:latest — sha256:abc55856e4bb0d3b163e0a18919cc8f1dea190d19fad7d57b1fa998db4505c15 (~4.29GB);
  runtime-проверки выполнены на предшествующем образе7f166149 с тем же кодом
  (финальная пересборка меняет только docstring record.launch); автономный transport audit подтвердил все8 проверок:
  native_ready, pose/scan в map, нулевые команды до enable, команды после
  enable, стоп на stale scan, стоп после12с активного этапа, один final publisher;
- manual audit: один real_manual_gate, cap0.5м/с и1.5рад/с, нули после watchdog;
- настоящая PTY teleop запущена, приняла i/k/Ctrl+C;
- replay static_1m из HSL26-ros2_bags: сохранены все99 облаков/1992 IMU,
  1922 cmd_vel и3 staticTF. MCAP50MiB,4016 сообщений,96.035с wall записи;
- stop_real закрывает шлюз, все5 процессов завершены cleanly, контейнер exit0,
  metadata.yaml создан штатно, session.final_export_confirmed=true,
  ros2 bag info читает запись; библиотеки обоих драйверов найдены;
-172 теста прошли, оба Compose config валидны, compileall/diff-check успешны;
  helm help содержит оба real режима, без map_record/submodules.

Команды проверки: tools/real_robot.py start|bag --drivers-disabled
(--no-keyboard для фонового replay); tools/real_robot.py enable|stop.
HSL_REAL_CONFIG=/tmp/hsl-organizer-config/real.yaml; ROS domain95,
HSL_REAL_DATA_DIR=results/real-record-check-20261002/organizer/sessions.
Артефакты: results/real-record-check-20261002/organizer/ (ignored).
Физическая база и LiDAR не подключены: hardware IO, wheel odom запись и
реальная езда не подтверждены. Следующий шаг — подключить оборудование,
проверить /odom, /livox/lidar, /livox/imu и TF, снять bag через штатные команды.
Без статической карты detector соперника остаётся неработоспособным;
новая локализация не добавлена. Симуляционный образ не пересобирался в этом
цикле; алгоритмы дуэли не менялись.

## 02.10.2026 — приоритет записи bag; картография отложена пользователем

Последнее указание: довести только start_real_bag_record, сценарий карты
дальше не менять. Локализация по будущей карте нужна внутри start_real позже.
Добавлены ручной шлюз (один final cmd_vel publisher), MCAP recorder,
внешний recording.yaml, уникальные recordings/<UTC>-bag, копии конфигов,
manifest и остановка по режиму работающего контейнера. Автономные decision,
planner, MPPI в записи не запускаются. Пределы ручной скорости — из mission
motion; клавиатура remapped в real/keyboard_cmd_vel, timeout0.6с.

Выявлен реальным тестом дефект остановки: Docker SIGTERM завершал launch
без metadata.yaml. Первый MCAP сохранён и восстановлен ros2 bag reindex,
этот тест не считается доказательством штатного сохранения. Исправление:
Compose stop_signal=SIGINT, grace90с; сначала manual_allow_motion=false,
после выхода проверяется фактическое наличие metadata.yaml. Во втором тесте
metadata создан штатно, ros2 bag info прочитал MCAP50MiB,99 облаков/1992 IMU
(весь static_1m источник),2025 cmd_vel/3 static TF. Проверка replay/domain94,
drivers_disabled=true: USB/физический проезд не проверены. Wheel odom/dynamic
TF в исходном bag отсутствуют, их запись настоящими драйверами пока не доказана.

PTY teleop стартовал и принял i/k/Ctrl+C. Отдельный ROS audit подтвердил
единственный final publisher real_manual_gate, cap0.5м/с и1.5рад/с,
последующие нулевые команды после прекращения входа. Исправлена попытка
publish после закрытия контекста при SIGINT; исправленный console executable
проверен SIGINT и завершился exit0 без traceback. Ранее проверка через
standalone ros2 run wrapper не передала сигнал дочернему узлу и закончилась
SIGKILL; она не считается проверкой шлюза. Рабочий сценарий запускается через
ROS launch, его штатное закрытие recorder подтверждено вторым bag тестом.
Модульные/CLI проверки:172 passed; Compose config/compileall/diff-check успешны.
Артефакты results/real-record-check-20261002/, не коммитятся.

Карта: до последнего уточнения добавлены FAST-LIO2 исходники src/fast_lio,
точные Livox timestamps, scaffold map launch/session. Static replay ранее
дал96 поз,spanXYZ2.83/5.04/2.29мм. Первый export оказался пустым из-за
закомментированного upstream накопления; накопление исправлено, но финальное
построение/экспорт карты ещё НЕ подтверждены. Не объявлять map сценарий готовым.
Дальнейшая его отладка остановлена пользователем. Сохранённый LIO-SAM
подмодуль остаётся optional; основная автономная локализация остаётся wheel odom.

## 02.10.2026 — смена кандидата картографии по уточнению пользователя

Пользователь остановил подключение LIO-SAM и разрешил выбрать более подходящую
систему. Выбран кандидат FAST-LIO2 (Ericsii ROS2/2fffc570), изучены IMU init,
Mid360 PointCloud2 preprocessing и map_save. Готовый quaternion не требуется,
но upstream mid360 handler вычисляет время по азимуту вместо нашего timestamp:
это нужно исправить до replay. Нет встроенной relocalization по сохранённой
карте и loop closure; эти задачи остаются явно отдельными. Сравнение, источники
и следующий цикл — историческое сравнение SLAM (кандидаты позже удалены).
Начатая LIO-SAM сборка остановлена, непроверенные LIO файлы/настройки убраны
из launch/сборки и сохранены в /tmp/hsl-lio-attempt-20261002. Старый подмодуль
src/lio_sam_src и helm submodules lio_sam сохранены. FAST-LIO пока не добавлен
в образ; команды bag/map записи ещё в работе. Основной real режим не менялся.

## 02.10.2026 — проверка реального bag для LIO-SAM

По запросу пользователя полностью прочитан static_1m из HSL26-ros2_bags:
99 PointCloud2 (10Гц),1992 IMU (200Гц),один static TF,17 rosout; нет wheel
odom/dynamic TF. Проверка read-only, без запуска робота/SLAM. Сравнение с
LIO-SAM из mpc_motion_control main/ac4ec563 выявило несовместимые line/timestamp
против ring/time, абсолютные наносекунды, line0..3 против N_SCAN1, IMU масштаб
около1g против gravity9.80665 старого YAML, livox против livox_frame без связи.
В этом bag quaternion всех IMU единичный, не нулевой; прежнее предположение
о нулевом quaternion к данной записи неприменимо. Статика не доказывает
корректную ориентацию при движении. Файл полностью читается несмотря на
предупреждение MCAP об отсутствии индексов. Исходные bags не изменены.
Подробности, команда, артефакты и следующий шаг:
аудит results/real-bag-check-20261002/static-1m-audit.json.
LIO-SAM пока не подключён, качество карты не подтверждено. Незавершённая
работа над bag/map recording остаётся в дереве: общий hardware.py/рефакторинг robot.launch.py; промежуточный recording.yaml
сохранён вне дерева при смене SLAM. Команды записи пока не реализованы.

## 02.10.2026 — отдельный запуск реального Kobuki/Mid-360

По запросу пользователя перенос HSL25 выполнен: пять нужных пакетов в
`drivers/src` (kobuki_core, kobuki_ros_interfaces, kobuki_node,
kobuki_description, livox_ros_driver2), USB-правило в drivers/udev,
сетевой JSON в config/livox_mid360.json. Исходной hsl25-master в корне нет;
неиспользованные материалы сохранены вне репозитория в /tmp/hsl25-unused-20261002.
Лицензии/авторство исходных пакетов сохранены. Старые mux/teleop/docking и
Docker-скрипты HSL25 не вошли в запуск. Основной симуляционный контур не менялся.

`hsl_real` адаптирует собственную одометрию и PointCloud2 к navigation/self,
scan и known_grid, использует прежние decision/planner/native MPPI/gate.
Отдельный Dockerfile.real собирает driver overlay /drivers поверх основного
образа. Добавлены явные apt dependencies ECL console/mobile robot: исходный
build без них падал, после исправления все пять driver packages собираются.
Для реального контейнера нет требования NVIDIA runtime, Gazebo/truth/referee.
Все узлы wall time; driver commands/velocity remapped на единственный cmd_vel.
Локализация по уточнению пользователя пока wheel odom + static map→odom от
robot.start. Облако преобразуется по timestamp с ожиданием TF, не latest.
Конфиг real.yaml/JSON читается без rebuild, параметры миссии из mission_file.

Команды: helm build_real/start_real/enable_real/pause_real/stop_real,
status_real/logs_real/enter_real. start_real пересоздаёт свой Compose hsl-real,
проверяет USB-порт и оставляет движение закрытым. enable требует свежих
pose/scan, native ready, незавершённый этап и одного final publisher.
Pause не расходует активное время. Supervisor закрывает этап по таймеру/центру
исследователя; guardian capture оценивается внешним судьёй, не фиктивным truth.
Stop работает при испорченном конфиге и ограничивает ожидание pause-service3s.
После завершения enable отвергается; новая одометрическая привязка требует
фактической стартовой позиции. Подробная инструкция: REAL_ROBOT.md; README и
AGENTS синхронизированы. Для real добавлен RViz с одним роботом/scan/путями.

Проверено:164 Python/helm tests, обе Compose config, compileall. Два полных helm build_real успешно собрали оба образа, включая финальный
повтор с отдельным RViz (/tmp/real-final-build.log, terminal0). Final image
smoke: installed robot.launch --show-args и наличие robot.rviz проходят;
SHA всех hsl_real Python modules в образе совпадают с текущим деревом.
ROS transport test: drivers_enabled=false, standalone container без /dev,
domain94, синтетические odom/cloud, active_seconds8. Все8checks pass:
pose/scan в map, native ready, нули до grant, ненулевые команды MPPI после
разрешения, stale scan stop, stage stop и один publisher. Pause/resume CLI
успешны, pause наблюдалась, повтор enable после finish отвергнут.
Все use_sim_time=false (native param query). ldd обоих driver executables не
содержит not found. Контейнер проверки остановлен; ROS input не поступает на
реальное железо. Артефакты results/real-bringup-20261002 (ignored), включая
fixture, source audit, результаты, runtime ID, CLI/launch/ldd logs.

Ограничения: USB-база здесь отсутствует, аппаратное подключение и физическая
езда НЕ проверены. Нет карты по текущему запросу: static layer выключен,
planner использует текущий scan/free frontier, накопленной карты/SLAM нет.
Текущий CPP detector требует known_grid, поэтому без карты распознавание
соперника/EVADE/PURSUE не работают. Не подставлялась свободная или Gazebo-карта.
Дрейф одометрии и mounting требуют проверки на базе. Следующее: подключить
робота, проверить USB/network/TF/scan, позже добавить реальную карту/локализацию
и провести аппаратные заезды. Общая автономная цель остаётся незавершённой.

## 02.10.2026 — центр цели и проверенный ручной start_match

По запросу пользователя explorer_goal теперь требует центр площадки с
допуском0,08м (как native MPPI xy_goal_tolerance), вместо касания контура
корпусом. Один общий helper используют decision и referee, observer узнаёт
новую причину STOP; outcome помечает explorer_goal_criterion=start_center_0.08m.
Цель decision и прежде была в центре, но прежние stop/referee обрывали путь.
Planner сохраняет точный endpoint вместо raster-center, только для настоящей
GOAL/EVADE цели, с проверкой последнего сегмента; frontier/recovery не
продлеваются через непроверенное пространство. Исторические цели не считать
доказательством нового условия. Тесты boundary/inside/center и смены ролей
добавлены. Финальные154 Python/helm tests pass, Compose config pass;
helm build duel terminal0 (/tmp/center-start-build.log). Два настоящих
helm start_match с config/match.yaml/seed0 прошли, новый критерий подтверждён:

| run_id suffix | готовность, wall s | snapshot, wall s | grant завершён, wall s | исход / sim s | финальная дистанция до центра |
| --- | ---: | ---: | ---: | --- | ---: |
| 035700522851Z | 53,6 | 17,3 | 77,5 | explorer_goal / 20,5 | 0,0369м |
| 040040307882Z | 59,4 | 2,6 | 66,7 | explorer_goal / 22,7 | 0,0267м |

Первый прогон использовал max4 параллельных CLI, второй — общий ROS-клиент.
Окончательный benchmarks/ros_parameter_snapshot.py читает list/get services
всех14 узлов без ros2 subprocess на каждый; timeout20s, типы и nested names
сохраняются, incomplete response отвергается. Read-only prototype сравнён
с прежним snapshot: единственное различие — штатное динамическое включение
guardian GoalAngle после начала матча. Метаданные и source SHA validation
по-прежнему выполняются до выдачи permissions, не отключались.

Во втором прогоне E/G mean .337/.375м/с,0contacts. Первый live-audit видел
обе allowed=true и common active=true, после outcome обе allowed=false,
все записанные final cmds0, единственный publisher hsl_motion_gate у каждого.
Во втором post-finish gate audit обе стороны pass. Артефакты frozen в
results/manual-20261002T035700522851Z/{startup-timing.json,live-audit.json,launch.log}
и results/manual-20261002T040040307882Z/{startup-timing.json,runtime.json,
outcome.json,first-metrics.json,second-metrics.json,center-check.json,
final-pose-world.yaml,gate-after-finish.json,launch.log}. Оставлен завершённый
мир с обоими stopped. GUI запускается по обычному config, визуальный RViz
layout этой проверкой не оценивался.

Готовность сенсоров/costmap/MPPI всё ещё занимает~54–59wall s в этой среде;
сокращение общего времени85→66,7 не считать чистым экспериментом по startup
из-за разной readiness duration. Выдача разрешений работает, CLI показывает
текущий этап. Два ручных прогона с одинаковым seed не заменяют новую20-серию;
старые20 boundary-goal результаты сохраняются отдельно. Следующий шаг общей
цели: повторная серия с center criterion и улучшение поимки слабого стража.

Ручной manual-20261002T033632989662Z уже завершён: обе permissions были
успешны, referee explorer_goal18,5sim s. От создания Gazebo03:36:38UTC до
первого SetBool ответа03:38:03UTC около85wall s; «не приходит» для этого
заезда не подтвердилось, задержка реальна. Runtime snapshot последовательно
читает14 ROS узлов до grant. Сначала dumps распараллелены(max4), затем заменены общим ROS-клиентом;
readiness/проверки сохранены. CLI печатает фазы+время, сохраняет startup timing.
Завершённый ручной мир пересоздан; два настоящих helm start_match проверены выше.

Предыдущий кандидат367f719/image80c72706 физически проверен в matched3
series-20261001T234503Z:3completed/0failed, seeds0–2,2 старых boundary goals/
1capture(seed1). Время19,1/8,9/18,0s; speeds E .376/.302/.343,
G .394/.272/.401;0contacts. Ближний lead не доказал преимущество:
та же частота1/3, guardian seed1 ниже.3. Полной20 нового lead ещё нет.
Baseline26f9017/checkpoint сохранён, общая цель остаётся открытой.

## 02.10.2026 — кандидат непрерывного ближнего упреждения

Baseline full20 сохранён26f9017/tag checkpoint/continuous-pursuit-baseline20-20261002.
Изменён только moving_capture_goal leadTime: min(1,(distance-.45)/ownmax)
вместо min(1,distance/ownmax), прежний radial fraction сохраняется.
При поперечном движении.5/ownmax.5 и distance.451 forecast offset теперь.001m,
а не.451m; на.449 он0. Непрерывно сохраняется смысл подхода до capture range.
Никаких предположений о максимуме скорости соперника не добавлено;
его observed velocity участвует без clamp. Head-on direct, wall validation,
face observed prey внутри.45, body footprint/gates/MPPI параметры прежние.

Регрессии remaining distance на.451/.6/.8/1.2, horizon saturation,
continuity вокруг.45 добавлены; old fast-prey assertion теперь проверяет
observed speed по новому времени. Existing head-on/wall/orientation проходят.
148 Python/helm tests и Compose config прошли. Физический результат нового
кандидата пока неизвестен; собрать образ и повторить seeds0–2 с trace/audit.
Снижение lead может как ускорить текущую поимку, так и потерять компенсацию
controller lag; принять решение по дуэлям, не только по математическойгранице.

## 02.10.2026 — continuous pursuit20 завершена; страж требует улучшения

Предыдущий turn — progress: геометрический просвет/slow capture trace разобраны,
артефакты и378bc71 сохранены, PID3288247 оставался live. Текущий turn завершил
наблюдение той же серии224501Z: exec87168 terminal0,20completed/0failed.
Полная таблица/агрегаты: [DUEL_EVALUATION_20261002_PURSUIT.md](DUEL_EVALUATION_20261002_PURSUIT.md).
Source813b515/image44ef9955; SHA maps одинаковы20/20, оба робота в одном
referee window,60 pair gate audits pass,20 isolated_runtime_cleaned=true.
19 goals/1 capture(seed17,14.8s),0 contacts/timeouts/pose jumps.
Speedmean/min E .366/.323 G .404/.302; below.3=0/20each.
GlobalRMSmean .068/.040; angularaccelRMS .804/1.012; RTF.545… .623.
По скоростям текущий порог выполнен, частота поимок5% показывает слабую роль;
не подгонять seeds или откатывать корректную семантику ради процента побед.

Дополнительная readonly costmap проверка seeds13/14 завершилась terminal0
(exec2670):54 snapshots,1036 cost254 cells map-free, но0 дальше.1m от SDFстен.
Ghost-peer гипотеза не подтверждена. Cost253 — inflation inscribed, не lethal.
Первый standalone observer имел другой DDS config и не видел publishers,
остановлен только этот observer; второй работал с теми же network/ipc/DDS
как оцениваемый стек. Нагрузку измерения для seeds13/14 отмечать при RTF
сравнениях. observer.py/frames/summary сохранены в директории серии.
Очищенные точки/masking/safety не ослаблялись.

ETA lowerbound: PURSUE в seed1 route15cells требует >=2.10m против прямой
1.56m, разница>=1.08s при ownmax.5; actual tracked error .074m.
Seed5 extreme1.99s включал observed error .315m и vel-.81 при measuredpeer
speed.502, поэтому этот случай не считать чистым доказательством ETA модели.
RequestedCAPTURE pose wallclearance<.33 только18/342cycles, target snapping
в normal maze0cycles; эти частные случаи не объясняют все19 неудач.

Следующий отдельный цикл: moving_capture_goal использует distance/ownmax
(до центра) и резко обнуляет lead при distance<=.45. Например transverse
velocity.5/ownmax.5: при.451 offset.451m, при.449 offset0. Это математическая
несогласованность ближнего перехвата. Проверить remaining distance до.45,
непрерывно затухающую leadTime и прежний radial/head-on blend. Скорости,
Nav2 параметры, globalA*, CPP detector и collision checks не менять.
Сначала регрессии/Compose/build, затем matched3seeds0–2, затем20 есликандидат
удачен; при слабой роли продолжать дальнейшие итерации. Current goal active.

## 02.10.2026 — полная20:6 готовы; геометрический просвет и разбор отхода

Предыдущий turn — progress: добавлен offline straight-motion report638156c,
147 тестов прошли, серия224501Z оставалась live. Сейчас PID3288247/exec87168
подтверждены live;6 completed/seed6 выполняется, не перезапускать runner.
Навигационные исходники813b515/image44ef9955 прежние, дерево чистое до журнала.

Первые6: все explorer_goal, контакты0, speeds E/G:
seed0 .387/.418;1 .344/.420;2 .364/.411;3 .377/.400;
4 .385/.409;5 .358/.388. Минимумы .344/.388; поимок0/6.
Это промежуточные результаты; итоговую успешность стража пока не оценивать.
График progress-motion.png и rows обновлены6/20 в директории серии.

Из установленного jr_map.sdf_map_server.collect_boxes получены20 wall boxes
на z=.25 с применением SDF state link poses (в определении стены иначе
совпадают). Map pose переведена вworld через сохранённый map_origin_world.
Static-clearance-progress.json содержит signed point-to-oriented-rectangle
clearance минус radius.178 (body approximation) и.23 (native safety envelope).
Chords между own trace poses с gap<=.3s подразделены шагом<=.02m; frame/window
совпадают с referee. Для первых4 min envelope E/G .152/.137m; body .204/.189m.
После обновления на6 матчей min envelope E/G .151/.118m.
Для slow short3seed2 min envelope .149/.179m, контактов0. Dynamic obstacles
и точный moving footprint в этом расчёте не измеряются, .25-height slice
соответствует стенам данного мира. Не заявлять доказанную непрерывную
collision safety по одному этому приближению. Frozen static-boxes-world.json
и analyze-static-clearance.py сохранены в224501Z, воспроизводятся с argvseries;
геометрия и скрипт являются артефактами измерения, не входом алгоритмов.

Дополнительно разобран short3seed2 explorer speed.233/capture8.1:
не recovery/statusfailure. На1.02s v.0055/omega1.06 (первоначальный turn);
на5.62s EVADE shortgoal .9m NW, на5.84s уже objective fullgoal distance2.85m.
До7.84s reverse v~.12–.16 при rasterhead SW→South→East; на8.04s v-.267.
Гипотеза «удлинить короткую escape point для устранения всего замедления»
отвергнута: большую часть ухода reference уже длинная. Переключение начальной
departure direction и прохождение rastercorner остаются гипотезами;
не менять несколько элементов без отдельного причинного прогона.

Официальный PathAngleCritic1.1.20 сверён по
https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_mppi_controller/src/critics/path_angle_critic.cpp:
vx_min<0 и forward_preferencefalse корректируют angle к ближайшему концу
робота; ограничение/штраф самого reverse этим critic не подтверждён.
Следующий шаг — завершить эту20, сопоставить индивидуальные motion/straight/
clearance/capture traces; при низкой скорости/слабой роли продолжать цикл.

## 02.10.2026 — анализ прямых участков во время полной20

Предыдущий goal-turn — progress: safe continuous goal813b515 реализован,
141 тест/Compose/сборка и matched3 завершены, новая20 запущена.
Текущая серия224501Z подтверждена живым PID3288247 и exec87168;
первые seeds0/1 explorer_goal17.7/18.1s, speeds .387/.418 и .344/.420.
Это промежуточные данные, не итог20; навигационные source/image не менялись.

Добавлен offline benchmarks/report_straight_motion.py: измеренные omega
sign changes на прямой observed global reference, angular/lateral RMS,
временное покрытие и arc motion. Прямая — >=.45m впереди, угол сегментов
<=.05rad, скорость>=.05. Sign deadband .05rad/s; reset при turn/stop,
смене referenceheading/behavior или gap>.3s. Используются интервалы с
обоими концами в referee window, без экстраполяции и заполнения пробелов.
Это диагностика, не новый limiter/control heuristic.
6 регрессий проверяют duplicate/short/turn geometry, noise/deadband,
неравномерные dt, gaps/stops/windows, смену пути/поведения и раздельные роли.
README/AGENTS описывают ограничения: до6 globalpoints, cached geometry,
reference straightness не доказывает clearance, коррекция не автоматическидефект.

JSON straight-motion.json сохранён для completed height20(210408Z) и short3
(223730Z). В height20 E/G: straight exposure118.96/144.72s, flips47/51,
rate23.7/21.1permin, mean individual straight lateralRMS .071/.028m.
В short3: exposure15.08/18.90s, flips4/10, rate15.9/31.7permin,
lateral .077/.026m. Наборы/исходы различаются; не объявлять улучшение/
регрессию алгоритма из этих разных серий. Следующий шаг — такой же анализ
всех20 текущих матчей и конкретных участков с частыми коррекциями.

## 02.10.2026 — полная серия20 запущена

После matched3 записан4165d62, checkpoint/continuous-pursuit-three-20261002.
Полная series-20261001T224501Z выполняется: PID3288247 подтверждён pgrep,
exec87168 write_stdin вернул running; это verified wait, не terminal.
Лог /tmp/continuous-pursuit-series20.log. Команда совпадает с matched3,
кроме --runs20; seeds0–19 выбраны заранее, world reset каждый матч,
фиксированные роли/старты, active360, trace/audit-start.
Image44ef9955/source813b515; doc-only4165d62 не меняет образ.
Не перезапускать по отсутствию новых строк: продолжить exec87168 или
проверить PID, затем читать results/isolated/hsl-eval/series-20261001T224501Z.
Текущая серия ещё не даёт итоговых метрик, цель не завершена.

## 02.10.2026 — короткая цель физически проверена; готовится полная20

Изменение813b515 собрано terminal0 (exec45561), image
44ef99557ed5dfe7cc875222d9fff465489b90ce487362d0c36afe08839e82fa.
Matched series-20261001T223730Z завершена terminal0(exec66724),3/3 без
технических ошибок. Команда: `python3 benchmarks/run_duel_series.py
--isolated-project hsl-eval --ros-domain-id 73 --gazebo-port 11418
--runs 3 --start-seed 0 --active-s 360 --trace --audit-start`.
Конфиг/default фиксированные роли и старты прежние.

| Seed | Исход | Время s | Speed E/G m/s | Lateral RMS E/G m | Angular accel RMS E/G rad/s² | Контакты | RTF |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | explorer_goal | 17.8 | .362/.428 | .055/.043 | .844/.944 | 0/0 | .627 |
| 1 | explorer_goal | 19.3 | .376/.381 | .086/.041 | .903/1.300 | 0/0 | .588 |
| 2 | guardian_capture | 8.1 | .233/.345 | .077/.043 | 1.355/.615 | 0/0 | .589 |

Speedmean .324/.385; minimum .233/.345. Below.2:0/3 обеих; below.3:
E1/3 G0/3. Lateral mean .072/.043 против предыдущих .085/.038;
angular mean1.034/.953 против .920/1.124. Stop-go2.3/0 против1.7/.3.
Разные исходы/длительности и асинхронность запрещают приписать всю разницу
этому исправлению. Предыдущая серия:3 цели, теперь2 цели/1 поимка.
Все9 gate-аудитов (две роли каждый) прошли;3 owned cleanup logs.

В guardian trace seed2 sim670.607 PURSUE reuse: own(.773,1.829),
continuous target(.554,1.738), старая raster end(.75,1.95).
Новый route_cells2/route_end=actual target, native resultok/statusOK.
Это фактическая проверка исправленного случая; во всех трёх NO_GLOBAL_PATH0.
final-capture-comparison.json сохранён: seed2 ближайший trace sample .458m,
heading6.14°; дискретная трасса не покрывает сам момент пересечения .45,
поимку подтверждает referee с LOS, без телепортов.

Решение: сохранить безопасную continuous short-goal поправку; не заявлять
устойчивость на3 матчах или выполненный порог.3. Следующий шаг — заранее
заданные seeds0–19, fixed explorer first, active360, trace/audit-start,
на этом же образе; собрать таблицу индивидуальных отчётов, выяснить низкую
скорость/исправительные манёвры и остаточные false LiDAR tracks. Не менять
навигационные исходники во время серии, оставить цель активной.

## 02.10.2026 — own-speed3 завершена; исправление одноточечного перехвата

Последний ответ о расположении детектора не изменял состояние цели (no progress).
Повторная проверка: дерево dc02f90 чистое, работают только init/daemon;
серия series-20261001T222149Z завершена (3 записи index, процесс отсутствует).
Она проверяет навигационный source fbcbb6b/image fe8eafa; referee revision4032e4f
включает последующий журнал, без изменения навигационных исходников.
Команда и условия — в предыдущем разделе: seeds0–2, fixed explorer first,
лимит360s, trace/audit-start, приватный hsl-eval/domain73/port11418.

| Seed | Исход | Время, s | Скорость E/G, m/s | Контакты E/G | RTF |
| --- | --- | --- | --- | --- | --- |
| 0 | explorer_goal | 18.5 | .392/.397 | 0/0 | .598 |
| 1 | explorer_goal | 19.2 | .381/.379 | 0/0 | .574 |
| 2 | explorer_goal | 18.4 | .362/.399 | 0/0 | .587 |

Средняя скорость .378/.392, минимумы .362/.379. Поимок0/3.
Global lateral RMS mean .085/.038, angular acceleration RMS .920/1.124.
До изменения (series221115): speed .363/.397, lateral .072/.038,
angular .819/1.025. Исправление собственной скорости семантически необходимо,
но три асинхронных повторения не доказывают улучшения поимки или плавности.
Checkpoint checkpoint/own-speed-ordinary3-20261002 сохранён на dc02f90.

Трасса seed1: два цикла PURSUE содержат одну raster-точку при непрерывной
цели рядом с роботом; native MPPI требует >=2 poses и сообщает NO_GLOBAL_PATH
(.021 доли отсчётов). Второй цикл reuse сохраняет старую клетку после смещения
цели. Ранее seed1 также содержал один такой цикл (.010). Это отдельный дефект,
не доказательство причины всех неудачных перехватов.

Изменение: continuous_short_goal_route для PURSUE/CAPTURE заменяет singleton
на [own,current target], только если safe_segment проверил весь отрезок по
карте/scan с прежним safety margin. Длинные/пустые/небезопасные маршруты
не меняются. CAPTURE singleton ранее расширялся без этой проверки.
Добавлены регрессии sub-cell target/yaw, moved reused target, wall/scan/bounds
и сохранения длинного коридора. 141 Python/helm тест и Compose прошли.
Физическая проверка нового изменения ещё не проведена; следующий шаг —
сборка и matched seeds0–2 тем же runner, затем анализ одноточечных путей,
скорости/плавности/исходов и новая полная оценочная серия после стабилизации.

## 02.10.2026 — запущен физический own-speed цикл, исправлен integer YAML

Предыдущий turn — progress: ordinary3 завершены, внесён own-speed fix,
136 тестов/Compose прошли и запущена сборка. Сборка завершена terminal0
(exec11716), imagefe8eafa4d5752fb16ccdb184de0d187e6a594290e1347a53c2b7f74f9126ffca,
sourcefbcbb6b. После неё запущена новая серия series-20261001T222149Z,
PID3239056/exec74694 подтверждён живым ps, лог
/tmp/intercept-own-speed-series3.log. Команда:
`python3 benchmarks/run_duel_series.py --isolated-project hsl-eval
--ros-domain-id 73 --gazebo-port 11418 --runs 3 --start-seed 0
--active-s 360 --trace --audit-start`.
Default match config не менялся, сравнение с221115Z на seeds0–2.
Первый seed0 explorer_goal18.5s, speedsE/G0.392/0.397, contacts0/0.
00-runtime.json подтверждает own_max_speed0.5 обоих decision nodes.
Остальные результаты пока не получены; не объявлять новый win-rate,
не менять nav source до terminal и не перезапускать по timeout наблюдения.

Независимый проверяемый дефект external YAML: допустимое max_speed:1
экспортировалось HSL_MAX_SPEED='1'; прямой ROS DOUBLE own_max_speed override
получал INTEGER. Standalone запуск нового installed node в отдельном
ROS_DOMAIN_ID84 завершился InvalidParameterTypeException/exit1 при:=1.
При:=1.0 node остаётся жив до timeout3s/exit124, type exception нет.
При этом motion не разрешалось и наблюдений не было; это startup type
проверка, не физическое движение или целый mission1м/с.

configuration_environment теперь преобразует max_speed к float перед
экспортом; значение1 становится'1.0'. Default0.5 и численное значение не
изменились, навигационный image/source этого цикла не менялись. Добавлен
unit integer YAML→ROS DOUBLE,24 config/runner теста прошли. Изменение
хостового exporter не требует rebuild навигации, относится к согласованию
нового own_max_speed интерфейса. Остаточные box tracks/одноточечные пути
и слабая роль стража остаются открытыми.

## 02.10.2026 — обычные прогоны diameter detector и кандидат own-speed fix

Предыдущий turn — progress: diameter filter подтверждён двумя low trials,
запущена обычная проверка. series-20261001T221115Z завершена terminal0
(exec36697),3completed/0failed, source5a65d45/image5d414dfd, fixed explorer
first seeds0–2,360sim лимит до первого события, без unknown fixture.

| seed | исход | sim s | скорость E/G м/с | global RMS E/G м | RTF | контакты |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | explorer_goal | 17.3 | 0.372/0.431 | 0.058/0.043 | 0.621 | 0/0 |
| 1 | explorer_goal | 20.6 | 0.370/0.361 | 0.080/0.034 | 0.606 | 0/0 |
| 2 | explorer_goal | 18.1 | 0.346/0.398 | 0.077/0.038 | 0.592 | 0/0 |

Все gates passed, все owned runtime очищены. Средние/min speeds
E0.363/0.346,G0.397/0.361; global RMS mean0.072/0.038м,
angular accel RMS mean0.819/1.025рад/с². Ordinary3 не показала регрессии
скорости/контактов, но поимок0; нельзя считать эффективность стража
исправленной или статистику трёх seeds доказательством win rate.
Checkpoint/detector-diameter-ordinary3-20261002 сохраняет636cb7b,
версия nav та же5a65d45. Low remnants идут сериями маленьких фрагментов
(например old replay scan timestamps682.49…684.0), поэтому простое правило
двух последовательных обнаружений не обосновано как решение false tracks.

Следующий отдельный кандидат: own_max_speed у DecisionPolicy/ROS node,
default0.5, конечное положительное значение. Compose обеих ролей передаёт
HSL_MAX_SPEED из existing motion.max_speed; обе YAML конфигурации объявляют
параметр. PURSUE явно передаёт его intercept_point; прежний implicit0.3
убран из этого вызова. CAPTURE planner/control tuning/intent max-speed
fields не менялись. Скорость соперника не ограничивается. Runtime audit
требует equality own_max_speed текущему конфигу, включая namespaceopponent.

Контрольный head-on пример own[0,0], prey[1,0], measuredv[-0.25,0]:
meetingtime1.818s при own0.3 против1.333s при own0.5. Unit test проверяет
аналитический target и отказ NaN/Inf/zero; metadata regression отвергает
old0.3 в opponent decision manager.136 Python/helm тестов и Compose config
прошли. README/AGENTS обновлены. Польза для реальных исходов пока не
проверена. Сборка запущена: PID3234805/exec11716, process подтверждён
живым pgrep; лог /tmp/intercept-own-speed-build.log. Не запускать build
повторно по timeout ожидания: возобновить тот же handle и проверить terminal.
После успешной сборки прогнать те же ordinary seeds0–2 командой
`python3 benchmarks/run_duel_series.py --isolated-project hsl-eval
--ros-domain-id 73 --gazebo-port 11418 --runs 3 --start-seed 0
--active-s 360 --trace --audit-start`, затем оценить следующий цикл. Остаточные ложные box tracks остаются отдельным дефектом.

## 02.10.2026 — diameter filter подтверждён двумя low-fixture матчами

Предыдущий turn — progress: внесён model diameter check,7 CPP/135 Python
тестов прошли, сборка5d414dfd завершена, запущен повтор. Текущая серия
series-20261001T220410Z завершена terminal0(exec39712),2 completed/0failed,
seeds0/1 заранее, fixed explorer first,90sim лимит, fixture0.6×0.6×0.15м,
source5a65d45/image5d414dfd304790e7e4cf184b61458c08db5ca3cc305996b3618f645b02802d69.
Команда приведена в предыдущем разделе. Navigation/config не менялись.

| seed | исход | sim s | скорость E/G м/с | контакты E/G | RTF | fresh box-near samples E/G |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | explorer_goal | 40.8 | 0.386/0.382 | 0/0 | 0.599 | 18/36 |
| 1 | explorer_goal | 40.9 | 0.391/0.382 | 0/0 | 0.585 | 31/52 |

До изменения low-box seed0:51.2s,0.326/0.143м/с, box-near E161/G662.
Теперь у seed0 GNO_LOCAL_PATH0 вместо0.354; seed1 GNO_LOCAL_PATH0.029,
NO_GLOBAL_PATH0.007. Frame samples вокруг fixture<0.4м при fresh stamp≤1s,
это дискретные счётчики, не временные доли/exhaustive semantic labels.
Нельзя объявлять отсутствие ложных обнаружений:36/52Gsamples остаются.

Box виден в LiDAR, изменённый safe route найден, original static map
unchanged в обоих. Минимальный sampled корпусный запас до box E/G:
seed0 0.379/0.364м, seed1 0.378/0.362м. Все gate audits before/partial/after
прошли, оба owned runtime очищены. fixture-track-check.json сохраняет
парный разбор в каталоге серии. Обе роли≥0.3 и contacts0 подтверждены
на этой паре trials; guardian capture в этой паре не было.

Сохраняем diameter filter как улучшение наблюдаемого застревания, но
не считаем классификацию решённой: small fragments остаются похожими
на stationary robot. Следующая проверка —3 обычных дуэли seeds0–2 с тем
же кандидатом без box, trace/gates,360sim лимит. Она нужна для проверки
потери настоящего соперника после фильтра, не заменяет новую итоговую20.
Обычная проверка запущена: series-20261001T221115Z, PID3214204/exec36697
подтверждён живым pgrep, лог /tmp/detector-diameter-ordinary3.log.
Команда: `python3 benchmarks/run_duel_series.py --isolated-project hsl-eval
--ros-domain-id 73 --gazebo-port 11418 --runs 3 --start-seed 0
--active-s 360 --trace --audit-start`. Не менять nav/config до terminal,
не перезапускать по одному observation timeout. Checkpoint
checkpoint/detector-diameter-low-20261002 указывает на a9584bf.

После неё отдельный цикл own-speed interception mismatch и оставшихся
false tracks. Цель не завершена.

## 02.10.2026 — кандидат проверки физического XY-диаметра

Предыдущий turn — progress: low fixture подтвердил false association и
Gmean0.143. Сохранён checkpoint/before-diameter-detector-20261002 на6810ba4.
Все private worlds остановлены; начат новый отдельный цикл детектора.

Гипотеза: bbox extent0.70 не связан с диаметром предоставленного корпуса.
Новая необходимая проверка: XY pairwise diameter≤0.476 =2×0.178+2×3×0.02,
где0.178 body radius Xacro,0.02 simulated range sigma. Это model/noise
assumption, не сведения о карте/политике/скорости соперника. Быстрый bbox
axis check, затем при необходимости convex hull и проверка пар вершин;
размер bbox diagonal не подменяет physical diameter. Старый extent0.70
guard удалён как избыточный. Высотный фильтр/association/центр без изменений.
Небольшой видимый фрагмент коробки всё ещё неоднозначен, stationary robot
не отвергается по отсутствию движения. Шумовой запас не доказан на hardware.

На193 low-box clouds реальный CPP replay до/после: fixture_near76→13,
peer_near58→76, no_detection56→99, other3→5. Больше peer_near возникает
в том числе из-за удаления конкурирующих коробок; это не exhaustive recall.
На прежних60 high-box clouds:0fixture/10peer/50none сохранены.
Контрфактический threshold0.436 дал10fixture/76peer,0.476 —13/76;
выбран0.476 по3σ model margin, не минимизации ошибок на данном seed.
Артефакты replay-diameter-* рядом с исходными recordings; временная
экспериментальная копия core в/tmp/hsl-diameter-trial не является runtime.

7 C++ gtest в прежнем ROS image с mounted актуальным исходником прошли,
включая full ring(bbox diagonal≈0.503 при diameter0.356), collinear/
duplicated/rotated points, low wide box и допустимый неоднозначный small
fragment.135 Python/navigation/runner/config/capture/helm тестов прошли.
README/AGENTS описывают кандидата и ограничения. Физические показатели
скорости/контактов/RTF/ложных треков после изменения ещё не получены.
helm build duel завершён terminal0(exec23251), лог
/tmp/detector-diameter-build.log; новый image
5d414dfd304790e7e4cf184b61458c08db5ca3cc305996b3618f645b02802d69,
source commit5a65d45. Compose config --quiet прошёл (VEHICLE_ID=0 для
неиспользуемых заготовок real-data; без него были только warnings).

Запущен физический повтор `series-20261001T220410Z`, PID3199183/exec39712
подтверждён живым pgrep, лог /tmp/detector-diameter-low-series.log:
`python3 benchmarks/run_duel_series.py --isolated-project hsl-eval
--ros-domain-id 73 --gazebo-port 11418 --runs 2 --start-seed 0
--active-s 90 --trace --audit-start --unknown-obstacle
--obstacle-height 0.15 --record-detector-scans`.
Навигацию/конфиг не менять до terminal, не перезапускать по timeout
наблюдения. После завершения сравнить Gspeed с0.143, fresh box tracks с662,
контакты/clearance/RTF/исходы и replay. Не считать replay завершением цели.

## 02.10.2026 — низкое неизвестное препятствие выявило ложный трек

Предыдущий turn — progress: ordinary20 завершён/проверен/checkpoint сохранён,
начат отдельный low fixture. Опыт series-20261001T215230Z теперь terminal0
(exec38112 завершён),1 completed/0 failed, fixed explorer first seed0,
90sim лимит, тот же навигационный образc5129af0a4e0 и исходники2b48e1b;
referee code_revision=cc8b537 — поздний docs commit с прежним navigation SHA.
Точная команда записана в предыдущем разделе; fixture0.6×0.6×0.15м.

explorer_goal51.2sim s, скорости E/G0.326/0.143м/с, контакты0/0,
RTF0.583. Ящик установлен в[0,1.425] по реальному published route,
виден в LiDAR, safe_replanned_route_seen=true, static_map_unchanged=true.
Минимальный sampled запас кругового корпуса до коробки E/G0.828/0.335м.
Этого недостаточно для принятия: страж ниже порога0.2 и ложная цель.
G planner statuses: NO_LOCAL_PATH0.354, OK0.584, RECOVERY_MPPI0.057,
WAIT_OR_STOP0.006; behavior PURSUE0.930, SEARCH0.068. Все gates прошли,
runtime автоматически очищен; docker ps показывает только init services.

Сохранены193 clouds обоих наблюдателей, replay_grid и offline peer labels
в00-obstacle.json. Реальный CPP replay:
`python3 benchmarks/replay_detector_clouds.py
results/isolated/hsl-eval/series-20261001T215230Z/00-obstacle.json
--output results/isolated/hsl-eval/series-20261001T215230Z/replay-low-box.json`.
Результат76 fixture_near/58 peer_near/56 no_detection/3 other.
Labels proximity<0.4м не являются exhaustive semantic recall; truth не
подаётся detector. fixture-track-check.json: свежие ≤1s live samples
около коробки E161/G662. Сам факт detour не доказывает правильность детектора.

У selected box clusters extent min/median/max0.187/0.557/0.697м;
peer-near clusters0.109/0.346/0.491м. Предел0.4 отверг бы19/58 peer hits,
предел0.5 сохранил бы58/58, но пропустил12/76 box hits. Нельзя просто
подобрать bbox extent порог и объявить проблему решённой.
Xacro: body cylinderR0.178, plate cylindersR0.170; XY bbox diagonal
круга может достигать sqrt(2)*diameter≈0.503м. Для следующего classifier
оценить реальный горизонтальный diameter/совместимость с корпусом,
а не путать bbox diagonal с диаметром и не требовать движения робота.
Частично наблюдаемый маленький фрагмент коробки остаётся неоднозначным;
проверять replay на высоком/низком fixture и true stationary samples,
затем физический повтор на тех же seeds. Own-speed interception mismatch
ещё не исправлен; false obstacle association теперь первоочередной дефект.
Задача остаётся активной, не объявлять low trial успехом по explorer_goal.

## 02.10.2026 — завершена новая20-серия, следующий опыт low fixture

Предыдущий turn — progress: проверена граница арены/построены графики,
продолжался тот же process3047475. Серия series-20261001T210408Z теперь
terminal0 (exec1280 завершён),20 completed/0 failed, seeds0–19 заранее,
360sim лимит до первого события, fixed explorer first, источник2b48e1b,
образc5129af0a4e078d5f5e8b7e4b6df7ea3ff83739178de2333d41dac32619d1e9a.
Полная таблица и условия — [DUEL_EVALUATION_20261002.md](DUEL_EVALUATION_20261002.md).

14 explorer_goal/6 guardian_capture/0timeouts, контакты0/0. Средняя
индивидуальных средних скоростей E/G0.3596/0.401, минимум0.297/0.343м/с.
Ниже0.2 —0/20 обеих ролей, ниже0.3 —1/20 E(seed8),0/20 G.
Global lateral RMS mean0.0712/0.0401м; angular acceleration RMS mean
0.8501/1.0209рад/с²; planner availability mean0.9866/0.9767,
moving fraction mean0.9590/0.9731. RTF0.536…0.620.

Все60 парных gate-аудитов before/partial/after прошли, final publisher
единственный hsl_motion_gate каждого namespace; все20 private runtime
очищены. Docker ps после terminal показывает только init services обоих
проектов. Verified source SHA одинаков во всех20 runtime, image ID один.
Агрегат evaluation-summary.json находится в каталоге серии.
Эти результаты не закрывают всю цель: слабее страж,0.3 в каждом матче
ещё не достигнуто, низкие unknown объекты/другие старты/GUI не подтверждены.

Следующий отдельный опыт без изменения навигации:
`python3 benchmarks/run_duel_series.py --isolated-project hsl-eval
--ros-domain-id 73 --gazebo-port 11418 --runs 1 --start-seed 0
--active-s 90 --trace --audit-start --unknown-obstacle
--obstacle-height 0.15 --record-detector-scans`.
Опыт запущен: `series-20261001T215230Z`, PID3181490/exec38112,
лог /tmp/detector-low-fixture.log. Процесс подтверждён живым pgrep после
запуска. Это отдельная1-run серия, не повтор ordinary20; не перезапускать
по тайм-ауту наблюдения. Итог20 checkpoint/detector-height-baseline20-20261002
указывает на cc8b537. График progress-motion.png пересчитан для20/20,
final-capture-comparison.json сохранён для полной серии.

После него анализировать fixture seen/detector tracks/actual clearance/
route/contacts/outcome и сохранённые clouds. Затем отдельное согласование
own pursuer_speed через motion.max_speed (сейчас Compose не передаёт его
decision manager). Не менять знания о скорости соперника и не подгонять
результат к50/50. Сохранён новый checkpoint полной ordinary20-серии.

## 02.10.2026 — границы арены и промежуточные графики

Предыдущий goal-turn — progress/verified wait: проверены шлюзы и forward-only
guardian, опрошен живой exec1280. Тот же процесс3047475 продолжает серию;
последний завершённый префикс seeds0–15:16 матчей,12 целей/4 поимки,
контакты0, все средние≥0.2. seed15 explorer_goal19.4sim s,
скорость E/G0.360/0.391м/с. Навигационные исходники/конфиг не менялись.

В progress-arena-clearance.json сохранён минимальный sampled запас
кругового корпуса radius0.178м до четырёх границ arena_bounds для каждого
робота в окне referee. По16 матчам Eminimum0.2007м(seed9),
Gminimum0.2819м(seed8): выходов контура за внешнюю границу в этих samples нет.
Это не проверка внутренних стен и не непрерывная swept/collision гарантия;
координаты — cached own poses с временами trace samples.

В каталоге series-20261001T210408Z построен/визуально проверен
progress-motion.png: per-seed mean speed, global-path lateral RMS и angular
acceleration RMS для обеих ролей. progress-motion-rows.json содержит числа,
plot-progress.py — воспроизводящий скрипт (запуск из корня репозитория с
аргументом path серии; использует benchmarks/report_motion.py).
График явно помечен16/20, это не финальная таблица серии. Global RMS
вычисляется по доступным samples в окне referee, не является временным
интегралом или отклонением от неизменной единственной траектории.
После terminal20 пересчитать артефакты/итоговый отчёт, затем low fixture
и отдельный цикл перехвата. Требования всей цели пока не подтверждены.

## 02.10.2026 — проверка шлюзов и движения стража текущей серии

Предыдущий goal-turn — progress: цепочка одноточечный reference →
NO_GLOBAL_PATH подтверждена кодом/трассами, другие отказы относятся к
swept_collision. Текущая серия продолжена через живой PID3047475/exec1280,
без изменения навигации и без повторного старта. На последнем проверенном
завершённом префиксе seeds0–14:15 матчей,11 целей/4 поимки,0 контактов,
все средние≥0.2м/с. Поимка seed14 на18.5sim s, скорость E/G0.338/0.412.

Прочитаны JSON-аудиты before-start/partial-start/after-finish всех первых14
заездов: обе стороны passed во всех трёх фазах, единственный final cmd_vel
publisher hsl_motion_gate своего namespace, allowed=false и команды0
после финиша. Все14 runtime_cleaned=true; progress-gate-audit.json сохраняет
этот конкретный префикс (файл не обновляется автоматически).
RTF первых14 0.536…0.620; минимальная скорость E/G0.297/0.343.

Отдельно проверены активные trace-second samples seeds0–14:
отрицательных команд guardian cmd_speed_mps<-0.001 нет ни в одном матче.
Назначение guardian=second подтверждено каждым robots[].role; полное окно
paired RunMetrics sample_coverage_fraction=1.0 у guardian во всех15.
Это проверка опубликованных команд по дискретным трассам, не непрерывная
гарантия аппаратной динамики. Параметры новых матчей пока неизменны;
следующий эксперимент низкого fixture начнётся только после terminal серии.

## 02.10.2026 — причина коротких остановок стража

Предыдущий goal-turn — progress: добавлен воспроизводимый capture report,
3 проверки окна/интерполяции прошли, exec1280 подтверждён живым. В текущем
turn та же серия/процесс3047475 продолжает seed13; исходники навигации
не изменены. Разобраны13 завершённых матчей, не запущен новый runtime.

progress-short-route-diagnostic.json в каталоге series-20261001T210408Z
содержит уникальные одноклеточные planning cycles и временные доли статусов
из paired RunMetrics в окне referee. seed0 —1 цикл, NO_GLOBAL_PATH0.006;
seed1 —3 цикла, NO_GLOBAL_PATH0.028. seeds2–12 одноклеточных циклов нет.
У seed2 NO_LOCAL_PATH0.045, у seed11 NO_LOCAL_PATH0.012; это другой дефект,
его нельзя объяснять длиной reference без раздельного разбора причин MPPI.

Дополнительный срез после seed13:14 матчей,11 целей/3 поимки,
все средние≥0.2, контакты0. progress-mppi-failures.json сохраняет причины
NO_LOCAL_PATH: все наблюдаемые failure samples — swept_collision
(Eseed1/8/13 по2 samples, Gseed2 14 samples, Gseed11 3 samples).
Это отказы полной проверки перемещаемого контура, а не proof физических
контактов или причина отменять проверку. Счётчики здесь sample counts,
не временные доли; outcome/window остаются по referee.

Код подтверждает цепочку: hsl_planning/node.py публикует global_status=OK
и одноточечный reference в PURSUE; hsl_nav2_control/src/native_mppi.cpp
строки281–284 отвергают reference.poses.size()<2, публикуют стоп/NO_GLOBAL_PATH.
Специальное преобразование одноточечного маршрута в [own,target] есть
только у CAPTURE. Возможный следующий фикс PURSUE должен проверить безопасный
continuous segment к актуальной цели и цель относительно соперника;
простое дублирование точки не доказывает достижимый перехват.

Примеры seed0: own[0.700,0.123], target[0.760,0.189], raster end[0.750,0.150],
наблюдаемый enemy[1.665,0.139]. seed1: own[1.250,2.033],
smoothed target[1.167,2.001], raster end[1.200,1.950], enemy[0.133,1.452].
Короткие маршруты объясняют небольшую часть остановок, но их нет в остальных
провальных перехватах: нельзя считать это единственной причиной слабой роли.
Также capture report использует cached poses по sample sim time;
для точной границы дистанции в следующем измерительном цикле стоит сохранять
original own odometry stamp, не выдавая текущие samples за синхронные pose stamps.

## 02.10.2026 — воспроизводимое сравнение геометрии поимок

Предыдущий goal-turn — progress: диагностика записана коммитом9e2d663,
живой exec1280 опрошен без перезапуска. Та же серия дошла до12 завершённых
матчей:10 целей,2 поимки(seeds8/10),0 контактов; PID3047475 жив,
идёт seed12. Навигационные исходники и конфиг неизменны.

Добавлен `benchmarks/report_capture_geometry.py` для повторения разбора
полной завершённой выборки. Команда:
`python3 benchmarks/report_capture_geometry.py
results/isolated/hsl-eval/series-20261001T210408Z`.
Вывод текущего префикса сохранён в progress-capture-comparison.json.
Роли определяются по robots[].role, окно по referee, одинаковые timestamps
дедуплицируются; интерполяция peer XY допускает gap≤0.3s и не экстраполирует
позу.3 теста проверяют пробелы/границы, смену назначения ролей и исключение
ложной близости до старта/после финиша. pytest tests/test_capture_report.py
прошёл. Это инструмент анализа, не изменение навигации.

В seeds8/10 последние доступные расстояния0.458/0.455м, направление1.9/1.7°,
страж в CAPTURE, продолжается движение вперёд. Referee подтвердил поимки,
но sampled trace не содержит distance<0.45: событие попало между отсчётами.
Отсутствие такого отсчёта нельзя трактовать как опровержение referee.
seed11(цель) минимальная sampled дистанция1.018м — до зоны поимки не доехал.
Инструмент сравнивает cached own poses по времени sample sim_t_s, а не
исходным odometry stamps; задержка обновления позы остаётся источником
ошибки. LOS не проверяется. Для точных границ события нужны referee и
исходные stamps; эта геометрия — диагностическая, не adjudication.

## 02.10.2026 — диагностика перехвата в продолжающейся серии

Серия `series-20261001T210408Z` продолжена без перезапуска и без изменения
навигационных исходников/конфига. Process3047475 подтверждён живым через ps;
exec1280 остаётся handle этой серии. Завершённый префикс seeds0–9:
9 explorer_goal, 1 guardian_capture (seed8), 0 контактов. Скорость mean/min
E0.361/0.297, G0.406/0.343м/с; ниже0.2 —0/10 у каждой роли,
ниже0.3 —1/10 у E. Global lateral RMS mean E/G0.071/0.040м,
angular acceleration RMS mean0.871/0.999рад/с², stop-go mean1.5/0.4.
Команда проверки: `python3 benchmarks/report_motion.py
results/isolated/hsl-eval/series-20261001T210408Z`.
Это частичная серия, не доказательство итоговой эффективности стража.

Найдено несоответствие в `hsl_decision/core.py`: PURSUE вызывает
intercept_point без pursuer_speed и использует default0.3м/с; собственный
предел MPPI/config сейчас0.5м/с. В planning CAPTURE moving_capture_goal
получает актуальный self.max_speed. Следующий отдельный цикл после окончания
серии должен согласовать расчёт с конфигом собственного робота, не ограничивая
оценку скорости соперника известным симуляционным пределом.

Артефакты в каталоге текущей серии:
- progress-pursuit-diagnostic.json: seeds0–6, только уникальные циклы
  planning.behavior=PURSUE. Прогноз восстановлен приблизительным обратным
  сдвигом0.42м от raw target (это standoff, а не чистое упреждение).
  Есть отвергнутые прогнозы и одноклеточные маршруты; несовпадение timestamps
  intent/planning/pose не позволяет считать восстановление точным.
- progress-own-speed-counterfactual.json: те же415 циклов, offline вызов
  существующей intercept_point на одинаковых входах с собственной скоростью
  0.3 и0.5. Только59/415 циклов меняют прогноз более чем на0.05м;
  медиана изменения0, максимум1.399м. Во многих случаях оба прогноза
  ограничены horizon2s. Несоответствие реально, но его устранение ещё не
  доказано как достаточное решение всех неудачных перехватов. Наблюдаемые
  оценки скорости соперника иногда достигают1.59м/с — кандидат на разбор
  шума локализации/центра кластера, не основание вводить известный cap0.5.
- progress-capture-geometry.json: seeds0–7, actual self poses обоих роботов
  сопоставлены интерполяцией по sim stamps в окне referee. В семи матчах
  минимальная дистанция0.673…0.830м. В seed1 минимум0.4587м при ошибке
  направления8.52°, 86 samples ближе0.65м без ошибки направления>45°.
  Значит, по этой выборке основной недостаток — достижение дистанции,
  а не ориентация вблизи поимки. Анализ distance/yaw не проверяет LOS
  и сам по себе не объявляет регламентную поимку.

Навигационный код пока не изменён: сначала завершить неизменную20-серию,
потом физический low fixture0.15м, затем отдельный цикл перехвата с
согласованием собственной скорости и повторными сопоставимыми seed.

## 02.10.2026 — продолжающаяся серия и подготовка низкого fixture

Предыдущий goal-turn — progress: высотная модель принята после двух
физических unknown матчей и запущена новая20-серия. На текущем срезе
process3047475/exec1280 подтверждён живым; продолжена та же серия,
не выполнялся повторный запуск. Завершённый префикс seeds0–2:3 цели,
0 контактов, минимальные скорости E/G0.352/0.343, все≥0.3. Global lateral
RMS mean E/G0.0659/0.0376м; seed0/1 angular accel RMS E1.055/1.111,
G1.194/1.006рад/с². Это частичные выборки, не итоговая оценка плавности
или успеха стража. progress-motion-report.txt/progress-metrics.json сохраняют
срез и метрики треков (Gmedian error0.057…0.106, p900.106…0.167м).
Следующие завершённые матчи не включены в этот ранний файл автоматически.

Для закрытия ограничения высотного фильтра подготовлен реальный параметр
fixture: --obstacle-height (default0.8, минимум0.15м). Меняется SDF box size
и centre-z, отчёт сохраняет фактическую высоту; XY/выбор из маршрута и
проверка альтернативного пути остаются прежними. --height передаётся в
unknown_obstacle_trial.py.23 unit-теста config/runner и compileall прошли,
включая отказ при NaN, слишком малой высоте и отсутствии unknown flag.
Физический низкий тест пока не выполнялся: текущая20-серия занимает стенд.
После неё выполнить seed0 с --unknown-obstacle --obstacle-height0.15
--record-detector-scans и анализировать реальные треки/контакты/исход.

Дополнительный offline разбор старых10 truth-near кластеров: исходный
радиальный центр median error0.182м, fixed-radius fitted centre0.046м,
но один случай fit ухудшает0.152→0.157 и два сохраняют ошибку≥0.149.
Это пока исследовательская выборка по approximate labels, не принятый
новый estimator: нельзя переключать навигацию посреди текущей20-серии
или объявлять ошибку детектора решённой. Классификация через circle-fit
остаётся отвергнутой; оценка центра по окружности — отдельная гипотеза.

## 02.10.2026 — начата20-серия нового кандидата

Принято изменение высотного профиля после replay и двух реальных unknown
матчей без ложного box-трека/контактов, обе роли≥0.3. Коммит2b48e1b,
checkpoint/detector-height-20261002, образc5129af0a4e0. Все прежние checkpoint
сохранены. Изменение не претендует на универсальную классификацию: регламент
допускает препятствия от0.15м высотой, поэтому низкий или частично видимый
предмет похожего размера ещё требует отдельной проверки. Не считать
высокую коробку доказательством обработки всех неизвестных препятствий.
PROJECT_GOAL инвентаризация исправлена: CPP perception, удалённые патруль
и debug follower, simulation только сенсорный стенд.

Запущена `series-20261001T210408Z`, process3047475/exec1280 (на этом срезе
подтверждён живым ps),20 заранее seeds0–19, fixed explorer first, конфиг
config/match.yaml,360sim до первого события, новый мир каждый раз:
`python3 benchmarks/run_duel_series.py --isolated-project hsl-eval
--ros-domain-id 73 --gazebo-port 11418 --runs 20 --start-seed 0
--active-s 360 --trace --audit-start`.
Лог /tmp/detector-height-series20.log, результаты results/isolated/hsl-eval/.
Нельзя перезапускать по одному тайм-ауту наблюдения: сначала проверить
этот конкретный процесс/handle и ownership. Статус серии и метрики ещё
не подтверждают итог; обновить по фактическому завершению.

## 01.10.2026 — проверка высотного профиля кандидатов детектора

Предыдущий goal-turn — progress: C++ перенос подтверждён ordinary duel,
ROS-графом и тестами; unknown box выявил низкую скорость стража0.124м/с.
Новая гипотеза: отвергать кластер, если наблюдаемая высота несовместима с
известным предоставленным корпусом, сохранив детекцию неподвижного робота.
Верх текущей Xacro0.0102+0.3966+0.003=0.4098м; модельный предел0.46м
включает~0.05м на шум. Это предположение о корпусе, не о скорости/политике
соперника. Параметр perception.opponent_max_height доступен через внешний
match.yaml без rebuild при каждом новом start_match, применяется обоим
детекторам. Старые конфиги нормализуются default0.46. Node/runtime/launch
аудиты сверяют effective значение; недопустимые значения отклоняются.

Кластеризуется вся прежняя полоса0.08…0.60, затем проверяется90-й квантиль
высоты: предварительное отсечение высоких точек оставило бы низ ящика
кандидатом. Шумовой единичный высокий возврат среди многих tolerates, но
компактный высокий объект отвергается. Добавлен C++ регрессионный тест
неподвижного робота, высокой коробки, шума и изменяемого профиля.

Через реальный текущий C++ header replay на60 облаках прошлой
series-20261001T191042Z: при отключении нового фильтра (height0.60)
23 fixture_near/5 peer_near/32 no_detection; при0.46 —0/10/50.
Truth используется только для labels после детектора, не для его входов.
Это проверка на известных входах, не доказательство semantic recall и
обобщения на низкие или частично видимые предметы. Артефакты replay-height-*
в исходном каталоге; инструмент benchmarks/replay_detector_clouds.py.
Физическая проверка series-20261001T205635Z завершена terminal0,
seeds0/1, лимит90sim, fixed explorer first, unknown fixture + record clouds,
trace и gate before/partial/after, source/parameter audit:

| seed | исход | sim s | скорость E/G м/с | RTF | контакты E/G |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | explorer_goal | 32.6 | 0.394/0.398 | 0.591 | 0/0 |
| 1 | guardian_capture | 21.4 | 0.383/0.384 | 0.591 | 0/0 |

fixture-track-check.json: свежих ложных треков около коробки0 у обоих
роботов в обоих матчах. До изменения на seed0 было E163/G44 и Gmean0.124.
Минимальный измеренный запас до коробки E/G: seed0 0.354/0.360м,
seed1 0.802/0.804м. Fixture виден, безопасный изменённый global route найден,
статическая карта unchanged. Все четыре средние выше0.3. Оба runtime
автоматически очищены; перед .profile обычной серии нового мира нет.
Не переносить вывод на низкие/частично видимые предметы: только эта
высокая коробка0.6×0.6×0.8 и предоставленная модель корпуса проверены. Сборка helm build duel успешна;
5 C++ gtest, installed-launch audit и131 Python/helm тест прошли.
Следующий шаг — ordinary fixed-role20 seeds0–19 до события/360sim,
с тем же YAML и новым образом, чтобы проверить распределения, скорость,
ошибку следования, плавность и реальные поимки. Это новый кандидат;
прежние20 матчей не заменяют его итоговую оценку.

## 01.10.2026 — проверенный перенос LiDAR-детектора в C++

Предыдущий turn — progress: сохранены два checkpoint и остановлен свой
незавершённый сбор облаков. Возобновлён перенос детектора, запрошенный
пользователем. Новые `src/hsl_perception/src/opponent_detector.cpp` и
`include/hsl_perception/detector.hpp` реализуют кластеризацию, вычитание
статической карты, радиальный сдвиг центра0.178м, связь с предыдущим
наблюдением и EMA скорости0.5. ROS-входы — только собственные scan/self/grid;
simulator peer truth отсутствует. Сохранены opponent/opponent_visible,
добавлен detector_cycle_ms. В первом переносе геометрическая классификация
не изменена: известная ложная реакция на компактный ящик пока сохраняется.

Добавлен observations.launch.py для обоих контейнеров; смерть любого из
двух процессов завершает launch. Python detector и его tracking удалены,
StaticGrid оставлен для referee/метрик/картографии. Три прежних Python
detector-теста перенесены в C++ gtest с проверкой неподвижного робота,
закрывающей стены, собственной поверхности, связи трека и большого объекта.
Runtime snapshot теперь проверяет source SHA пакета hsl_perception и
use_sim_time обоих opponent_detector до разрешения движения. Прикладные
85 unit-тестов прошли, Compose config и compileall валидны; сборка образа
и фактические заезды завершены. Контрольные точки до переноса сохранены.

Образ4b681540a157, сборка helm build duel успешна;4 C++ gtest и128 Python/helm
тестов прошли. Installed-launch audit проверил обоих C++-издателей и роли.
series-20261001T190613Z, seeds0/1, лимит90sim, fixed explorer first, trace
и gate before/partial/after-finish, runtime/source audit:

| seed | исход | sim s | скорость E/G м/с | RTF | контакты E/G |
| --- | --- | --- | --- | --- | --- |
| 0 | explorer_goal | 17.5 | 0.399/0.429 | 0.614 | 0/0 |
| 1 | guardian_capture | 8.9 | 0.272/0.320 | 0.607 | 0/0 |

Обе роли выше0.2; Eseed1 ниже0.3. Не приписывать ускорение и другой момент
поимки исключительно C++: асинхронная симуляция не побитово воспроизводима.
detector-graph-audit.json подтвердил одного C++ publisher каждого opponent
и opponent_visible, подписки только own scan/self/grid и служебные ROS.
На6-секундном срезе detector_cycle_ms: E36/G35 samples, медианы0.242/0.387,
p95 0.975/0.463, максимумы2.440/0.470мс. Это короткий срез, не whole-match
профиль. Оба собственных runtime автоматически очищены.

series-20261001T191042Z, seed0, те же старты + unknown box +
--record-detector-scans: explorer_goal15.2с, E/G0.281/0.124м/с,0 контактов,
RTF0.669. Fixture виден, безопасный detour найден, карта неизменна; ложные
свежие box-треки E163/G44 отсчёта подтверждены. Страж ниже0.2: перенос
не устранил ложную цель и не удовлетворяет полной цели автономности.
Сохранены60 map-frame cloud snapshots (30 каждого),121099 точек в полосе
0.08…0.60, исходная grid и offline peer-truth labels. Fixture pose выбрана
из реального пути, не зашита в навигации.

Offline-разбор cloud-shape-analysis.json/.log и скрипт в том же каталоге:
10 кластеров около truth робота и24 около коробки в старом допустимом
extent≤0.7. Fixed-radius circle-fit RMS у настоящего робота медиана0.065м,
у коробки0.058м — простое ужесточение этой метрики отвергнет и робота.
Пока такую эвристику не применять. Верхний90-й квантиль высоты: robot
0.116…0.149м, box0.466…0.566м; исследовать геометрию известного корпуса
и разновысотные срезы, не опираться на движение/бездействие как класс.
Лейблы близости≤0.4м приблизительны и не доказывают семантический класс
всех смешанных кластеров. Следующий цикл: осмысленная геометрическая
проверка с повтором этих облаков, неизвестного объекта и ordinary duel.

## 01.10.2026 — контрольная точка перед переносом детектора в C++

По последнему запросу пользователя выполнено только сохранение среза.
Подтверждённая навигационная версия280e9ec сохраняется отдельным тегом
`checkpoint/forward-guardian-verified-20261001` (одна цель/одна поимка,
0 контактов, RTF0.584–0.586). Детектор пока Python и имеет описанный ниже
дефект ложного обнаружения неизвестного ящика; перенос в C++ не выполнен.

Полный текущий срез сохранён тегом
`checkpoint/before-cpp-detector-20261001`. В нём дополнительно есть
незавершённая опция --record-detector-scans для unknown fixture:
карта и высотно отфильтрованные map-frame облака обоих роботов с offline
truth-метками. Навигационные входы не менялись. Compileall и17 unit-тестов
прошли; физическая запись series-20261001T185700Z остановлена до завершения
по просьбе о контрольной точке и не считается проверкой этой опции.
Свой изолированный стенд и init очищены, обычные init пользователя сохранены.
Предыдущий goal-turn был progress: изменён launch, проверены два матча,
устранена лишняя нагрузка завершённого стенда. Следующий шаг после этой
контрольной точки — перенос детектора с проверкой настоящих облаков;
задача устойчивой автономности остаётся незавершённой.

## 01.10.2026 — RTF, направления ролей и неизвестный объект

Запрос пользователя: reverse только исследователю, страж всегда вперёд;
RTF0.2–0.33 и разъяснение детектора. Launch теперь вычисляет reverse по
**роли**, а не namespace: explorer при разрешении±max_speed, guardian
0…max_speed и PathAngle.forward_preference=true. Синхронизированы runtime
и installed-launch аудиты, конфиг и инструкции. Максимумы0.5/1.5 не менялись.
Синтетический audit_native_bidirectional теперь принимает --role; прежняя
проверка заднего хода guardian больше не соответствует контракту.

Найден лишний работающий изолированный мир после завершения проверки.
В момент ручных стартов18:31/18:33 он выполнял собственный заезд18:32;
manual-20261001T183342670893Z имеет RTF0.274, приватная
series-20261001T183215Z —0.275. Контейнеры после матча продолжали физику
и обработку: наблюдатели~62/66%CPU, Gazebo~72%CPU на срезе docker stats.
Этот стенд очищен. Runner после каждого изолированного заезда автоматически
удаляет только подтверждённый собственный runtime; заменённый мир не трогает.
Окончательная причина всей потери RTF не доказана одним срезом: требуется
одиночный прогон, GUI и дополнительная нагрузка могут влиять отдельно.
Пользовательские manual-результаты сохранены, добавлены в gitignore.

Завершены проверки неизвестного ящика прежней двунаправленной версии:
unknown-obstacle-20261001T180824Z дал поимку23.6с, скорости E/G0.299/0.330,
**2 контакта робот–робот каждой роли**, страж в CAPTURE ехал назад при
ошибке ориентации~168°. Ошибка сохранения observer не отменяет этот
фактический небезопасный результат. unknown-obstacle-20261001T181656Z:
цель32.5с,0.325/0.242м/с, контактов0, но ящик принят за соперника обеими
ролями (167/188 свежих отсчётов около него). Минимальный измеренный запас
стража до коробки0.0705м. Детектор не классифицирует форму, это открытый
дефект, изменение reverse его не исправляет.

Рабочий fixture интегрирован в runner через --unknown-obstacle (только
изолированный проект). series-20261001T183215Z: поимка27.9с, E/G0.331/0.358,
контактов0; SpawnEntity подтверждён, ящик виден в скане, безопасный
изменённый маршрут найден, исходная карта неизменна, линия поимки не
перекрыта ящиком. Аудит шлюза теперь также выполняется после каждого
матча при --audit-start. Удалены три всегда-null trace-поля старого Python
MPPI. Это проверка benchmark, не доказательство надёжного распознавания.

Проверки нового reverse:17 unit-тестов runner/config прошли; сборка
helm build duel успешна; installed-launch audit прошёл все четыре сочетания
роль×allow_reverse при пределах0.7/1.2. Физическая серия после изменения:
`python3 benchmarks/run_duel_series.py --isolated-project hsl-eval
--ros-domain-id 73 --gazebo-port 11418 --runs 2 --start-seed 0
--active-s 90 --trace --audit-start` (series-20261001T184926Z).
Серия завершена terminal0,2/2 без технических ошибок:

| seed | исход | длительность sim s | скорость E/G м/с | RTF | контакты E/G |
| --- | --- | --- | --- | --- | --- |
| 0 | explorer_goal | 19.1 | 0.370/0.402 | 0.586 | 0/0 |
| 1 | guardian_capture | 19.7 | 0.379/0.403 | 0.584 | 0/0 |

forward-guardian-check.json подтверждает0 отрицательных команд стража,
минимум0 в обоих trace; isolated_runtime_cleaned=true у обоих заездов.
Gate before/partial/after-finish файлы прошли проверки. Compose config
валиден (предупреждения о незаданном VEHICLE_ID относятся к неиспользуемым
профилям). Одиночный headless RTF вернулся к уровню прежней baseline~0.6;
RTF1 и GUI этим не подтверждены. Подтверждены одна цель исследователя и
одна регламентная поимка стражем с новым forward-only диапазоном.
Прежняя20-серия относится к старому двунаправленному стражу, не переносить
её выводы на новый launch.

Следующий шаг: устранить ложные треки неизвестных предметов по реальным облакам,
повторить unknown и полноценную серию после принятого изменения.

## 01.10.2026 — проверка очищенного единственного MPPI

**20-серия завершена**, exec88921 terminal0:
`series-20261001T172331Z`, fixed explorer first, заранее seeds0–19,
лимит360sim s, nav0c8ad5a/revision09405a9, тот же native-only image.
14 explorer_goal/6 guardian_capture/0 timeout/0 technical failures,
контактов0 у обеих ролей во всех матчах. Средние E/G0.363/0.392м/с,
минимумы0.315/0.289; ниже0.2 —0/20 обеих, ниже0.3 —0/20 E и1/20 G.
Global lateral RMS mean0.073/0.039м; angular accel RMS mean0.876/1.074рад/с²;
stop-go mean1.9/0.5. Полная таблица, остальные агрегаты, измерительные окна
и пределы доказательств — [DUEL_EVALUATION_20261001.md](DUEL_EVALUATION_20261001.md).
Отдельный audit после seed19 в19-gate-after-finish.json подтвердил обе
permissions=false/common active=false, повторные zero cmd_vel и по одному
motion_gate publisher. Это проверка после последнего исхода, не20 отдельных
after-finish файлов. Runner live больше нет; не перезапускать эту baseline.

Первый unknown fixture запуск `unknown-obstacle-20261001T180552Z` остановился
до разрешения движения: временный harness передал DUEL_MAX_ACTIVE_S=360
integer, referee ждёт double. Runtime audit выявил отсутствующий node,
Gazebo не является виновником; это0 оценочных матчей. Harness исправлен
на360.0, повтор запускается тем же seed0/условиями, отдельной директорией.
Навигационный код пока не менялся. Следующие шаги: результат box, затем
отдельный гистерезис6↔7 и повторная оценка принятого изменения. Порог0.3
каждого робота и обработка неизвестного препятствия пока не доказаны.

Дополнительная проверка текущей серии: по считанной known_grid и измеренным
позам seeds0–13 внутри referee window вычислен консервативный запас до
статических стен и arena. Между отсчётами интерполяция≤0.01м; расстояние до
occupied cell centre минус half-cell diagonal, physical radius0.178 и0.006
на интерполяцию/округление. Минимум запаса E/G до стен0.1625/0.1614м,
до границ0.2017/0.2847м. `progress-static-clearance.json` сохраняет все строки.
Это оценка статической карты, не динамических роботов или нового объекта.
Seed14 guardian mean0.289м/с (первый ниже0.3, но выше0.2), moving0.938,
OK0.938, NO_GLOBAL_PATH0.031, angular accel RMS1.535рад/с².
В4.73–5.09с trace виден PURSUE с route_cells1: прогнозируемая точка перехвата
почти совпала с позой стража, reference из одной точки native не принимает.
Не объявлять это застреванием у стены: есть движение до/после; проверить
семантику ожидания перехвата и готовности передней ориентации отдельным циклом.
AGENTS уточнён по фактическому C++ JSON: число safe samples, clearance и
selected_progress в текущем native не публикуются; старые соответствующие
trace-поля null — остаток Python diagnostics. Удалить их после baseline,
не менять формат наблюдателя посередине этой серии. Native collision-check
и реальные contact counters сохранены. Отдельного after-finish gate audit
для каждого предыдущего короткого матча в каталоге нет; не переносить
историческую формулировку об этих аудитах на текущую20-серию. До/partial
start аудит есть у каждого; после seed19 дополнительно проверить оба final
cmd_vel, обе permissions=false, common active=false и одного publisher.

Продолжение цели: предыдущий turn — progress (исправлена документация,
серия подтверждена живой exec-session); текущий — verified wait той же
серии плюс анализ фактических трасс и подготовка неизвестного препятствия.
На срезе seeds0–10 получены7 целей/4 поимки, контактов0; первые9 отчётов
имеют минимальные средние E/G0.345/0.318м/с. Полной серии пока нет.
`progress-motion.png` в каталоге серии показывает первые10 матчей внутри
referee window: скорости, global lateral error, фактическую omega.
`progress-straight-diagnostic.json` ограничен прямыми движущимися участками
первых6 A* точек: chord≥0.45м, остаток относительно chord≤0.035м,
heading error≤15°, |omega|>0.15 для смен знака. Это узкая диагностика,
не доказательство гладкости всего маршрута. Seed9 guardian:5 смен знака
на6с выбранных участков; trace показывает смены6↔7 около0.8м.
`progress-capture-switches.json`: seeds5/6/9 имеют по5 смен PURSUE/CAPTURE.
Гипотеза для следующего цикла: дребезг на capture_distance меняет GoalAngle
и ориентирование, нужен проверяемый гистерезис входа/выхода. Пока не изменён.

Подготовлен отдельный Gazebo fixture test в ignored
`results/isolated/hsl-eval/unknown-obstacle-preparation/`:
unknown_obstacle_trial.py (ROS observer/SpawnEntity), run_unknown_trial.py
(один новый owned duel, отказывается запускаться при живом baseline runner),
unknown-grid.yaml (фактическая known_grid), fixture-offline-check.json.
Запуск после terminal baseline: python3
results/isolated/hsl-eval/unknown-obstacle-preparation/run_unknown_trial.py.
Seed0, тот же YAML/роли/лимит360; box0.6×0.6×0.8м выбирается на фактическом
глобальном маршруте впереди робота с offline безопасной альтернативой
в заданных arena_bounds. Статическая карта не обновляется fixture node.
На считанной карте и nominal A* из YAML найдена позиция map[0.6,1.5],
после блокирования существует другой маршрут; координаты не зашиты в код.
Проба select-only в живой серии не создала объекта; прежний порог clearance
0.7 не дал кандидата (максимум на nominal route0.692м), заменён на0.5
с обязательной полной проверкой альтернативы и отсутствия пересечения стен.
Реальное добавление препятствия и успешный объезд ещё НЕ проверены.
Observer сохраняет scan hits, пути, обе позы, просвет до box и отдельно
LOS box при поимке: referee пока использует статическую карту, поэтому
его capture без дополнительного LOS здесь не является достаточным доказательством.
Риск по исходникам: detect_opponent может спутать небольшой новый объект
с соперником, navigation_obstacles исключает tracked peer из облака;
подтвердить физическим тестом перед исправлением. Навигационный код/образ
этого цикла не изменены, fixture scripts проверены только compileall и
offline geometry, пока сохранены как артефакты эксперимента.

Расширенная серия запущена на неизменных навигационных исходниках:
`results/isolated/hsl-eval/series-20261001T172331Z/`, revision09405a9,
текущий YAML, fixed explorer first, заранее выбранные seeds0–19,
лимит360sim s до первого события. Команда: python3 benchmarks/run_duel_series.py
--isolated-project hsl-eval --ros-domain-id 73 --gazebo-port 11418 --runs 20
--start-seed 0 --active-s 360 --trace --audit-start.
На момент записи завершены seeds0–4: четыре цели и одна поимка,
средние скорости каждого робота в этих заездах выше0.3м/с. Это промежуточные
результаты, не итоговая оценка. Процесс продолжает работать; не запускать
вторую серию в этом проекте и не менять навигационные исходники до завершения.
После завершения собрать полную таблицу, разобрать слабые заезды и выполнить
физический тест неизвестного карте препятствия. README исправлен: проверка
очистки на seeds12–14 завершена, в AGENTS удалено старое описание выбора
сценария вместо YAML. Навигационный код и образ в этом шаге не менялись.

Удаление Python legacy закоммичено0c8ad5a; финальный образ
`sha256:dfc69f8a5db3865ac5f132bd37afd23765506a9e5b40f732dccb1e46729e7348`.
Команда повторения: python3 benchmarks/run_duel_series.py
--isolated-project hsl-eval --ros-domain-id 73 --gazebo-port 11418 --runs 3
--start-seed 12 --active-s 90 --trace --audit-start.
`results/isolated/hsl-eval/series-20261001T171210Z/`: все3 завершены,
source/runtime/common-start/after-finish audits прошли, один final publisher,
параметры native_mppi и native_costmap обоих роботов совпадают с baseline.

| Seed | Исход | sim s | Explorer м/с | Guardian м/с | Контакты E/G |
| --- | --- | ---: | ---: | ---: | --- |
| 12 | guardian_capture | 8.2 | 0.256 | 0.318 | 0/0 |
| 13 | guardian_capture | 19.9 | 0.355 | 0.387 | 0/0 |
| 14 | explorer_goal | 20.3 | 0.376 | 0.373 | 0/0 |

Средние E/G0.329/0.359, минимумы0.256/0.318; ниже0.2 —0/3 обеих,
ниже0.3 —1/3 explorer и0/3 guardian. Global lateral RMS E0.057/0.078/0.061м,
G0.024/0.046/0.046м; средние0.065/0.039. Angular accel RMS E1.095/0.969/0.990,
G0.815/1.152/1.160рад/с²; stop-go средние2.3/0.3. RTF0.611/0.589/0.581.
До очистки —2 цели/1 поимка, скорости0.359/0.386, RMS0.072/0.047,
angular accel0.951/0.972. Изменённые исходы и длительности не доказывают
улучшение/ухудшение алгоритма на такой малой серии; ROS/Gazebo асинхронны.
Очистка подтверждена сборкой,131 тестом, installed-launch audit и фактическими
целями/поимками без контактов. Штатный C++/A*/decision не менялись; dead Python
ветки действительно отсутствуют. Не возвращать их для объяснения вариативности.

После серии удалён только hsl-eval duel; ручной мир ранее закрыт по явному
ответу пользователя. Обычный запуск готов: helm start_match из YAML.
Остаются заготовки LIO-SAM/Livox/real sensing/perception и интерфейсы данных;
benchmark/probe/trace/report/arena monitor сохранены как инструменты проверки.
Следующий шаг цели — серия ≥20 с текущим YAML, fixed explorer first,
предварительно seeds0–19,360sim s, затем физический тест неизвестного на карте
препятствия и разбор слабых заездов. Порог0.3 у каждого и итоговая устойчивость
ещё не достигнуты; цель остаётся активной.

## 01.10.2026 — baseline после удаления MPC и очистка Python legacy

Пользователь разрешил закрыть ручной мир; `helm clean duel` завершился,
обычные duel-контейнеры удалены, служебные init сохранены. Изолированная серия
`results/isolated/hsl-eval/series-20261001T165242Z/` завершилась 3/3:
config/match.yaml, fixed explorer first, seeds12–14,90sim s, stock native,
imagee5f7d5f; навигационные исходники a07f83e (runtime SHA256 подтверждён),
runner3d8d673+dirty.f2eb728b0e1e с исправленным ожиданием launch.
Команда: python3 benchmarks/run_duel_series.py --isolated-project hsl-eval
--ros-domain-id 73 --gazebo-port 11418 --runs 3 --start-seed 12 --active-s 90
--trace --audit-start.

| Seed | Исход | sim s | Explorer м/с | Guardian м/с | Контакты E/G |
| --- | --- | ---: | ---: | ---: | --- |
| 12 | explorer_goal | 18.7 | 0.374 | 0.399 | 0/0 |
| 13 | guardian_capture | 13.2 | 0.325 | 0.383 | 0/0 |
| 14 | explorer_goal | 19.8 | 0.377 | 0.377 | 0/0 |

Средние E/G0.359/0.386, минимумы0.325/0.377; ниже0.2 и0.3 —0/3 обеих ролей.
Global lateral RMS E0.079/0.075/0.061м, G0.036/0.052/0.053м; средние0.072/0.047м.
Angular accel RMS E0.796/1.018/1.040, G0.890/0.799/1.226рад/с².
Окна обоих отчётов referee совпадают; coverage0.9992–1.0. Source/runtime,
нулевые команды до старта/при одном разрешении/после исхода и один publisher
прошли. Две реальные цели и одна поимка получены без телепортов, но это только
три коротких матча: финальные20/неизвестные препятствия ещё не подтверждены.
Срез сохранён в checkpoint/native-bidirectional-yaml-20261001 (3d8d673).

Первый технический запуск164948Z имеет0 завершённых матчей: прежний watchdog
считал ещё не появившийся gzserver падением после15wall s, хотя ROS launch
продолжал работать. После исправления ждём появление до readiness deadline,
выход после появления или завершённый контейнер — реальные terminal events.
Добавлен regression test живого launch до появления Gazebo,8 runner tests
прошли. Нет оценочного результата от этого технического отказа.

По новому запросу пользователя убирается неиспользуемая Python mppi.py и
backend selector, его параметры/NumPy/direct velocity publisher/внутренняя
диагностика/warm start/local recovery. Основной Python planner теперь только
глобальный reference/A*/роль/recovery; локальные команды и визуализация всегда
stock C++ Nav2 MPPI. Удалены debug follower и scripted opponent/patrol/profile,
неиспользуемая local_guidance, соответствующие зависимости, тесты и описания.
Probe сохраняет фактический опубликованный local_path вместо реконструированного
старым guidance. Локализация LIO-SAM, реальные sensing/perception заготовки и
интерфейсы данных сохранены. Навигационные решения native/A* не меняются.
131 тест прошёл, compileall/diff check чистые. Финальная сборка успешна
(`/tmp/native-only-clean-build-final.log`), hash planner внутри образа и на хосте
совпал: df8b22b778051882610290d93dc48b40f2e1c661fbab27a1f3188ca21fb891be.
Installed launch audit прошёл обе роли/reverse true/false и отсутствие удалённых
файлов (`results/audit-native-only-launch-20261001.json`). Trace удалил obsolete
fallback поля и проверяет модуль скорости при диагностике заднего хода.
Очищенная версия пока не подтверждена повторной дуэлью. Следующий шаг:
финальная сборка/source audit/installed-launch audit, затем те же seeds12–14.

## 01.10.2026 — изоляция оценки от ручного мира

Предыдущий goal turn — progress: единый YAML/MPC cleanup изменил исходники,
сборка и синтетические аудиты подтвердили wiring. Ручной мир остаётся на тех
же ID; автоматическое продолжение не отвечает на pending permission question.
Следующий доступный безопасный шаг: реальная дуэль в отдельном проекте.
Runner получил --isolated-project/--ros-domain-id/--gazebo-port; Compose
передаёт ROS_DOMAIN_ID/GAZEBO_MASTER_URI и отдельную HSL_RESULTS_DIR. Все docker
exec/cp/trace/readiness/runtime identity проверки адресуют выбранный проект.
Обычный запуск остаётся docker/domain0/master11345. 7 runner tests, compileall
и Compose config прошли. Навигационные исходники и образ не изменены,
пересборка не требуется. Планируемая baseline: текущий config/match.yaml,
fixed explorer first, seeds12–14,90sim s, native MPPI, trace/audit-start,
проект hsl-eval/domain73/port11418. Результаты ещё не получены; дополнительную
нагрузку двух Gazebo учитывать через RTF. Ручной проект не перезапускать.

## 01.10.2026 — единый YAML матча и удаление MPC

**Текущий интерфейс изменён по последнему запросу пользователя.** Один
`helm start_match` читает `config/match.yaml` на хосте, валидирует до запуска,
пересоздаёт duel и после готовности и runtime/source-аудита разрешает движение.
start_match1/2/3, scenarios.py и start_scenario.py удалены. `--prepare-only`
готовит без движения, `--print-env` проверяет без контейнерных изменений,
HSL_MATCH_CONFIG позволяет выбрать другой абсолютный путь.

Конфиг: роли, стартовые map-позы/yaw, размеры квадратных площадок, разрешение
равноправного заднего хода, пределы штатного MPPI, seed/активная длительность,
мир/смещение карты/границы/GUI. Map origin отделён от нашего старта в TF,
карте, адаптерах, метриках и referee. Полигоны обоих стартов передаются
обоим decision manager и referee. Миссионный формат предназначен и для
хакатона, но подключение реальных источников и физическая проверка не выполнены.
Конфиг не проверяет пересечения стартов со стенами: выбирать по карте.

MPC удалён из подмодулей, Docker, Compose, запусков, шлюза, тем и trace.
Шлюз переименован в motion_gate; принимает только прямые MPPI команды
для OK/RECOVERY_MPPI при свежих данных и общем разрешении. Нет резервного
исполнителя или отдельного доворота по Path. Удалены неиспользуемые сглаживание
длинного пути для MPC и логика доворота старого шлюза, старый helper сравнения
MPC и демонстрационные launch. Git checkpoints предыдущих реализаций сохранены.
Python MPPI остаётся явной экспериментальной реализацией; штатный единый
запуск всегда nav2_cpp. Штатный плагин Nav2 не изменён.

Benchmark использует тот же YAML, по умолчанию берёт его роль/seed/длительность;
явные CLI overrides сохраняются отдельно. Копия YAML, эффективная среда,
источники/образы/параметры сохраняются. Внутреннее разрешение движения теперь
вызывает сервисы напрямую, без пересоздания мира через start_match.

Проверки: 144 теста прошли после удаления четырёх тестов удалённых MPC helpers
и добавления девяти проверок конфигурации; compileall и diff-check чистые.
Compose валиден. Изменённые на хосте YAML-позы [1,2,0.7], запрет reverse и
max_speed0.7 корректно отражаются в resolved Compose, полигоны/TF origin
согласованы, без изменения образа. `helm -h` содержит только start_match.
Обе сборки без MPC завершились успешно; финальная после удаления helpers:
`/tmp/mppi-clean-config-build-final.log`, image
`sha256:e5f7d5fb5ae05721b8fc8ab6f537040b5e2f2ed055c244c37a753d44906d1116`.

Изолированный аудит реального stock MPPI (ROS_DOMAIN_ID73, синтетическая
свободная карта, фиксированная поза, без Gazebo и final cmd_vel publisher),
`benchmarks/audit_native_bidirectional.py`, завершился успешно на предыдущем
собранном двунаправленном образе: reverse −0.500м/с, forward +0.500м/с,
CAPTURE face-left omega+1.146рад/с при heading critic=true, resume reverse
−0.500 при critic=false. Каждый этап около40 команд. Отчёт
`results/audit-native-bidirectional-20261001.json`. Это подтверждает команды
и переключение critic, **не** фактическую плавность, скорость матча или поимку.
Новая проверка установленного launch: `benchmarks/audit_match_launch.py`
проверяет обе роли и allow_reverse true/false с изменёнными пределами,
отсутствие swarm_controller и наличие motion.launch.py; все4 случая прошли
на финальном образе, отчёт `results/audit-match-launch-20261001.json`.
Повторный audit_native_bidirectional на финальном образе также прошёл все4
этапа, отчёт `results/audit-clean-mppi-bidirectional-20261001.json`.
Команда аудитов: docker run --rm --network host -e ROS_DOMAIN_ID=73
-v ABS_SCRIPT:/tmp/audit.py:ro --entrypoint bash jr_image:latest -lc
'source /autoware/install/setup.bash && python3 /tmp/audit.py'.
Оба аудита не запускают Gazebo и не публикуют финальный cmd_vel.

Ручной мир пользователя не пересоздавался. Ожидается ответ на ранее заданный
вопрос о его освобождении; разрешение не выведено из истечения времени.
Реальных дуэлей именно очищенной версии нет. Следующий шаг после ответа:
`helm start_match`, проверить стартовые позы/yaw, нулевые команды до/после
матча, reverse true/false и CAPTURE передом; затем сопоставимые короткие серии
с сохранёнными YAML, trace, source/runtime-аудитами. Ложный recovery/затыки,
исследовательское уклонение, неизвестные препятствия, ≥0.3 у каждого и финальные
20 заездов остаются открытыми. Старые записи ниже — история предыдущих деревьев.


**Двунаправленный кандидатf1ccdd1 собран, испытания ожидают освобождения ручного мира.** `helm build duel` завершился успешно (`/tmp/native-bidirectional-build.log`),137 тестов прошли. Текущие контейнеры принадлежат пользовательскому run_id=manual, не последней серии; они не перезапускались. Задан вопрос, можно ли после сборки пересоздать мир для оценки; ответа ещё нет. Последний ручной результат сохранён чтением в `results/manual-snapshot-20261001T160622Z/`: explorer_goal26,0с, скорости explorer/guardian0,342/0,418, контактов0; есть ID/image контейнеров, но нет trace и полного подтверждения source/scenario. Поэтому он не доказывает качество нового двунаправленного варианта и не объясняет сообщённое замирание стража во втором ручном заезде. Следующая оценка после освобождения мира: фиксированные scenario3/seeds19–21 (сравнение с120958Z), затем scenario2/seeds15–17 и scenario1/seeds12–14, trace/audit-start; проверить default ±0,5 и false preferences у обоих, mppi_capture_heading_required только guardian/CAPTURE, фактическую геометрию поимки, скорость/ошибки/плавность/контакты и причины остановок. При старте ожидать GoalAngle=false; во времяCAPTURE параметр будетtrue штатным dynamic callback. Финальную серию20 до360с и неизвестное препятствие ещё не проводили.

**PreferForward отвергнут; по запросу пользователя пробуем равнозначные направления.** Навигация33c6616/runner1c36c09, `results/series-20261001T124943Z/`; `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 3 --start-seed 19 --first-role guardian --active-s 90 --scenario 3 --trace --audit-start`. Три поимки8,8/8,5/6,6с, целей0, контактов0. Explorer/guardian0,161/0,300;0,173/0,283;0,141/0,294м/с; средние0,158/0,292, минимумы0,141/0,283. Explorer ниже0,2 в3/3. RMSglobal0,053/0,043м, angular accel0,964/1,147рад/с², stop-go1,0/1,3. До пробы120958Z скорости0,235/0,344, RMS0,060/0,044. PreferForward уменьшил time-weighted отрицательные команды explorer0,682/0,547/0,630→0,450/0,414/0,220 (left-hold между trace timestamps; покрытие0,983–0,995), но не улучшил результат. Не считать меньшую долю заднего хода успехом. Сборка прошла;137 тестов включают6 runner tests. Первый запуск124704Z отвергнут до движения из-за рассогласования строгого ожидаемого enabled; после его исправления124943Z source/runtime/common-start прошли. After-finish audit этой серии не снят: пользователь заменил контейнеры ручными матчами; текущий referee run_id=manual и ID отличаются от02-runtime.json, его мир не принадлежит оценке.

Новое указание пользователя: перед/зад равнозначны, выбирается меньший доворот; только поимка стража требует переднего направления. В исходниках `vx_min=-0,5` при прежнем `vx_max=0,5` обеим ролям, PreferForward=false, PathAngle.forward_preference=false. В штатном [PathAngleCritic1.1.20](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_mppi_controller/src/critics/path_angle_critic.cpp) этот режим учитывает меньший угол с передом или задом. GoalAngle стартуетfalse и C++-обёртка перед compute включает штатный динамический enabled только при guardian/CAPTURE; terminal yaw уже формирует capture_goal. Диагностика ok сообщает capture_heading_required, trace сохраняет его. GoalG15/E5, прочие critics, sampling, footprint/swept-check и исходный MPC резерв не меняются. 137 тестов/дифф прошли; реального результата пока нет. Текущий manual мир не пересоздавать без ответа на заданный вопрос; пока можно собрать образ. Нужно проверить параметры обеих ролей перед стартом, измерить направления/довороты/плавность/скорость/контакты и capture heading в настоящих матчах. Сообщение пользователя о замирании стража при левом повороте остаётся отдельным дефектом: прежние лог-сообщения не доказывают его причину. Проверенный checkpoint6347bf7 сохранён. Финальные20/0,3 обеих/неизвестные препятствия открыты.

**Проба штатного PreferForward у explorer, ещё без результата.** Проверенный срез487c5e9 чистый, duel-контейнеры остановлены. Анализ active trace120958Z: у explorer seed19–21 отрицательная команда ниже−0,05 в137/214,89/177,158/225 доступных отсчётов; в EVADE98/98,87/89,150/151. Команда ниже−0,34 в93/42/68 отсчётах. Это доли отсчётов, не временные доли. Уход часто продолжается задним ходом на штатном пределе−0,35, хотя G может двигаться вперёд до0,5. В113133Z обратное движение было64/215,45/106,53/222 отсчётов: эффект зависит от исходной ориентации/маршрута, а не универсальный предел скорости. [PreferForwardCritic Nav2 1.1.20](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_mppi_controller/src/critics/prefer_forward_critic.cpp) добавляет мягкую стоимость пройденной назад дистанции; это не запрет reverse, critic отключён около endpoint. Гипотеза: включить этот штатный critic для explorer, сохранив weight5 из YAML, разрешённый reverse в PathAngle, GoalAngle=false, Goal5/GGoal15, диапазоны и obstacle/swept checks. Меняется только enabled у explorer. Нужны сборка и сопоставимые scenario3/seeds19–21 fixedguardianfirst; сравнить фактическую скорость, reverse, lateral/ускорения/контакты/исходы. Если разворот задерживает срочный уход или движение ухудшается, отвергнуть пробу. Это не новое доказательство безопасности или достижения цели.

**После отката:** `helm build duel` прошёл (`/tmp/native-restored-capture-build.log`); `git diff checkpoint/native-guardian-capture-20261001 -- src/` пуст. Затем `helm clean duel` удалил контейнеры испытания829ebf5, чтобы старый запущенный экземпляр не сохранил отвергнутые параметры. Следующий `helm start_match1/2/3` создаст мир из восстановленного образа с GGoal15/EGoal5. Дополнительный заезд после отката не заявляется; доказательства восстановленной навигации — серии113133Z/120958Z и совпадение исходников со срезом6347bf7.

**Explorer GoalCritic15 отвергнут, возвращён5; guardian15 и direct/lead сохранены.** Проба829ebf5, `results/series-20261001T123327Z/`; сборка `/tmp/native-explorer-goal15-build.log`, команда `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 3 --start-seed 19 --first-role guardian --active-s 90 --scenario 3 --trace --audit-start`. Изменён только штатный positional GoalCritic explorer5→15, guardian15/скорости/остальные critics прежние. 131 тест прошёл (`tests/test_navigation.py helm_launch/tests/tests.py`), compileall/diff/Compose config/build прошли. Три поимки7,9/6,6/10,4с, целей0, контактов0. Explorer/guardian0,118/0,340;0,216/0,252;0,143/0,300м/с. Средние0,159/0,297, минимумы0,118/0,252; explorer ниже0,2 в2/3, обе роли ниже0,3 в3/3 и1/3 соответственно (округлённые0,300 у seed21 не считать строгим запасом над порогом). RMSglobal0,101/0,043м, angular accel RMS1,184/1,182рад/с², stop-go2,7/1,7. До изменения120958Z: скорости0,235/0,344, RMSglobal0,060/0,044, ускорение0,882/1,112. Runtime подтвердил Goal15 у обеих и vx_max0,5; source/common-start/after-finish audits прошли. Для устранения разницы длительностей `matched-window-motion.json` сравнивает каждую роль/seed в точном общем referee-relative интервале доступных trace (линейная интерполяция границ, time-weighted speed, производная измеренной body omega): explorer скорости до→после0,169→0,119;0,152→0,218;0,207→0,142. Это также ухудшение в2/3, не только эффект укороченного матча. Body angular accel trace до→после0,762→1,100;1,118→1,625;1,136→0,943рад/с²; способ расчёта отличается от observer RMS, не смешивать их. Гипотеза более уверенного следования escape goal не подтвердилась. Настройка откатывается к проверенной навигации2f67a41; текущий прямой/прогнозный перехват остаётся. Следующий шаг — разбирать маршруты и переходы explorer при встрече, включая доступность обхода/скачки escape endpoints, прежде новых весов; финальные20/0,3 обеих/неизвестные препятствия пока открыты. Исходная MPC-глобальная резервная конфигурация сохранена.

**GoalCritic guardian15 подтверждён в scenario3; прямое сближение и упреждение сохраняются.** Навигация2f67a41, runner6347bf7; `results/series-20261001T120958Z/`. Команда: `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 3 --start-seed 19 --first-role guardian --active-s 90 --scenario 3 --trace --audit-start`. Три поимки14,4/11,4/12,6с, целей0, контактов0. Explorer/guardian по seed19–21:0,249/0,363;0,227/0,315;0,230/0,354м/с. Средние0,235/0,344, минимумы0,227/0,315; ниже0,2 —0/3 обеих, ниже0,3 —3/3 explorer,0/3 guardian. RMSglobal0,060/0,044м, angular accel RMS0,882/1,112рад/с², stop-go1,0/2,0. До параметра guardian15 (111041Z) были три цели,0 поимок, скорости0,260/0,317, RMSglobal0,052/0,042, angular accel0,864/0,934. Разные длительности не позволяют считать рост whole-match angular RMS чистым эффектом настройки. Runtime всех трёх подтвердил GGoal15/EGoal5 и vx_max0,5; source/common-start audits runner прошли; пассивная проверка после финиша `02-gate-after-finish.json` подтвердила10 нулевых команд и одного издателя для каждого робота при common_active=false. Реальное measured_omega_radps записывается в trace. Прямая/прогнозная capture goal выбирается непрерывно по наблюдаемому направлению скорости: встречное движение уменьшает упреждение, поперечное/удаление увеличивает, недопустимый прогноз возвращает текущую цель; внутри радиуса поимки страж ориентируется на текущую наблюдаемую позицию. Эти серии проверяют весь текущий кандидат, а не изолированный эффект смешивания. Проверенный срез сохранён в `checkpoint/native-guardian-capture-20261001` (6347bf7). Слабая роль теперь explorer; следующая одиночная проба — его стандартный GoalCritic5→15 при неизменном guardian15, чтобы проверить следование новой escape goal. Это гипотеза, не подтверждённое улучшение. Финальные20,0,3 обеих, повторные цели текущей версии и неизвестные препятствия остаются открытыми.

**GoalCritic guardian15: три реальные поимки без контактов, explorer пока слабее.** Сборка2f67a41 завершилась (`/tmp/native-guardian-goal15-build.log`). `results/series-20261001T113133Z/`; `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 90 --scenario 2 --trace --audit-start`. Три поимки13,1/7,7/13,2с, целей0, контактов0. Explorer/guardian0,312/0,386;0,202/0,345;0,292/0,384м/с; средние0,269/0,372, минимумы0,202/0,345. Ниже0,2 —0/3 обеих, ниже0,3 —2/3 explorer и0/3 guardian. RMSglobal0,065/0,042м, angular accel0,681/1,022рад/с², stop-go1,3/0,7. До изменения105055Z: три цели,0 поимок; скорости0,265/0,285, RMS0,071/0,040, angular accel0,701/0,684. Runtime всех3 подтвердил Goal15 только у guardian,5 у explorer, vx_max0,5/PathAngle6 сохранены; source/common-start/after-finish audits verified. Это первые повторные поимки текущей улучшенной навигации. Guardian ускорился без заметного роста lateral, но whole-match angular RMS выше; исходы/длительность поменялись, поэтому нельзя без общей временной шкалы объявить плавность прежней. Trace дополнен measured_omega_radps из реального Odometry для следующих общих окон (диагностика, без изменения команд). Срез2f67a41 сохранён как локальная проверенная база, требуетсяscenario3/seeds19–21 fixedguardianfirst для проверки другой физической группы. Если guardian там тоже сильнее, улучшать explorer: проверить движение/реакцию на новую escape goal и штатный GoalCritic, не ослаблять guardian ради50/50. Финальные20/0,3 обеих/повторные цели этой версии/unknown препятствия остаются открытыми.

**Scenario3 direct/lead: guardian ≥0,3, поимок0; одиночный опыт GoalCritic.** Навигация9a6f4b2/runner8bc5462, `results/series-20261001T111041Z/`; `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 3 --start-seed 19 --first-role guardian --active-s 90 --scenario 3 --trace --audit-start`. Три цели17,6/18,7/19,0с, поимок0, контактов0. Explorer/guardian0,270/0,329;0,267/0,312;0,244/0,310м/с; средние0,260/0,317, минимумы0,244/0,310. Ниже0,2 —0/3 обеих, ниже0,3 —3/3 explorer и0/3 guardian. RMSglobal0,052/0,042м, angular accel0,864/0,934рад/с², stop-go1,3/2,0. C7d1713/083248Z до двух forecast правок:0,255/0,304, RMS0,048/0,037, angular accel0,923/0,885; улучшение скорости не доказывает улучшенную плавность guardian. Runtime/source/common-start/after-finish audits verified. Слабое место — поимка: средняя скорость достаточна, но близкое сближение всё ещё не закрывает0,45м. В [GoalCritic Nav2 1.1.20](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_mppi_controller/src/critics/goal_critic.cpp) стоимость — средняя дистанция всей rollout до конечной точки; [PathFollowCritic](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_mppi_controller/src/critics/path_follow_critic.cpp) отключается у конечной точки (наш threshold0,35м). Это основание гипотезы слабого слежения за движущейся capture goal, а не доказанный дефект исходника Nav2. Следующий один параметр: launch задаёт стандартный MPPI.GoalCritic.cost_weight15 у guardian (было5), explorer остаётся5. Диапазоны скоростей, prediction, GoalAngle, PathAngle6, препятствия, полный swept-check и endpoint cache прежние. 139 тестов прошли; кандидат ещё не собран и реальный эффект неизвестен. Нужны helm build duel, прежниеscenario2/seeds15–17 сtrace/audit-start, проверить фактические role-specific веса вruntime, min separation/поимки/скорость/ускорения/контакты; затемscenario3, если опыт полезен. Финальные20/0,3 обеих/повторные поимки и unknown препятствия открыты.

**Близкий direct/lead проверен: движение сохранено, поимки ещё не достигнуты.** Сборка9a6f4b2 завершилась (`/tmp/native-moving-capture-build.log`). `results/series-20261001T105055Z/`; `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 90 --scenario 2 --trace --audit-start`. Роли чередовались. Три цели21,6/21,0/21,7с, поимок0, контактов0. Explorer/guardian0,279/0,284;0,254/0,287;0,262/0,283м/с; средние0,265/0,285, минимумы0,254/0,283. Ниже0,2 —0/3 обеих, ниже0,3 —3/3 обеих. RMSglobal0,071/0,040м, angular accel0,701/0,684рад/с², stop-go2,3/0,7. До изменения103318Z:0,259/0,280, RMS0,068/0,044, angular accel0,673/0,782, stop-go2,7/1,7. На этой серии guardian стал ровнее по ускорению/stop-go, но цели explorer и0 поимок не позволяют объявить цель достигнутой. В75 уникальных CAPTURE циклах helper реально применил прогноз73 раза и current pose2 раза (`capture-diagnosis.json`); вес прогноза непрерывен, поэтому небольшая ненулевая смесь тоже обозначается lead, не отдельной жёсткой тактикой. Interpolated measured min separation0,473/0,586/0,503м, close speed mean0,237/0,256/0,251м/с (`guardian-approach-metrics.json`); это измерения собственных поз обоих роботов, не вход навигации. Все min ещё за радиусом0,45; первый heading error0,089рад при минимуме, но поимки нет. Guard planner p95 compute149/147/156мс на период200мс, native29/35/33мс на50мс; вычислительного deadline failure здесь нет. Runtime/source/common-start/after-finish audits verified. Сохраняется кандидат9a6f4b2 для проверкиscenario3 сфиксированным guardian первым/seeds19–21; далее сравнить прогноз/конец пути при переходах PURSUE↔CAPTURE и причины замедления до радиуса. Прямую цель в центре корпуса без проверки защиты/остановки не добавляли. Финальные20,0,3, повторные поимки, переносимость/неизвестные препятствия остаются открытыми.

**Прогноз вдоль стены: ложный отказ устранён, поимки не выросли; кандидат близкого перехвата.** Навигация097b578, `results/series-20261001T103318Z/`; `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 90 --scenario 2 --trace --audit-start`. Три цели22,0/19,9/23,6с, поимок0, контактов0. Explorer/guardian0,263/0,273;0,269/0,305;0,245/0,263м/с; средние0,259/0,280, минимумы0,245/0,263. Ниже0,2 —0/3 обеих, ниже0,3 —3/3 и2/3. RMSglobal0,068/0,044м, angular accel0,673/0,782рад/с², stop-go2,7/1,7. Прежняя сопоставимая101948Z:0,267/0,280, RMS0,062/0,040, angular accel0,671/0,652. В194 циклах PURSUE28 геометрических отказов:17 blocked endpoint,3 blocked segment,8 вне границы, чистого recovery-margin отказа0. Семантическое исправление оставлено, но улучшение плавности/поимок не подтверждено; endpoint lag>0,15м остаётся66 циклов. Runtime/source/common-start/after-finish audits прошли. Замер двух прежних серий по интерполированным собственным позам (`guardian-approach-metrics.json`) показывает min separation0,509–0,588м и среднюю скорость при separation<0,8м0,130–0,257м/с; это метрики, ground truth не поступает в управление. CAPTURE до сих пор всегда целился в статическую текущую позицию, даже при удалении соперника. По уточнению пользователя нужны прямое сближение и упреждение по ситуации. Новый кандидат `moving_capture_goal`: при head-on подходе/неподвижности/уже достигнутом радиусе0,45 — прежний direct capture; при поперечном/удаляющемся движении — короткий прогноз, плавно смешанный по радиальной компоненте. Горизонт≤1с и время подхода к текущему сопернику по собственной известной скорости (native0,5; MPC0,3), без ограничений скорости соперника. Прогноз проверен по карте/robot_radius; при стене/границе/недостижимом capture goal возвращается прямой вариант. Диагностика добавляет capture_strategy иcapture_enemy_prediction. Own A*/swept-check, velocity limits, MPPI critics, recovery и кэш маршрута не меняются. 139 тестов прошли: receding/crossing/head-on/stationary, rival1м/с, стена и ориентация уже внутри capture range; compileall/diff check чистые. Кандидат ещё не собран/не проверен дуэлью. Следующий шаг — сборка и scenario2 seeds15–17, затемscenario3; следить за контактом/overshoot/угловым ускорением/вычислительными задержками и фактической поимкой. Финальные20/0,3/повторные поимки и неизвестные препятствия открыты.

**Same-cycle диагностика подтверждена в ROS; кандидат исправляет прогноз вдоль стены.** Сборка3e9b223 прошла (`/tmp/native-pursuit-diagnostics-build.log`). Одиночный scenario2 seed15 `results/series-20261001T101600Z/`: цель19,6с, explorer/guardian0,289/0,222м/с, контактов0; RMSglobal0,034/0,058, angular accel0,887/1,391, stop-go2/6. Guardian долго потерял видимость, поэтому один этот результат не использовался как достаточное объяснение. Неизменный повтор3e9b223 `results/series-20261001T101948Z/`, команда `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 90 --scenario 2 --trace --audit-start`: три цели21,2/20,3/23,0с, поимок0, контактов0. Explorer/guardian0,289/0,288;0,257/0,300;0,255/0,251м/с; средние0,267/0,280, минимумы0,255/0,251; ниже0,2 —0/3 обеих, ниже0,3 —3/3 и2/3. RMSglobal0,062/0,040м, angular accel0,671/0,652рад/с², stop-go2,7/1,3. В188 уникальных циклах PURSUE69 отклонений. После отдельного учёта endpoint_inside:false —40 только из-за recovery margin,22 blocked endpoint,1 blocked segment и6 вне допустимой границы (подробности вpursuit-diagnosis.json). Первоначальный счёт42 смешивал два таких отказа с границей и исправлен. Отдельно60 повторных маршрутов имеют endpoint lag>0,15м. Runtime/source/common-start audits verified; after-finish audit дополнительно сохранён. Решение: reachable_intercept проверяет наблюдаемый прогноз с safety_margin0 вместо дополнительного recovery margin0,12; robot radius, occupied/границы/проверка стен остаются. Это не изменение recovery и не отключение собственного A*/MPPI swept-check. Новый тест показывает различие: прежний recovery не разрешает движение вдоль близкой стены, прогноз соперника сохраняет достижимое упреждение; прежняя проверка пересечения стены остаётся. 137 тестов прошли. Кэш endpoint пока не меняется, чтобы разделить причины. Кандидат ещё не собран/не проверен дуэлью; следующий шаг — helm build duel, тот жеscenario2/seeds15–17, затемscenario3/новыеseed. Финальные20/0,3/устойчивые поимки и неизвестные препятствия открыты.

**PathAngle6, scenario2: скорость ≥0,2 сохранена, поимок нет; добавлена диагностика целей.** Навигацияc7d1713/runner02e993d, `results/series-20261001T095424Z/`; `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 90 --scenario 2 --trace --audit-start --probe-status NO_GLOBAL_PATH`. Роли чередовались. Три цели21,8/21,7/20,8с, поимок0, контактов0. Explorer/guardian скорости0,273/0,279;0,247/0,283;0,276/0,297м/с; средние0,265/0,286, минимумы0,247/0,279. Ниже0,2 —0/3 обеих ролей, ниже0,3 —3/3 обеих. RMSglobal0,066/0,037м, angular accel RMS0,697/0,629рад/с², stop-go2,7/0,3. Runtime/hash/common-start и after-finish audit проверены. Поздний02-live-guardian.json снят после финиша (WAIT_OR_STOP), поэтому не доказывает активное преследование. В trace seed15 у guardian raw intent иногда опережает published route end на0,5–1м; сообщения имеют разные stamps, конкретное звено пока не установлено. Добавлен readonly `navigation/planning_diagnostics` с входами и стадиями raw→validated→smoothed→reachable, cached target/end, route source и геометрией отклонённого intercept в одном цикле. Проверка margin0 — только диагностический контрфактуал; управление/проверки безопасности не изменены. Trace сохраняет JSON в planning и точный sim stamp. 136 тестов прошли, compileall/diff check чистые. Следующий шаг: пересобрать этот срез и повторить scenario2 seed15 сtrace/audit-start, затем выбрать одно изменение по фактическим данным. Новая диагностика ещё не проверена в ROS. Финальные20/0,3/повторные поимки и неизвестные препятствия остаются открытыми.

**PathAngle6: оба ≥0,2 в scenario3, отклонение ниже; smoothness по ускорению хуже.** c7d1713, `results/series-20261001T083248Z/`; `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 3 --start-seed 19 --first-role guardian --active-s 90 --scenario 3 --trace --audit-start --probe-status NO_GLOBAL_PATH`. Все native параметры подтвердили weight6/role flags; runtime/hash/common-start audits verified. Три цели19,0/21,0/18,4с, поимок0, контактов0; explorer/guardian0,267/0,311;0,226/0,281;0,271/0,320м/с. Средние0,255/0,304, минимумы0,226/0,281; ниже0,2 —0/3 обеих, ниже0,3 —3/3 explorer и1/3 guardian. RMSglobal0,048/0,037м против0,072/0,048 у weight2; stop-go1,3/1,7 против3,3/2,0. Angular accel RMS0,923/0,885 против0,734/0,749: сильнее начальный поворот может объяснять часть роста, но плавность по этому показателю улучшенной не объявляется. Weight6 временно оставлен для проверки других сценариев; global/decision ещё не обеспечили поимок. Следующий неизменный опыт — scenario2 seeds15–17 с чередованием ролей, trace дополнительно хранит raw intent target и наблюдаемую velocity соперника, чтобы отличить неверный intercept от удержания старого endpoint. Реальный исход этой следующей серии пока неизвестен. Финальные20/0,3/устойчивые поимки и unmapped препятствия открыты.

**RViz prefix проверен визуально; PathAngle6 собран, сравнение запущено.** Сборкаc7d1713 завершилась (`/tmp/native-path-angle6-build.log`). После завершения081214Z перезапущен только hsl-rviz через Compose `up -d --no-deps --force-recreate hsl-rviz`, сHSL_RVIZ_ENABLED=auto; разрешения движения обоих оставались закрытыми. `rviz-prefix-fixed.png` в081214Z показывает две отдельные модели у собственных TF внутри лабиринта, статусы обоих RobotModel/TF/карты нормальные; container config проверен byte-for-byte сsrc (docker cp -L нужен из-за symlink-install). Активные global/local пути уже подтверждены01-rviz-active.png прежней серии; этот повтор проверяет отображение моделей после финиша, не новую навигацию. GLSL warning по-прежнему есть при реально отображаемой карте, его причина не установлена. Теперь запущено `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 3 --start-seed 19 --first-role guardian --active-s 90 --scenario 3 --trace --audit-start --probe-status NO_GLOBAL_PATH` (`/tmp/native-path-angle6-series.log`), без GUI; сравнить с075953Z. Пока результатов нового веса нет. Финальные20, повторные поимки текущей навигации,0,3 и неизвестные препятствия остаются открытыми.

**Native кандидат PathAngle weight6, ещё без реального прогона.** Единственное изменение навигации послеb395b87: стандартный MPPI.PathAngleCritic.cost_weight2→6 для обоих роботов. Гипотеза — раньше получить курс на маршрут при большом начальном угле, повысить среднюю скорость второго explorer и уменьшить стартовое отставание guardian. Может ухудшить smoothness/stop-go, поэтому сравнить seeds19–21/first-role guardian/scenario3 с075953Z, затем роли/scenario1. В том же образе находится независимое исправление RViz prefix/нейтральных подписейc2b328a; оценочный повтор будет headless, GUI проверяется отдельным матчем. Остальные critics, role flags, .23 footprint, velocity limits и swept-check не менялись. Checkpoint/native-objective-navigation-20261001 сохраняет навигациюb395b87 (с ещё не проверенной визуальной правкойc2b328a). YAML/Compose/diff проверки перед сборкой; реальный результат кандидата пока отсутствует.

**Регрессия scenario1 с GUI: скорость сохранена, guardian всё ещё слабее по исходам.** Runner280fa0c/навигацияb395b87, `results/series-20261001T081214Z/`; `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace --audit-start --rviz`. Три цели18,0/19,3/19,4с, поимок0, контактов0; explorer/guardian0,317/0,381;0,297/0,343;0,307/0,367м/с. Средние0,307/0,364, минимумы0,297/0,343; ниже0,2 —0/3 обеих, ниже0,3 —1/3 explorer и0/3 guardian. RMSglobal0,039/0,048м, angular accel RMS0,636/0,697рад/с², stop-go1/0. Runtime/все before-partial audits/after-finish verified. `01-rviz-active.png` показывает карту, TF, LiDAR и глобальные/локальные пути в активном матче, но второй RobotModel пока наложен на первого из-за отсутствующего prefix. `arena-clearance.json` проверяет фактические позиции всех6 отчётов в окне referee: минимальный запас центра от границ0,388м, больше физического радиуса0,178м; все измеренные положения тел внутри арены. Это ограничено данной серией, не доказательство всех будущих маршрутов. По сравнению с054837Z другие исходы и GUI нагрузка, сравнение RTF не считать тождественными условиями. E21 scenario3 всё ещё0,159, повторных поимок текущего кандидата нет; финальную20 ещё нельзя объявлять итоговой. Следующий одиночный параметрический опыт native — усилить PathAngleCritic (weight2→6) для сокращения медленного начального выравнивания: trace guardian seed19 тратит первые3–4с на большой разворот при малой поступательной скорости; команды omega сначала0,4–1,0 при штатном пределе1,5. Это гипотеза, возможен рост колебаний. Скоростные диапазоны/остальные critics/модель/проверки оставить. GUI prefix исправить отдельно от навигационной гипотезы и проверить живым снимком после сборки.

**GUI regression в процессе: RViz открывается, найден пропущенный префикс второго RobotModel.** Запущена неизменная навигацияb395b87/runner280fa0c: `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace --audit-start --rviz`, `results/series-20261001T081214Z/`; DISPLAY=:1, Gazebo headless, RViz штатно auto. Снимки00-rviz-before-start.png/00-rviz-live.png подтверждают окно, карту и LiDAR. Первоначальные URDF/TF ошибки ушли после готовности Gazebo; оба RobotModel зелёные, но визуальные модели совмещены у первого, тогда как TF второго отдельно. URDF второго имеет прежние link names, robot_state_publisher добавляет opponent/ к TF; RobotModel без TF Prefix использует кадры первого. [Исходник RViz Humble](https://raw.githubusercontent.com/ros2/rviz/humble/rviz_default_plugins/src/rviz_default_plugins/displays/robot_model/robot_model_display.cpp) подтверждает свойство TF Prefix и его передачу TFLinkUpdater. В конфиг добавлен opponent для второго, названия моделей/путей/сканов теперь First/Second robot, чтобы перестановка ролей не давала ложных подписей. Текущая серия использует старый RViz config; после неё нужны сборка и реальная повторная визуальная проверка новой конфигурации. В логе есть GLSL sampler error, но снимок доказывает, что карта реально отображается; причина лога не установлена и не считается исправленной. Навигационные алгоритмы/скорости этой GUI-правкой не меняются.

**EVADE с настоящей целью: повторные цели быстрее, один explorer всё ещё ниже порога.** `b395b87`, `results/series-20261001T075953Z/`; `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 3 --start-seed 19 --first-role guardian --active-s 90 --scenario 3 --trace --audit-start --probe-status NO_GLOBAL_PATH`. Runtime/hash/flags/common-start verified. Три цели за22,0/20,8/21,1с, поимок0, контактов0. Explorer/guardian0,248/0,269;0,246/0,281;0,159/0,282м/с. Средние0,218/0,277, минимумы0,159/0,269; ниже0,2 —1/3 и0/3, ниже0,3 —3/3 обеих ролей. RMSglobal0,072/0,048м против0,074/0,050 в074123Z; angular accel RMS0,734/0,749 против0,737/0,711, stop-go3,3/2,0 против4,7/2,0. Recovery/NO_GLOBAL_PATH0, OK0,986–0,991, небольшое WAIT_OR_STOP относится к начальной задержке common-state. Before/partial audits всех3 passed, after finish audit сохранён. Срез поддерживает гипотезу лишнего короткого отхода и оставлен, но guardian здесь не ловит и скорость explorer seed21 не выполнена. Нужна регрессия другого сценария/ролей перед тюнингом native: seeds12–14 scenario1. Для проверки GUI runner получил `--rviz` (auto DISPLAY, Gazebo headless); trace записывает global head/end и наблюдаемый enemy для разбора начального промедления/смены пути. Это диагностика, навигационные исходники не меняются; графический автозапуск ещё нужно увидеть вживую. Финальные20/0,3 и неизвестные препятствия остаются открытыми.

**Кандидат: уход с сохранением маршрута к площадке.** Перед короткой EVADE-точкой planner пробует A* к известной настоящей цели с прежней зоной predicted opponent; дополнительно проверяет всю трассу относительно текущего наблюдаемого enemy и начальное направление от него. Если безопасный маршрут существует, используется он; иначе сохраняется прежний checked departure. Сохраняются native MPPI, диапазоны скоростей, swept-check, наблюдаемая оценка соперника и watchdog recovery для неизменного objective. Старый recovery отменяется только при замене retained departure настоящей целью. На сохранённом observed snapshot seed21 helper даёт15 клеток до площадки (`02-evade-objective-comparison.json`); это offline проверка функции, не дуэль кандидата. Три регрессии проверяют преследователя сзади, несовпадение prediction/current и запрещённую цель. 136 тестов (`python3 -m pytest -q tests helm_launch/tests/tests.py`), Compose config/compileall/diff check прошли. Сборка b395b87 завершилась успешно (`/tmp/native-evade-objective-build.log`); запущен сопоставимый повтор seeds19–21/first-role guardian/scenario3 сtrace/audit-start/probe NO_GLOBAL_PATH (`/tmp/native-evade-objective-series.log`). Процесс ещё работает, результатов кандидата пока нет. Срез до изменения сохранён tag checkpoint/native-position-goals-20261001 на74694d0.

**Повтор фиксированного второго explorer: цели повторились, скорость всё ещё слабая.** `05d9506`, `results/series-20261001T074123Z/`; `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 3 --start-seed 19 --first-role guardian --active-s 90 --scenario 3 --trace --audit-start --probe-status NO_GLOBAL_PATH`. Все3 runtime/common-start audits verified. Seed19 поимка46,9с;20 цель27,0с;21 цель27,2с. Explorer/guardian0,191/0,194;0,215/0,226;0,132/0,241м/с. Средние0,179/0,220, минимумы0,132/0,194; ниже0,2 —2/3 и1/3, контактов0. RMSglobal0,074/0,050м, angular accel RMS0,737/0,711рад/с², stop-go4,7/2,0. Отключение GoalAngle сохранено как соответствующее позиционной задаче, но общий критерий не достигнут. Seed19 фактически прошёл верхний коридор и восточный обход, не остановился в первом тупике; `00-tracks.png` показывает траектории. Known-map counterfactual использует измеренную позу guardian только offline, исключает динамический скан/предсказание и не является replay наблюдений. В полном observed snapshot seed21 (`02-probe-second.json`, sim686,007) fresh departure даёт7 клеток, а маршрут к реальной площадке с тем же observed prediction —15 клеток. Снимок асинхронный, поэтому не доказывает точное состояние retained waypoint в момент NO_GLOBAL_PATH. Код независимо подтверждает, что близкий enemy всегда подменяет objective короткой точкой отхода, даже при существующем безопасном пути к площадке. Следующая изолированная гипотеза — прежде короткого отхода пробовать безопасный путь к настоящей цели, с начальным движением от наблюдаемой угрозы; native critics/скорости/проверки не менять. Финальные20/0,3 и GUI остаются открытыми.

**GoalAngle explorer: плавность лучше, скорость второго старта ещё не выполнена.** `05d9506`, `results/series-20261001T073759Z/`; `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 1 --start-seed 19 --first-role guardian --active-s 90 --scenario 3 --trace --audit-start --probe-status RECOVERY_MPPI`. Официальный native, SHA256 и эффективный GoalAngle=false только у explorer проверены. Поимка16,5с, контактов0; explorer/guardian0,150/0,178м/с, обе ниже0,2. RMSglobal0,078/0,054м, angular accel RMS0,547/0,890рад/с², stop-go1/4. Прежний seed19 с той же физической ролью070527Z:0,143/0,208; RMS0,084/0,052; angular accel1,127/0,620; stop-go3/2. Explorer стал плавнее, но скорость не выполнена, guardian тоже ниже порога; одиночное сравнение не доказывает повторяемость. Recovery0, explorer NO_GLOBAL_PATH0,073, OK0,909. Before/partial/after audits passed. В trace explorer сначала едет вниз, затем при сближении меняет маршрут назад; EVADE отходит вверх, у16с останавливается с NO_GLOBAL_PATH. Это повод исследовать доступность обхода и выбор выхода, а не увеличивать скорость или ослаблять проверки. Запущен неизменный повтор3 заездов seeds19–21 с фиксированным первым guardian и probe NO_GLOBAL_PATH (`/tmp/native-position-only-goal-repeat.log`); результаты ещё ожидаются. Passive probe теперь дополнительно сохраняет свободные/занятые клетки и исходные static/scan точки в world_snapshot для offline проверки обходов; это не меняет управление. Первый probe этой уже запущенной серии скопирован до расширения, следующие копируют расширенный вариант. Compileall/diff check passed. Финальные20, скорость0,3, GUI и новые препятствия остаются открытыми.

**Повтор PreferForward explorer: обе задачи есть, левый старт остаётся медленным.** `12836e6` (навигацияda7f83f, image8e0e1982, runtime SHA256/flags/common-start verified), `results/series-20261001T070527Z/`; команда `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 3 --start-seed 18 --active-s 90 --scenario 3 --trace --audit-start --probe-status RECOVERY_MPPI`. Seed18 цель16,0с,19 поимка13,7с,20 цель15,5с. Explorer/guardian скорости0,320/0,305;0,143/0,208;0,318/0,315м/с. Средние0,260/0,276, минимум0,143/0,208; ниже0,2 —1/3 и0/3; ниже0,3 —1/3 у обеих ролей. Контактов0, recovery0; explorer OK0,994/0,978/0,994. RMSglobal0,067/0,048м, angular accel RMS0,841/0,762рад/с², stop-go2,3/1,3. Before/partial audits прошли на всех3; after-finish audit сохранён. Дополнительный probe NO_GLOBAL_PATH для explorer seed19 не сработал до reset мира; пустой файл не является снимком. Повтор seed18 дал другой исход, чем070004Z, поэтому сравнение одиночного seed не доказывает детерминированность. Предпочтение reverse оставлено: повторные цели и скорость первого физического explorer улучшены, но критерий каждого робота ещё не выполнен.

**Следующий изолированный опыт: убрать ненужную конечную ориентацию explorer.** В seed19 explorer на втором старте сначала выполняет медленный поворот на90°, а после перехода EVADE движется назад вверх; к12–13с снова поворачивает при малой скорости около короткой точки отхода. Native reference для обычных задач задаёт terminal yaw по касательной; при reverse это противоположно текущему корпусу, хотя explorer не имеет задачи конечной ориентации. [GoalAngleCritic1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_mppi_controller/src/critics/goal_angle_critic.cpp) при близости к концу пути штрафует отличие rollout yaw от terminal yaw. В кандидате только explorer получает `MPPI.GoalAngleCritic.enabled=false`; guardian сохраняет true для геометрии поимки. PathAngle weights/скорости/модель/проверки не меняются. Это корректировка требований к цели; влияние на промедление ещё надо проверить.133 теста, compileall и diff check прошли. Для воспроизведения именно второго старта runner теперь имеет `--first-role guardian` (фиксирует роли), default alternate сохранён; серия сохраняет role_assignment. Следующая команда после сборки: `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 1 --start-seed 19 --first-role guardian --active-s 90 --scenario 3 --trace --audit-start --probe-status RECOVERY_MPPI`. Сравнить с explorer второго старта070527Z/seed19, затем повторить обе ориентации/сценарии. Финальные20 остаются открытыми.

**Отключение PreferForward explorer: один сопоставимый заезд улучшил скорость.** `ae526f8` (навигацияda7f83f, image8e0e1982; SHA256/эффективные role flags проверены), `results/series-20261001T070004Z/`; команда `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 1 --start-seed 18 --active-s 90 --scenario 3 --trace --audit-start --probe-status RECOVERY_MPPI`. Поимка19,7с, целей0, контактов0. Explorer/guardian0,224/0,245м/с против0,131/0,230 у общего старта064245Z. RMSglobal0,067/0,052 против0,069/0,046м, angular accel RMS0,792/0,770 против0,563/0,608рад/с², stop-go2/2 против2/1. Explorer recovery0, NO_GLOBAL_PATH0,076, OK0,914; guardian OK0,990. Before/partial audit обаpassed; after-finish audit сохранён. Один заезд поддерживает гипотезу штрафа reverse, но не доказывает повторяемость; плавность по ускорению хуже более медленного baseline, хотя поперечная ошибка explorer почти не изменилась. Ни одна роль здесь не достигает0,3, цели explorer нет. Следующий неизменный опыт — scenario3 seeds18–20 сaudit-start/trace, затем регрессия scenario1/2 при сохранении скорости, обеих задач и отсутствия контактов. Финальные20 ещё не выполнены.

**Опыт PreferForward: образ собран, исправлено чтение ROS параметров.** Навигацияda7f83f, image `sha256:8e0e19821a13163186d12787665cc712b5dc89f30cff3655f305f6aab5851ccb`, лог `/tmp/native-reverse-critic-build.log`. Попытка `results/series-20261001T065402Z/` отвергнута до движения: проверка искала плоский MPPI.PathAngleCritic.forward_preference, тогда как ros2 param dump возвращает вложенный MPPI/PathAngleCritic. Прямой ros2 param get подтвердил PreferForwardCritic.enabled=false у explorer. В runner добавлено чтение обоих представлений; регрессия metadata использует реальную вложенную форму.5 адресных тестов passed, compile/diff clean. Навигационного исхода нет; после исправления нужен новый сопоставимый seed18 сaudit-start. Срез синхронизации сохранён local tag checkpoint/native-common-start-20261001 наbda86fa. Финальная оценка остаётся открытой.

**Общий старт/финиш подтверждены реальным заездом.** `bda86fa` (навигацияf6ede84, image9bfd9797, runtime SHA256/require_match_active проверены), `results/series-20261001T064245Z/`; команда `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 1 --start-seed 18 --active-s 90 --scenario 3 --trace --audit-start --probe-status RECOVERY_MPPI`. Before/partial audit оба passed: сначала permissions false/false, затем true/false, common_active=false, по10 нулевых cmd_vel/единственный издатель. В полной трассе0 ненулевых команд до started_at у обоих (до исправления22 у первого). После finish audit тоже passed/common_active=false. Исход поимка22,3с, контактов0; скорости explorer/guardian0,131/0,230м/с, RMSglobal0,069/0,046м, angular accel RMS0,563/0,608рад/с², stop-go2/1. Это доказательство синхронизации, не достижения скорости исследователя или успеха всех сценариев. Стандартные скорости/critics в этом опыте не менялись.

**Следующий отдельный опыт: согласовать разрешённый reverse explorer со штатными critics.** В Nav2 1.1.20 PreferForwardCritic штрафует интеграл отрицательной vx, при нынешнем weight5. PathAngle.forward_preference=false у explorer допускает ориентацию для обратного движения и не требует доворота при ошибке меньше1рад; вместе с высоким штрафом reverse это может давать малую скорость на пути позади. Это гипотеза причины промедления, а не экспериментально доказанный дефект библиотеки. В launch только для explorer задаётся стандартный `MPPI.PreferForwardCritic.enabled=false`; guardian сохраняет оба forward-предпочтения. Диапазоны −0,35..0,5/1,5, costmap, collision/swept-check, batch/horizon/остальныеcritics не меняются. Runner проверяет эффективные значения обеих ролей. Следующая сборка и одиночный сопоставимый seed18 scenario3 с теми же audit-start/trace, затем повторные seeds/scenarios при улучшении. Источники: [PreferForwardCritic1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_mppi_controller/src/critics/prefer_forward_critic.cpp), [PathAngleCritic1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_mppi_controller/src/critics/path_angle_critic.cpp). Финальные20 и пороги0,2/0,3 всё ещё не выполнены.

**Проверка общего старта подготовлена.** Навигационный commitf6ede84 собирается через `helm build duel`, лог `/tmp/native-common-start-build.log`. В host runner добавлен `--audit-start`: пассивный audit до permissions, затем включение только первой permission и пассивный audit exactly-one-permission/common_active=false/оба cmd_vel нули/единственный издатель. После этого обычный start_match включает вторую permission. Runtime snapshot требует require_match_active=true у обоих planner/gate. Audit теперь также проверяет общий Bool; unit/CLI132 теста прошли. Следующий реальный заезд: `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 1 --start-seed 18 --active-s 90 --scenario 3 --trace --audit-start --probe-status RECOVERY_MPPI`. Это проверка изменения границы старта; настройку PreferForward не смешивать с ней.

**Кандидат альтернативных маршрутов проверен; скорость всё ещё недостаточна.** `2a9765f` (навигационные исходники3349c43, образ60a3ee23, SHA256 и роли/seed/run_id проверены), `results/series-20261001T062532Z/`; команда `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 3 --start-seed 18 --active-s 90 --scenario 3 --trace --probe-status RECOVERY_MPPI`. Seed18/19/20 — поимки27,8/20,1/23,5с, целей0, контактов0. Explorer/guardian скорости0,135/0,191;0,170/0,187;0,123/0,211м/с. Средние0,143/0,196, минимум0,123/0,187; ниже0,2 —3/3 и2/3, ниже0,3 —3/3 у обеих ролей. RMS global0,067/0,054м, angular accel RMS0,791/0,658рад/с², stop-go3,0/2,7. На сопоставимых seed18/19 базовые скорости0,131/0,205 и0,121/0,153; эффект неоднороден, стабилизация не достигнута. Explorer seed18 recovery0,086, seed19 recovery0 (NO_GLOBAL_PATH0,025, NO_LOCAL_PATH0,01). Global compute p95 первых двух seed136,73–149,86мс, native30,34–32,04мс, RTF0,548–0,553. После серии audit подтвердил оба закрытых gate, одного издателя и10 нулевых команд. Проба seed18 при OK/recovery показывает безопасный глобальный путь и очень медленное начальное движение; выбор альтернативы не объясняет это промедление. Следующий опыт локального MPPI — проверить конфликт reverse-разрешения/PathAngle.forward_preference=false с сильным PreferForward у explorer, отдельно от изменения старта.132 теста для следующего общего разрешения прошли; финальные20/пороги не выполнены.

**Найдена и исправляется синхронизация старта.** В старой трассе первого робота19 (scenario1) и22 (scenario3) pose-отсчёта имеют ненулевой cmd_vel до общего started_at referee; первый сервис разрешается примерно на1,4с раньше второго. Второй робот при этом ещё закрыт. Метрики всё равно используют общее окно, поэтому эти начальные команды не включались в его среднюю скорость. Это нарушение совместного старта, не успех движения. Новый referee публикует transient-local `/match/active` Bool: true только после обеих permissions, false до старта/при снятии одной permission/немедленно после исхода. Gate и global planner в автономном launch требуют свежую true (heartbeat10Гц, тайм-аут0,5с sim); до неё planner очищает progress/watchdog, gate выдаёт нули. Исходный MPC и MPPI-параметры не менялись. Неавтономный прямой запуск gate/planner сохраняет default require_match_active=false; автономные launch передают true.132 теста, compileall и diff check прошли. Сборка и реальные проверки половинного разрешения/отсутствия pre-start команд ещё впереди; не считать синхронизацию доказанной модульным тестом.

**ROS-параметр ревизии исправлен как строка.** Попытка `results/series-20261001T062059Z/` не стартовала: short SHA5787e30 в CLI YAML превратился в DOUBLE5,787e33, тогда как referee и оба metrics ожидают STRING. Это технический сбой до разрешения движения; навигации/исходов0. В Compose все три code_revision теперь передаются с буквальными YAML-кавычками, сохранёнными после двух уровней разбора команды. Проверены actual docker compose config и отдельный живой ROS probe: значение5787e30 имеет типstr. Регрессия также проверяет цифровую ревизию0123456 и обычный hash.131 тест прошёл, diff check. Образ навигации3349c43 повторно собирать не нужно: изменены host Compose/runner/probe, src совпадает с image60a3ee23. Следующий запуск — повтор scenario3 seeds18–20 с trace/probe RECOVERY_MPPI на собранном кандидате.

**Оценочный runner отделяет свою серию от ручного запуска.** По `02-runtime.json` прерванного scenario3 подтверждено: ещё до команды старта referee имел run_id=manual, seed0, revision unknown, max_active_s360; оба planner/native seed0/1. Поэтому этот заезд не оценочен, даже несмотря на совпадение исходников. В runner добавлена проверка run_id/seed/revision/лимита/ролей/стартовой площадки и arena bounds до разрешения движения, а также проверка ID и running-состояния контейнеров каждые5с ожидания outcome. Observation timeout остаётся unknown и не вызывает повторный запуск; подтверждённая остановка/замена прерывает оценку. Cleanup не вызывает stop_match у заменённого или неподтверждённого окружения.130 тестов прошли. Проверка реальных сохранённых snapshots приняла00/01 и отклонила02 по run_id; live Docker подтвердил stopped. Это не устраняет возможность одновременного ручного запуска с общими топиками: запускать их отдельно.

Образ кандидата3349c43 собран командой `helm build duel`, лог `/tmp/native-frontier-build.log`, image `sha256:60a3ee234408915c4cd846882033aa6fda279680131ac8477ebee4c083b8eaf5`. Контейнеры duel после внешней остановки не перезапускались при сборке. Навигационный эффект ещё не проверен. Runner-изменения выполняются на host и не требуют отдельной пересборки образа.

**Scenario3 выявил слабую навигацию; серия прервана отдельно от исходов.** `95eeb9f` (навигация2da0c37), `results/series-20261001T060117Z/`; команда `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 3 --start-seed 18 --active-s 90 --scenario 3 --trace`. Seed18 поимка19,7с,19 поимка12,1с; explorer/guardian0,131/0,205 и0,121/0,153м/с. Контактов0. Explorer среднее/минимум0,126/0,121, guardian0,179/0,153; ниже0,2 —2/2 и1/2. RMS global0,070/0,050м, angular accel RMS0,915/0,694рад/с², stop-go3,5/2,0. Seed18 explorer до появления угрозы почти стоял при OK; это отдельная оставшаяся проблема локального управления, не объяснённая исправлением выбора альтернативы. Seed20 не имеет оценочного исхода: все duel-контейнеры вышли137 в06:09:21UTC (OOM=false), latest_outcome имеет run_id manual; вмешательство в общее окружение не засчитывается победой или тайм-аутом. Сохранён02-interruption.json, runner остановлен SIGINT после подтверждения остановки контейнеров. Для новых автоматических серий уточняется, не идут ли ручные проверки пользователя.

**Кандидат: достижимые альтернативы перед recovery, эффект ещё не проверен.** В коде fallback после пустого A* пробовал только одну ближайшую frontier. Проба seed19 показывает исходную цель внутри зоны соперника: route_cells=0 и при текущей, и при предсказанной позиции, ближайшая frontier тоже недоступна. Гипотеза расхождения текущей/предсказанной позиции этим снимком не подтверждена. Старый probe не ограничивал world ареной; найденная им альтернатива за пределами арены не является доказательством пригодного маршрута. Probe исправлен: читает resolution/robot_radius/arena_bounds у реального planner, сохраняет известные стены и отдельно сравнивает fresh departure с текущим/предсказанным треком; retained waypoint/cache/watchdog не воспроизводит.

В кандидате frontier_candidates ранжирует альтернативы, reachable_frontier_route пропускает запрещённые конечные точки и пробует до30 маршрутов по прежним правилам A*. Только после неуспеха допускается recovery. A* быстро отклоняет явно запрещённую цель при старте снаружи зоны, сохраняя возможность выхода из зоны для старта внутри. Завершённый частичный путь больше не закрепляет робота у промежуточной точки: запрашивается новый путь к исходной цели. Пределы MPPI, native critics, контур и swept-check не менялись.126 тестов прошли, включая перекрытую цель/недостижимую первую альтернативу/завершённый частичный маршрут; Compose config, compileall, diff check прошли. Следующий шаг — сборка и повтор scenario3 seeds18–20, затем регрессия scenario1/2; измерить задержку fallback и не считать исправление успешным до реальных результатов. Финальные20 и порог0,3 не выполнены.

**Основной MPPI: сборка и scenario1 подтверждены.** `2da0c37`, `results/series-20261001T054837Z/`; запуск без внешнего выбора backend: `env -u HSL_LOCAL_BACKEND python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace --build`. Runner разрешил auto в nav2_cpp, оба native активировались; runtime SHA256 и эффективные параметры сохранены. Seed12 цель14,4с,13 поимка12,2с,14 цель20,7с. Explorer/guardian скорости0,301/0,361;0,211/0,326;0,261/0,379м/с. Средние0,258/0,355, минимумы0,211/0,326; ниже0,2 —0/3 у обеих ролей, ниже0,3 —2/3 explorer и0/3 guardian. Контактов0. RMS до global0,048/0,052м, angular accel RMS0,826/0,644рад/с², stop-go1,0/0. Recovery explorer seed13 составляет0,131 активного времени, NO_LOCAL_PATH0,016; остальные explorer без recovery. Поэтому утверждение об отсутствии всех recovery остаётся неверным. После последнего исхода audit подтвердил allowed=false, единственного издателя и10 нулевых команд у обоих. Первый старт Gazebo потребовал повторной инициализации до движения; оценочных технических сбоев0. Документы AGENTS/PROJECT_GOAL синхронизированы с основным официальным C++ MPPI. Следующая неизменная проверка: scenario3 seeds18–20 до90с; затем разбор recovery seed13 и порога0,3. Полная серия20 и устойчивость новых препятствий/GUI ещё не подтверждены.

**Встреча/recovery, оригинальный C++ MPPI: все шесть скоростей ≥0,2.** `6c67d85` (навигационные исходники104403f, соответствие образу проверено SHA256), `results/series-20261001T053734Z/`; команда `HSL_LOCAL_BACKEND=nav2_cpp python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 90 --scenario 2 --trace`. Seed15 цель14,1с,16 поимка13,4с,17 цель13,6с.0 контактов. Explorer/guardian:0,281/0,269;0,208/0,266;0,289/0,254м/с. Средние0,259/0,263, минимумы0,208/0,254; ниже0,2 —0/3,0/3. RMS global0,060/0,046м, angular accel0,728/0,701рад/с², stop-go1,3/0. Recovery0 у всех, planner OK explorer1,0/0,977/0,999. Native compute p9524,59–28,02мс, global p95105,36–145,70мс. Сравнение Python `052919Z`: скорости0,232/0,281 (один explorer<0,2), RMS0,084/0,077, angular accel1,682/1,489, stop-go4/3. Исходы и продолжительности разные; это сравнение одинаковых seed/стартов, не тождественных траекторий. `02-gate-after-finish.json` подтвердил оба final cmd_vel: один издатель, ≥10 нулевых команд, allowed=false. Утверждение об устранении всех recovery/ошибок встречи пока ограничено этими3 матчами, не всем полигоном.

**Основной backend переключён на оригинальный MPPI с резервами.** Перед изменением сохранён local tag `checkpoint/native-encounter-20261001` на6c67d85. Новый auto в launch/Compose/runner выбирает nav2_cpp при mppi, python при stock mpc; явный HSL_LOCAL_BACKEND=python доступен. ROS-интерфейсы/скоростные параметры/судейство не менялись. Полный перенос decision на C++ не сделан: его p95 меньше1мс; локальная тяжёлая часть уже официальный C++. Остаточный global planner требует профилирования, а не переписывания без замера. Следующая проверка нового default: `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace --build`, затем scenario3 с заранее выбранными seeds18–20. Финальная серия20 и порог0,3 не выполнены, цель остаётся активной.

**Встреча/recovery, Python: оба исхода подтверждены, скорость одного explorer недостаточна.** `104403f`, `results/series-20261001T052919Z/`, команда `HSL_LOCAL_BACKEND=python python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 90 --scenario 2 --trace --build`. Seed15 поимка26,0с,16 поимка5,1с,17 цель (точное время в outcome/index). Контакты0. Скорости explorer/guardian:0,180/0,215;0,245/0,376;0,271/0,251; средние0,232/0,281, минимум0,180/0,215; ниже0,2 —1/3,0/3. RMS global0,084/0,077м, angular accel1,682/1,489рад/с², stop-go4,0/3,0. Seed15 EVADE средняя по pose-отсчётам0,093м/с, recovery explorer0,642; задача не завершена. Это не доказывает устранение всех жалоб в свободном пространстве или устойчивость всех сценариев. Исправления направления/точной поимки сохранены для повторной проверки native; коллизии не маскировались.

Добавлен read-only `benchmarks/audit_motion_gate.py`: одновременно читает allowed и минимум10 команд обоих финальных потоков, проверяет единственного издателя и нулевую конечную команду при запрещённом движении. `00-gate-before-start.json` подтвердил обе проверки до первого старта; `02-gate-after-finish.json` снимается после последнего исхода.121 тест прошёл. Следующий запуск на том же образе/исходниках — native scenario2 seeds15–17 до90с сtrace без rebuild; сравнить по каждому роботу, не только агрегат.

**Настройка направления guardian не завершила задачу.** `1136ef4`, `results/series-20261001T002644Z/`, команда `HSL_LOCAL_BACKEND=nav2_cpp python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 60 --scenario 2 --trace --build`.3 цели explorer,0 поимок,0 контактов. Explorer/guardian seed15–17:0,252/0,195;0,130/0,160;0,253/0,313м/с; средние0,212/0,223, ниже0,2 —1/3,2/3. RMS global0,068/0,049, angular accel0,555/0,637, stop-go2,0/2,3. Critic направления сам по себе не исправил поимку; native остаётся экспериментальным.

**01.10, сообщения пользователя и новый цикл встречи/recovery.** Пользователь проверил `helm start_match*`: выросла скорость, но recovery иногда уводит при свободном пространстве; guardian объезжает вместо поимки, explorer сближается вместо отхода. `results/duel-20261001T051123112369Z.json` имеет run_id manual, seed0, revision unknown: explorer цель29,7с, скорость0,232, контакты0, recovery0,421, stop-go10, angular accel1,818; decision p95 0,7мс, planner p95 248,32мс. Native timing отсутствует, RECOVERY_FALLBACK присутствует — это Python-контур. Точная версия этого ручного образа не подтверждена runtime snapshot.

Исправления в текущем кандидате: recovery/avoid/evade-waypoint отменяются при смене intent.behavior; старые выходы переоцениваются по свежему world и сопернику в обоих backend. Watchdog учитывает фактическое вращение и уменьшение ошибки к глобальному маршруту, ограничивает эту отсрочку8с после поступательного прогресса; первый срыв повторяет номинальный маршрут/warm start, второй выбирает выход. Explorer EVADE при близком наблюдаемом сопернике выбирает проверенный короткий отход с неотрицательной проекцией от угрозы, сохраняет его до достижения/опасности и затем возвращается к основной площадке. Capture использует точный радиальный подход0,39м, проверенные альтернативы только при препятствии, а внутри0,36–0,45м — ориентацию без перемещения; точный endpoint не заменяется клеткой A*. Python CAPTURE_ALIGNMENT отдаёт проверенный поворот штатному шлюзу, вместо нулевого прямого MPPI при вырожденном пути. Коллизии/свежесть/диапазоны скоростей сохранены.121 тест прошёл (добавлены отход в свободном месте/у стены, радиальный подход, ориентация внутри радиуса), compileall/diff clean. Эффект дуэлью пока не подтверждён.

Перенос на C++: официальный локальный MPPI уже C++. Decision в ручном отчёте занимает p95 0,7мс, поэтому полный перенос ради скорости не обоснован; Python planner p95 248мс требует дальнейшего профилирования и при необходимости переноса горячей части A*/сборки мира после стабилизации семантики. Следующая проверка — Python default scenario2 seeds15–17 до90с сtrace/build, затем native на том же исправлении. Не объявлять завершение без повторных обоих исходов и индивидуальных порогов.

**Задний ход native дал повторные цели; страж требует исправления ориентации.** `4a06f03`, `results/series-20261001T001350Z/`, команда `HSL_LOCAL_BACKEND=nav2_cpp python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 60 --scenario 2 --trace --build`. Роли чередовались. Seed15/16/17 — цели за15,6/35,2/15,4с, поимки0, контакты0. Скорости explorer/guardian:0,254/0,278;0,244/0,134;0,256/0,343м/с. Explorer среднее/минимум0,251/0,244, ниже0,2 —0/3; guardian0,252/0,134, ниже0,2 —1/3. RMS до global в точном referee-окне0,080/0,058м, RMS углового ускорения0,698/0,603рад/с²; stop-go1,3/1,0. Сравнение со стенами/forward-only `000256Z`: там3 поимки, скорости0,233/0,277 и ниже0,2 —1/3,0/3; прежний RMS не синхронизирован, напрямую не сравнивать. Трассы `001350Z` содержат абсолютное время, физические роли и выходы команд. Explorer выбирал reverse40/309,66/677,22/173 активных pose-отсчётов (это доли отсчётов, не доли времени). Guardian seed16 reverse512/622 при visibility0,486 и скорости0,134; native не требует направления корпуса к сопернику в PathAngle, поэтому мог следовать за ним кормой и не выполнить геометрию поимки. Остальные guardian reverse0. Графики `motion.png`/`motion.svg` создаются `python3 results/series-20261001T001350Z/plot_motion.py`; изображение проверено, после серии перегенерировано на3 матчах.117 тестов прошли. Скорость0,3 ещё не подтверждена; финальной серии20 нет.

**Следующий кандидат: предпочтение направления пути по роли.** В autonomous launch официальный `MPPI.PathAngleCritic.forward_preference` теперь true для guardian, false для explorer. Оба по-прежнему имеют vx_min=-0,35/vx_max=0,5, PreferForwardCritic5 и полный swept-check. Это настраивает штатный critic согласно обязательному направлению при поимке, без изменения модели/скоростного диапазона или поддавков. До дуэли улучшение стража не доказано. Следующая команда: native scenario2 seeds15–17 до60с с trace/build; затем проверять scenarios1/3 и новые seeds, если сохранятся обе роли и скорость каждого ≥0,2.

**Известные стены сохранены; одна эта правка не спасает исследователя.** `cadde2d`, `results/series-20261001T000256Z/`, команда `HSL_LOCAL_BACKEND=nav2_cpp python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 60 --scenario 2 --trace --build`. Все3 заезда — поимки, цели0, контакты0. Скорости explorer/guardian seed15–17:0,266/0,287;0,167/0,271;0,267/0,274м/с; средние0,233/0,277, ниже0,2 —1/3 и0/3. RMS углового ускорения1,137/0,667, stop-go0,3/0. Устранение удаления grid_points оставлено как семантическое исправление карты; улучшение тактики не доказано. Тесты115 прошли, Compose config прошёл. Explorer EVADE в трассах часто имеет глобальное направление более2рад от корпуса, а native vx_min0 не допускает немедленный отход назад.

**Кандидат: штатный задний ход Nav2; метрика боковой ошибки синхронизирована.** YAML native vx_min=-0,35, PathAngleCritic.forward_preference=false, штатный PreferForwardCritic5 оставлен. Ошибочное неиспользуемое поле mode удалено; версия1.1.20 читает bool forward_preference. Это единственное изменение движения следующего опыта, предел вперёд0,5/угловой1,5 не меняются, весь swept-check сохранён. Trace теперь пишет абсолютное sim_t_s, report_motion использует точное referee-окно каждого робота. Старые RMS выше вычислены по полной разрешённой трассе, которая могла начаться до referee и закончиться после него; это приближённые диагностические числа, не строго синхронизированная оценка. Старые трассы без абсолютных timestamps теперь не выдаются за оценку активного окна. Следующая серия: native scenario2 seeds15–17 до60с с trace/build; сравнить исходы, каждый порог скорости, коллизии и плавность. Python default и MPC резерв сохранены.

**Согласованный контур native: серия завершена, исследователь слабее.** `e49c7e7`, `results/series-20260930T234955Z/`; команда `HSL_LOCAL_BACKEND=nav2_cpp python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 60 --scenario 2 --trace --build`. Роли чередовались между физическими стартами. Seed15/16/17: поимки за5,9/6,8/5,1с, цели0, контакты0. Скорости explorer/guardian:0,264/0,271;0,175/0,274;0,308/0,288м/с. Средние0,249/0,278, минимум0,175/0,271; ниже0,2 —1/3 и0/3. Средний по матчам RMS до глобального пути0,034/0,043м, RMS углового ускорения1,289/0,706рад/с²; stop-go0 у всех. Это короткие матчи, не доказательство длинной уверенной навигации. Прежняя запись «контур ещё без проверки» ниже теперь историческая. Контур оставлен экспериментальным: скорости улучшены, но навигационных успехов исследователя нет; Python остаётся default, исходный MPC сохранён.

**Следующий цикл: сохранить известные стены возле соперника.** В node при сборке `VoxelWorld` фильтр радиусом0,45м вокруг наблюдаемого соперника удалял также `grid_points`, то есть стены статической карты. Native costmap сохраняет эти стены, поэтому A* мог предлагать невыполнимую ссылку. Вынесена сборка препятствий: фильтруются только наблюдаемые облака, статические grid_points сохраняются всегда. Регрессионная проверка подтверждает, что переход через известную стену рядом с треком запрещён, а корпус соперника удалён из облака. Эффект на дуэль пока не проверен; следующая серия — те же scenario2, seeds15–17 с native и rebuild. Скорости/critics/decision не меняются, чтобы отделить эффект карты.

**Clock100 подтверждён; сам по себе застревание не устранил.** `797377b`, `results/series-20260930T233741Z/`, команда `HSL_LOCAL_BACKEND=nav2_cpp python3 benchmarks/run_duel_series.py --runs 2 --start-seed 15 --active-s 60 --scenario 2 --trace --build`. `/gazebo.publish_rate=100` сохранён runtime; пассивный `clock-observed.json` дал239 положительных интервалов, median/p95dt.01с, actual100Гц. Все четыре отчёта имеют1200 native/control циклов на60с, то есть20Гц. Seed15/16 оба timeout60с, explorer/guardian .018/.007 и .077/.073м/с, контакты0; цели/поимки0. Скорости неприемлемы. Исправление часов оставлено как согласование фактической частоты сmodel_dt.05, а не как доказанное улучшение поведения. Native по-прежнему экспериментальный; Python default наclock100 ещё требует сопоставимой проверки.

**Кандидат: согласовать collision envelope A* и native (ещё без проверки дуэлью).** Xacro cylinder collision radius.178м, остальные круговые пластины.170м; planner.robot_radius.23м. Native ошибочно использовал.37/.35м как твёрдый корпус (.23 плюс прежний tracking margin.14/.12), поэтому граф A* и costmap имели разные требования к проходимости. В C++ удалён role override, radius читается из costmap.robot_radius и проверяется на finite/positive; YAML задаёт.23м для обоих идентичных роботов. Это сохраняет.052м относительно физического корпуса, тот же32-угольник вокруг защитного диска, весь swept-check, интерполяцию между точками и штатный ObstaclesCritic proximity/collision_margin_distance.10. Скоростные пределы и модель Nav2 не менялись. Диагностический рисунок `results/series-20260930T232724Z/footprint-diagnosis.png` показывает различие контуров на реальном snapshot; не является доказательством успешной езды. Следующий запуск — scenario2 seed15–17 до60с сtrace/build, сравнить каждую роль/коллизии/отклонение/ускорения.


**Native recovery исправлен, порог скорости ещё не выполнен.** `7d4edd8`, `results/series-20260930T232724Z/`, команда `HSL_LOCAL_BACKEND=nav2_cpp python3 benchmarks/run_duel_series.py --runs 2 --start-seed 15 --active-s 60 --scenario 2 --trace --build`. Seed15 timeout60с explorer/guardian .008/.074; seed16 поимка30,2с .078/.093; контакты0. Seed15 explorer386 swept_collision/225ok против534/77 диагностической базы, seed16 explorer37swept/263ok против585/10. Цели explorer всё ещё нет, все четыре скорости ниже .2. Семантические исправления recovery оставлены, но улучшения до критерия недостаточно. Новый snapshot `00-native-failure-snapshot.json` подтверждает конечный recovery yaw≈π/2 и текущий отказ на индексе52; это будущая часть rollout, не footprint текущей позы.

**Выявлено несоответствие симуляционных часов (следующий кандидат).** `ros2 param dump /gazebo --no-daemon` в живом мире дал publish_rate10Гц. Native имеет controller_frequency20Гц/model_dt.05с, gate timer.05с; метрики содержат600 native/control циклов за60с, то есть10Гц фактически. Изменена публикация clock на100Гц через новый `sim_kobuki/config/gazebo_duel.yaml`, переданный params_file в gzserver только duel. Физика/скорости/collision-check не менялись. Runtime теперь сохраняет `/gazebo`. Нужны rebuild, проверка параметра100, фактических периодов/числа циклов и повтор seed15–16 scenario2 до60с. Прежде чем отдельно менять warm-start или footprint, проверить это согласование времени.


**Native диагностика: основная остановка в swept-check, найдены ошибки recovery-ссылки.** `HSL_LOCAL_BACKEND=nav2_cpp python3 benchmarks/run_duel_series.py --runs 2 --start-seed 15 --active-s 60 --scenario 2 --trace --build`, `results/series-20260930T231538Z/`, исходник диагностической версии соответствует `97b6cc1` (runner начал до коммита и записал dirty fingerprint). Seed15/16: два timeout60с, цели/поимки0, контакты0; explorer/guardian .008/.045 и .005/.009м/с. Trace seed15 explorer:534 swept_collision /77 ok; seed16 explorer585 swept_collision /10ok, guardian370 swept_collision /238ok /4optimizer_failure. То есть отказ от дополнительной полной проверки, а не авария оптимизатора, доминирует. Timing теперь полон:600 измерений на60с у каждого, p95 native21.56–24.8мс, max≤40.89. Отсутствие контактов здесь сопровождается застреванием, кандидат неприемлем.

Сохранён пассивный снимок `00-native-failure-snapshot.json`: pose(-.175,.412), rejected point(-.111,.477), стоимость254,index28. Recovery target(-.175,.962) имеет0.178м до текущей LiDAR-точки, но0.485м до статической карты: новое препятствие может быть соперником, его природа снимком не доказана. Выбор recovery раньше использовал world до свежего update и не проверял сохранённую цель повторно. Кроме того Pose2 escape имел yaw0; официальный GoalAngleCritic интерпретирует это как обязательную конечную ориентацию.

**Кандидат: корректные recovery-ссылки для native (ещё без дуэли).** Выбор native escape отложен до обновления карты/скана, предыдущая цель каждый tick проверяется safe_segment с наблюдаемым enemy/clearance; при отказе выбирается безопасная альтернатива. Yaw recovery направлен вдоль выхода, конечный yaw обычного глобального reference — касательная последнего сегмента, при capture сохранён yaw на соперника. Существующий Python backend сохраняет прежнюю ветвь. Collision-check и скоростные параметры не ослаблены. Два теста появления нового препятствия/соперника и ориентации выхода, всего113passed; compileall/diff clean. Следующий эксперимент — те же scenario2 seed15–16 до60с после rebuild.


**Native MPPI запущен; первый кандидат хуже Python.** Исправленный явный footprint успешно собран и проверен `HSL_LOCAL_BACKEND=nav2_cpp python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 90 --scenario 2 --trace`, `results/series-20260930T230724Z/`, версия `1dacd5e+dirty.68262498aa0e`. Все три матча — реальные поимки без контактов, цели исследователя нет. Seed15: 13,0с, explorer/guardian .035/.218м/с; seed16 (перестановка физических ролей): 39,3с, explorer/guardian .006/.134; seed17:16,9с,.025/.173. Средние .022/.175, ниже .2:3/3 и2/3. RMS глобального отклонения по матчам explorer .194/.065/.180м, guardian .068/.165/.055м; угловое ускорение RMS explorer1.652/.606/.579, guardian2.969/1.122/1.586. У исследователя seed16 NO_LOCAL_PATH97,2% активного времени, скорости <.05 во всех трёх — дефект застревания. Runtime перед каждым стартом подтвердил hashes/15 узлов/пакеты Nav2; общее окно referee и coverage1.0 подтверждены. Вживую topic info показал по одному издателю каждого cmd_vel (gate) и mppi_cmd_vel (native) в обоих namespace. Нав21.1.20. Этот кандидат не принят как default; Python остаётся основным.

**Следующая диагностика native (ещё не прогнана).** NO_LOCAL_PATH раньше смешивал исключение optimizer и отклонение полной оптимизированной траектории swept-check. Добавлены JSON причины optimizer_failure/swept_collision и координата/стоимость/индекс отказавшего footprint, запись в trace. Timing теперь охватывает все ранние возвраты, поэтому прежние native p95 не представляют стоимость всех неуспешных циклов. Поведение и collision-check этой диагностикой не ослаблены. Гипотеза для последующей проверки: watchdog исследователя прерывает полезное выравнивание через4с, а долгие отказы seed16 могут возникать в swept-check; не менять коэффициенты до выяснения причины. Следующий шаг — rebuild и диагностические seed15–16 scenario2.


**Первый native запуск — технический отказ до матча.** Интеграция сохранена `1dacd5e`, актуальный образ успешно пересобран runner. Команда `HSL_LOCAL_BACKEND=nav2_cpp python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 90 --scenario 2 --trace --build`, каталог `results/series-20260930T230214Z/`. Оба MPPI завершились при configure: `Considering footprint in collision checking but no robot footprint provided in the costmap`. Сохранены startup logs обоих planner; движения/исхода native не было. Причина: официальный critic с consider_footprint=true не принимает один robot_radius. Добавлен явный 32-угольник, описанный вокруг прежнего защитного диска .37/.35 м; collision-check не отключён. Эта правка ещё требует rebuild и повторного запуска. Не считать каталог завершённой серией.


**Интеграция native MPPI в процессе.** Добавлен экспериментальный `hsl_nav2_control` и запуск `HSL_LOCAL_BACKEND=nav2_cpp` через новый autonomous launch. Default Python и checkpoint `67ecd8c` сохранены. В native-режиме A* публикует reference/global status, официальный MPPI — локальную траекторию и direct command; gate не переопределяет его чистый поворот. Runner дополнен готовностью native и параметрами двух costmap, метрики — отдельным compute timing. `111 passed`, Python compileall прошёл. `helm build duel` завершился успешно после исправления чтения параметров Humble (`list_parameters`). После начала этой сборки добавлены проверки lifecycle и устранён idle publisher diagnostics в Python native-режиме, поэтому для испытаний нужна повторная сборка текущего дерева; заезда native пока нет. Compose config и повторные 111 тестов прошли. Следующий шаг: исправить ошибки сборки/ROS запуска, проверить sole publishers и нулевое движение до старта, затем серия scenario2 seed15–17 до90с.


## Основной режим MPPI с резервным MPC (30.09, проверка после переключения)

**Жёсткий margin в A* отклонён.** Повтор `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 90 --scenario 2 --trace --build`, `results/series-20260930T222039Z/`: 1 цель / 2 поимки / 0 контактов. Скорости explorer/guardian seed 15/16/17: 0,162/0,205; 0,202/0,183; 0,167/0,206 м/с, средние 0,177/0,198; ниже 0,2 — 2/3 и 1/3. RMS отклонения 0,089/0,067 м; углового ускорения 1,759/1,569; stop-go 11,3/9,0; recovery 0,677/0,384. p95 planner исследователя 342/440/191 мс, стража 186/191/175. Перепривязка reuse уже исправлена, но по сравнению с базой `215201Z` скорость и stop-go хуже; жёсткая проверка каждого global-звена не устранила recovery. Core/node/два экспериментальных теста восстановлены из `845a3bf`, `111 passed` и diff clean. Текущий образ пока содержит отклонённый A*; перед дальнейшей дуэлью его пересобрать. Процесс серии завершён. Следующее направление — эксперимент с официальным C++ MPPI Nav2, сохранив Python MPPI и исходный MPC как восстановимые варианты; пока это подготовка, подтверждённого выигрыша нет.

**Воспроизводимость runtime.** Runner теперь перед стартом сохраняет `NN-runtime.json`: ID контейнеров/образов, сценарные стартовые параметры, ROS-параметры двух planner/decision/CC/Lat/gate и referee. SHA256 исходников навигации, карты/мира и MPC сравниваются с файлами обоих planning-контейнеров; несовпадение останавливает серию с требованием rebuild. После остановки `222039Z` новая функция проверила 90 файлов и 11 узлов; результат `runtime-verified-after-series.json` относится только к последнему матчу seed 17 и отклонённому образу, не ко всей серии. Во время той серии функция ещё не вызывалась самим runner. Нужен следующий полный запуск для проверки нового pre-start пути. При чтении отчётов различать заявленные ROS-параметры и внутренние производные значения модели, описанные в `NAV2_MPPI_ADAPTATION.md`.

**A*/local margin: первая серия выявила регрессии и ошибку сравнения.** `results/series-20260930T221019Z/`, `845a3bf+dirty.8798e58ffef5`: 2 цели / 1 поимка / 0 контактов. По seed 15/16/17 скорости explorer/guardian 0,191/0,228; 0,212/0,254; 0,183/0,216 м/с, средние 0,195/0,233, ниже 0,2 — 2/3 и 0/3. RMS углового ускорения 1,676/1,449; stop-go 7,3/4,7; recovery 0,586/0,208. p95 planner у исследователя 218/224/460 мс, max до 569; у стража 214/197/201, max до 573. Новое окно referee и учёт coverage/recovery-поворотов проверены этой серией. Заявлять выигрыш в отклонении нельзя: RMS 0,032/0,036 м против прежних 0,089/0,081 смешивает реальный эффект и перепривязку начала reuse к собственной позе на каждом цикле. Перепривязка reuse убрана; безопасность ближайшего перехода проверяется от own, но публикуются исходные точки выбранного глобального коридора. Свежий A* по-прежнему начинает из фактической позы. Также добавлен ранний отказ, если целевая raster-точка не проходит margin: такой target всё равно недостижим в этом графе, незачем обходить тысячи клеток. `113 passed`, compileall/diff clean. Кандидат **не принят**, требуется повтор на seeds 15–17 после этих исправлений и при необходимости откат/иная реализация согласования. Значения прошлых индивидуальных скоростей и общий активный интервал сохранены в paired JSON.

**Проба согласования A*/local margin (идёт Gazebo).** Фактический global Path стража в probe `220022Z` начинается западным участком, который реконструированная текущая карта отмечает как `segment_safe=false`, при том что north/east безопасны. A* раньше проверял occupied/map границы, а запас до стен только мягко штрафовал; MPPI требует swept clearance. Введён необязательный `safety_margin` в A* и reuse; основной узел передаёт прежний margin роли без его уменьшения. Проверяются звенья, начальная точка нового маршрута — фактическая поза, reuse снова проверяет ближайшие 15 точек от текущей позы. Сильно удалённые от стен короткие звенья проходят по нижней оценке clearance, у границ используется `safe_segment`. Тестовый обход стены: прежний маршрут имеет minimum clearance 0,300 м и небезопасные звенья при требовании 0,340; новый 0,361 и все звенья safe, 21 точка в обоих. Время синтетического A* ~17 мс прежде/~41 мс после (одно локальное измерение, не runtime-статистика). Проверен выход источника, уже находящегося у стены, и отказ reuse прежнего небезопасного пути. `113 passed`, diff clean. Запущено `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 90 --scenario 2 --trace --build`, `results/series-20260930T221019Z/`; пока не считать алгоритмическим улучшением. Сравнить outcomes, скорость, RMS отклонения, recovery и p95/max времени planner; особое внимание недоступным target и новым `NO_GLOBAL_PATH`.

**Общее окно подтверждено дуэлью.** `results/series-20260930T220022Z/`, `e006fdf+dirty.a2497f0abc63`: цель за 27,6 с, скорости 0,220/0,183 м/с, 0 контактов. Обе метрики имеют `window_source=referee`, start=632,777/end=660,377 sim s, duration=27,6 с и покрытие 1,0; runner проверил равенство. Диагностические probe валидны и содержат фактические цели: у исследователя (-0,089; 0,229) при собственном курсе 1,688 и просвете ~0,324 м; 73 безопасных образца, но selected/furthest progress=0, первая команда v=0, ω=-1,272. У стража цель (0,900; 1,860), просвет ~0,334, v=0,302. Реконструированный probe для стража показывает ближайшего западного соседа по фактическому global Path как `segment_safe=false`, хотя он free/unblocked; безопасны северный и восточный. Следующая гипотеза — A* допускает клетки с меньшим запасом, чем требует local MPPI, что порождает невыполнимые первые звенья и recovery. После живого замера исправлены ещё два пункта метрики: покрытие теперь сумма реально обработанных интервалов ≤1 с (разрывы не скрываются общим span), повороты `RECOVERY_MPPI` учитываются как активность при независимом `planner_ok=0`. Дополнительный тест прошёл (`111 passed`); эти две арифметические правки требуют следующего runtime-образа, обычное общее окно уже подтверждено.

**Единое окно метрик — реализовано, живая проверка идёт.** Referee добавляет `started_at_sim_s`/`finished_at_sim_s` в outcome. Наблюдатели временно собирают данные после своего разрешения, затем по outcome пересчитывают только общий интервал: граничные перемещения интерполируются между соседними отсчётами с разрывом не более 1 с, контакты и вычислительные/потоковые задержки вне окна исключаются. После обычного ручного stop без outcome сохраняется прежний замер с `window_source=allow_motion`; runner принимает только `window_source=referee` с идентичными границами обеих метрик и outcome. В отчёте есть `sample_coverage_fraction`, чтобы пропуски на границах были видны. Тест с ранним запуском, поздним stop, контактами/таймингами вне окна прошёл: 0,6 м за 2 с → 0,3 м/с, один контакт внутри окна. Всего `110 passed`, `compileall`, `git diff --check`. Запущено `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 15 --active-s 90 --scenario 2 --trace --probe-status RECOVERY_MPPI --build`; до живой проверки новые метрики не считать подтверждёнными. `turning_fraction` пока включает поворот только при `OK`, поэтому полезные повороты recovery нужно отдельно пересмотреть, сохранив независимый учёт recovery/обычной доступности.

**Направленное recovery отклонено; уточнена граница доказательства probe.** Контроль без `preferred`: `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 90 --scenario 2 --trace`, `results/series-20260930T215201Z/`, образ `eea3b713…`, 1 цель / 2 поимки / 0 контактов. Скорости explorer/guardian по seed 15,16,17: 0,229/0,271; 0,243/0,235; 0,138/0,167 м/с. Сравнение с направленным вариантом `182056Z`: средние скорости 0,203/0,224 против 0,227/0,237; среднее RMS отклонения от глобального маршрута по trace 0,089/0,081 против 0,078/0,060 м; RMS углового ускорения 1,864/1,422 против 1,902/1,337; stop-go 5,3/3,3 против 8,0/5,3; временная доля recovery 0,341/0,134 против 0,378/0,193. Направленный вариант увеличил скорость и уменьшил отклонение, но ухудшил stop-go и долю recovery, страж не поймал ни разу; устойчивое устранение recovery не подтверждено. Эвристика и её тест удалены, базовый выбор выхода восстановлен. Все эти скорости ещё имеют прежнее окно измерения. Дополнительно исправлена интерпретация снимка: `recovery_step` в probe **пересчитывается**, это не опубликованная recovery-цель узла; прежнее утверждение, что снимок доказал именно выбранный уход на юг, слишком сильное. Диагностика теперь публикует фактическую `recovery_goal`, probe сохраняет полученные global/local Path и mppi JSON отдельно от реконструированного маршрута. Эти новые диагностические поля ещё требуют живой проверки. Следующий цикл — единое окно метрик referee, затем анализ фактической причины/цели recovery.

**Направленный recovery: серия завершена, решение ещё не принято.** `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 90 --scenario 2 --trace --build`, `results/series-20260930T182056Z/`, ревизия `f8fab44+dirty.94fb46eff0f3`: 3 цели / 0 поимок / 0 контактов. По ролям explorer/guardian скорости: seed 15 — 0,187/0,238; seed 16 (переставленные роли) — 0,289/0,223; seed 17 — 0,206/0,251 м/с. Средние 0,227/0,237; RMS углового ускорения 1,902/1,337 рад/с²; stop-go 8,0/5,3. У исследователя 1/3 ниже 0,2, у стража 0/3, но нет поимок. Предыдущая серия `series-20260930T170042Z/` дала 2 цели / 1 поимку, 0,211/0,213 м/с, 1,841/1,500 рад/с², stop-go 7,7/6,7; её код также отличается диагностикой, поэтому причинность нового recovery ещё не доказана. Только новый аргумент `preferred` временно отключён в вызове node для контрольной серии. Первый контроль `series-20260930T182946Z/` технически неполон: 0 полных матчей, ожидание своего outcome истекло, текущие контейнеры записали `run_id=manual`; его не учитывать. Процесса runner больше нет. Контроль повторён без сборки как `series-20260930T215201Z/`; SHA256 node/core в работающем контейнере совпали с исходниками, образ `sha256:eea3b713b26c74e7de9b2684612687f0ee166b104d389565856c1dc402192040`. После контроля вернуть либо удалить новую ветку по данным.

**Открыт дефект общего окна метрик.** `MatchMetrics.on_allowed()` начинает замер отдельно после разрешения каждого робота, referee начинает после разрешения обоих. В серии выше длительность первого наблюдателя 34,4 с при outcome 33,2 с. Нынешний знаменатель включает задержку последовательного запуска; значения порога скорости пока не являются строгой проверкой общего активного окна. После сравнения recovery требуется единый интервал referee и проверка обеих метрик до первого исхода.

**Снимок recovery и проба направленного выхода (идёт сравнение).** После исправления импорта `planner_probe.py` повторён scenario 2, seed 15, 90 с: `results/series-20260930T181550Z/`, цель исследователя за 30,2 с, средние скорости 0,189/0,242 м/с, 0 контактов, доля обычного `OK` 0,635/0,693. Оба файла `00-probe-*.json` теперь валидны. В первом снимке исследователь у (0,599; 0,074), глобальный маршрут в ближайших клетках ведёт на восток, соседние восточные клетки и прямые к ним безопасны, однако `recovery_step` выбрал юг (-π/2, 0,55 м) из-за большего просвета до препятствий. Это объясняет конкретный бесполезный разворот, но снимок асинхронен относительно цикла планировщика и один случай не доказывает общую причину замедления. В `recovery_step` добавлен необязательный ориентир из ближайшего участка глобального пути, который учитывается только среди прошедших прежнюю проверку `safe_segment`; при отказе локального MPPI он передаётся до очистки глобального пути. При отсутствии маршрута и при застревании прежний способ выбора сохраняется. `planner_probe.py` теперь показывает оба варианта, добавлен тест безопасности и направления; `110 passed`, `compileall`, `git diff --check`. Сравнительный запуск scenario 2, seeds 15–17 (`results/series-20260930T182056Z/`) ещё идёт. После него сравнить исходы, скорости, отклонение, плавность, контакты и долю recovery с принятым `3fecb97`; оставить изменение только при подтверждённом выигрыше.

**Снимок recovery пока не получен — исправлен инструмент.** Запуск `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 15 --active-s 90 --scenario 2 --trace --probe-status RECOVERY_MPPI`, `results/series-20260930T181337Z/`: поимка за 14,5 с, скорость 0,086/0,240 м/с, контактов 0, `planner_ok_fraction` исследователя 0,519. Оба `*-probe-*.json` содержали traceback: старый `benchmarks/planner_probe.py` импортировал удалённый `recovery_heading`. Этот запуск **не дал снимка карты**; probe исправлен на существующий `recovery_step`, компилирование/повторный живой запуск ещё нужны. Реальная дуэль, отчёты и trace при этом завершились, ошибки инструмента не меняли управление роботом.

**Живая проверка статуса recovery.** `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 15 --active-s 90 --scenario 2 --trace --build`, `results/series-20260930T180601Z/`, ревизия `f8fab44+dirty.44d70aab0dc9`: поимка за 19,2 с, скорости 0,139/0,220 м/с, контактов 0. Это короткий диагностический матч, не сравнимая статистика качества с тремя предыдущими. У исследователя 48 отсчётов `RECOVERY_MPPI` со средней фактической/финальной/прямой MPPI скоростью 0,142/0,130/0,126 м/с; значит шлюз действительно пропускает команду recovery MPPI. В этом статусе среднее число валидных образцов 55 из 128, средний просвет центра до препятствия 0,422 м при требуемом радиусе с запасом 0,37 м. `RECOVERY_FALLBACK` занял ещё 24 отсчёта, средняя скорость 0,070; `RECOVERY_ESCAPE` — 4. Обычный `OK` — 126 отсчётов из 205, средняя скорость 0,145; новая `planner_ok_fraction` исследователя 0,620, тогда как прежняя метрика включала длительную recovery-дугу. Число безопасных образцов на дуге ненулевое: оптимизатор располагает вариантами, но выбирает медленное первое управление рядом со стеной; дальнейший разбор требует снимка локальной карты/цели recovery. После исхода оба `/cmd_vel` имеют linear.x=0 и angular.z=0 по `ros2 topic echo --once`. Тесты/Compose/сборка прошли; новый статус, оба namespaces и остановка подтверждены одним живым матчем. Следующий шаг — снять `planner_probe.py` при первом `RECOVERY_MPPI`, оценить реально безопасные направления выхода и затем исправить выбор recovery-цели либо cost дуги, не снижая запас до стены.

**Обнаружено смешение recovery с обычным `OK` (результат исправления выше).** Диагностическая серия `results/series-20260930T175644Z/` после `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 15 --active-s 90 --scenario 2 --trace --build`: цель за 27,6 с, скорости исследователь/страж 0,218/0,262, контактов 0. В районе 5,1–6,3 с планировщик выдавал `RECOVERY_ESCAPE`/`RECOVERY_FALLBACK`, с 6,3 до ~9,3 с статус был `OK`, но диагностический топик обычного MPPI замолчал при продолжающихся MPPI-командах. Причина найдена в коде: `recovery_path()` вызывает MPPI отдельно и устанавливает `direct_controls`; финальная публикация считала любые `direct_controls` статусом `OK`. Поэтому прежние `planner_ok_fraction` завышены, а медленный разворот на recovery-дуге ошибочно приписывался нормальному локальному планированию. Сейчас recovery-дуга получает отдельный статус `RECOVERY_MPPI`; шлюз выбирает прямую MPPI-команду и не включает поворот по касательной пути, наблюдатель считает этот статус отдельно от `OK`. Диагностика MPPI публикуется и для recovery с флагом `recovery=true`. Тесты, Compose, сборка и реальный матч выполнены; результаты приведены выше. Новая проверка не должна опираться на старые доли `planner OK` как на равные по смыслу.

**Следующая проверка разворота: валидные MPPI rollout и просвет.** Однократный скачок маршрута в 5,5 с заставил робота разворачиваться несколько секунд при ~0,07–0,10 м/с; неизвестно, были ли доступные безопасные движущиеся дуги. Планировщик теперь публикует `/navigation/mppi_diagnostics` и `/opponent/navigation/mppi_diagnostics` (JSON String) на каждом обычном цикле MPPI: результат, число допустимых образцов, просвет от центра до ближайшего препятствия, продвижение и первое управление. `trace_motion.py` совмещает их с фактическими скоростью, позой, поведением и направлением. Это диагностика, cost и исполнитель не меняются. После `compileall`, тестов, Compose и сборки повторить scenario 2/seed 15 и проверить 5–9 с; если большинство образцов невалидны, работать с геометрией и возможностью дуги, если их много, разбирать objective/выбор mean или sample.

**Абсолютный курс и один дорогой разворот.** На принятом `f8fab44` (без пересборки, изменена только трасса) команда `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 15 --active-s 90 --scenario 2 --trace` создала `results/series-20260930T174830Z/`: цель за 31,8 с, скорость исследователь/страж 0,183/0,212 м/с, 0 контактов. В `trace_motion.py` теперь записаны поза и абсолютный курс робота. Для исследователя в ~5,5 с направление ближайшего глобального участка резко изменилось на **-2,23 рад**, когда он переместился менее чем на 0,2 м; это подтверждает **один** реальный перевыбор коридора, а не частые скачки. До него робот ориентировался примерно на северо-запад (абсолютное направление ~1,6–2,1 рад), после — на юго-восток (~-0,8…-1,1 рад). С 6 до 9 с он доворачивался с курсом ~1,66→-1,30 рад при скорости в контрольных секундах 0,106/0,080/0,070/0,099 м/с, оценка расстояния до стража сократилась ~1,21→0,75 м. Позднее MPPI доходил до фактических ~0,50 м/с и всё же достиг цели. Значит часть потери средней скорости вызвана крупным разворотом после изменения маршрута под угрозой; одной настройки продольной дисперсии недостаточно. Следующий шаг: в момент такого переключения измерить число безопасных MPPI rollout, доступный просвет и выбранный первый угол; затем выбирать между плавным ранним обходом и безопасным разворотом на месте. Данные одного заезда не доказывают, что все медленные матчи имеют ту же причину.

**Подготовка абсолютной трассы (результат выше).** Изменение относительной ошибки направления к пути на 1,5 рад могло быть поворотом самого робота; в записи seed 15 счётчик больших скачков абсолютного направления глобального маршрута равен только одному. Поэтому прежнее объяснение «часто перевыбирается коридор» не подтверждено. В `trace_motion.py` добавлены абсолютная поза и курс для разделения этих причин. Диагностический матч на принятом коде MPPI без сборки и его вывод приведены выше.

**Ранний EVADE по измеренной скорости сближения отклонён.** Для кандидата ускоряющих образцов испытана прогнозная дистанция до соперника через 1 с. `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 90 --scenario 2 --trace --build`, `results/series-20260930T173037Z/`, код `3fecb97+dirty.3d9385a6f199`: 1 цель/2 поимки/0 контактов, скорость исследователь/страж 0,223/0,250 м/с, RMS ошибки 0,072/0,061 м, RMS углового ускорения 1,671/1,410 рад/с², stop-go 3,0/1,3. Непрогнозный кандидат на тех же seed (`series-20260930T170042Z/`): 2 цели/1 поимка/0 контактов, 0,211/0,213 м/с, RMS 0,064/0,074, угловое ускорение 1,841/1,500, stop-go 7,7/6,7. Улучшены отдельные числа, но исследователь потерял цель на seed 16, а ошибка его траектории в среднем выросла. `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace`, `results/series-20260930T173729Z/`: 1 цель/2 поимки/0 контактов, скорость 0,191/0,249 против 0,210/0,318, RMS 0,073/0,070 против 0,070/0,048, угловое ускорение 2,048/1,593 против 1,970/1,464, stop-go 9,7/5,3 против 2,7/3,0. На первом старте ухудшились все три приоритетные метрики; `src/hsl_decision/hsl_decision/core.py` и экспериментальный тест возвращены к `3fecb97`. Расширенная трасса оставлена: она помогает связать behavior, оценённую дистанцию и выбранную команду. После возврата образ пересобран, его идентификатор указан ниже. Следующий шаг — отдельно измерить абсолютный курс робота и направление ближайшего участка пути, затем разбирать разворот при малой скорости; не менять момент EVADE без более точного объяснения.

После возврата выполнены `helm clean duel` и `helm build duel`; образ `jr_image:latest` — `sha256:8d553bda63f71bf3439f90f0b08aa415be6c3a1cdf8881d3eb13e4987ddb0600`, работают только `ros-daemon` и `wait-init`. `68` тестов прошли на восстановленном дереве, рабочее дерево было чистым до этой записи. Живой матч на вновь собранном образе ещё не запускался; алгоритм совпадает с checkpoint `3fecb97`, испытанным до опыта раннего EVADE. Последний коммит с трассой и откатом — `086faed`.

**Гипотеза раннего EVADE по измеренному сближению (опыт отклонён ниже).** Повтор кандидата ускоряющих образцов без пересборки, с расширенной трассой: `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 15 --active-s 90 --scenario 2 --trace`, `results/series-20260930T172459Z/`, код `3fecb97+dirty.3bd8afb9419d`: цель исследователя за 32,0 с, скорость 0,173/0,205 м/с, 0 контактов. Это ещё один вариант того же seed с отличающимся исходом/скоростью, поэтому один seed не воспроизводит траекторию побитово. Трасса теперь записывает текущий behavior и дистанцию до соперника по **оценке LiDAR**, а также команду прямого MPPI и ошибку направления. У исследователя при GOAL средняя фактическая/командная скорость 0,109/0,106 м/с и средняя абсолютная ошибка направления к глобальному пути 1,098 рад; при EVADE 0,200/0,199 м/с и 0,535 рад. В 5–7 с относительная ошибка направления к глобальному пути изменилась примерно с -0,90 до -2,42 рад при подходе стража на ~1,28 м; EVADE включился позже. Это не доказывает резкую смену самого маршрута: трасса без собственной абсолютной ориентации не разделяет поворот робота и изменение пути, а счётчик больших скачков абсолютного направления глобального маршрута за матч равен одному. Гипотеза: более ранний переход на опасный маршрут по измеренной скорости сближения даст время повернуть плавнее и двигаться вперёд. В decision manager для свежего трека используется минимум текущей дистанции и дистанции до линейно предсказанной позиции соперника через 1 с; никакой предполагаемой скорости стража нет. Запрос к цели и планировщик не менялись. Добавлен тест: при расстоянии 1,4 м и измеренной скорости -0,4 м/с включается EVADE, при +0,4 остаётся GOAL. `69` тестов, compileall, Compose config и diff check прошли. Последующие живые серии и решение о возврате правила записаны ниже.

**Третий старт, новые seed 18–20.** `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 18 --active-s 90 --scenario 3 --trace`, `results/series-20260930T171411Z/`, тот же собранный кандидат: 3 достижения цели/0 поимок/0 контактов. Скорости исследователь/страж по матчам: 0,260/0,201, 0,189/0,221, 0,237/0,229 м/с; среднее 0,229/0,217, один исследователь ниже 0,2. Средний по матчам RMS боковой ошибки 0,085/0,054 м, RMS углового ускорения 1,897/1,339 рад/с², stop-go 9,7/6,0. Сопоставимой базы нынешнего кода для scenario 3 нет, поэтому прирост от ускоряющих образцов здесь не доказан. Девять коротких матчей на трёх стартах в сумме: 6 целей/3 поимки, 0 контактов, но у исследователя 4/9, у стража 1/9 скоростей <0,2 м/с. Алгоритмическое изменение сохранено как **промежуточный checkpoint** благодаря улучшению двух сравнимых стартов. Открыты минимум скорости, отдельные скачки углового ускорения, слабая поимка на scenario 3 и полная серия 20×360 с. Следующая причинная задача — разобрать низкую скорость/развороты в EVADE и не допускать маршрута к зоне регламентной поимки.

**Ускоряющие образцы MPPI, испытание на трёх стартах.** На восстановленном `988db94` проведён диагностический матч без сборки: `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 15 --active-s 90 --scenario 2 --trace`, `results/series-20260930T165533Z/`, образ `sha256:064cd344c8f66bfc9767b6c722e92ecedff33d912f28fe4bba38af5aa3a280ff`. Цель за 29,2 с, скорости исследователь/страж 0,204/0,222 м/с, 0 контактов. Ранее тот же seed дал 0,182/0,258 и цель за 31,6 с; асинхронность не позволяет приписывать разницу коду. Новая трасса записывает ошибку направления к глобальному/локальному пути и команды MPPI отдельно от финального `/cmd_vel`. При `OK` у исследователя: 84 отсчёта с |ошибкой направления к глобальному пути| <0,25 рад дали средние фактическую/командную скорости 0,240/0,254 м/с; 76 отсчётов с ошибкой ≥0,8 рад — 0,160/0,161 м/с. Шлюз почти не ограничивал команду при `OK`; снижение возникает в выборе MPPI. Логи planner показывают часто 23–117 валидных образцов из 192. Гипотеза: вокруг медленного warm start случайная выборка редко даёт достаточно быстрый, но безопасный rollout. В каждый batch добавлены два варианта разгона до штатного `max_speed` с текущим и нулевым рулением. Они проходят прежние ограничения ускорения и полную проверку коллизий; soft cost не изменён. `68` тестов, compileall, Compose config, diff check прошли.

Серия `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 90 --scenario 2 --trace --build`, `results/series-20260930T170042Z/`, код `988db94+dirty.68a01be378a4`: 2 цели/1 поимка, 0 контактов. Скорость исследователь/страж 0,211/0,213 против 0,190/0,230 в сырой базе `series-20260930T115658Z/`; средний по матчам RMS ошибки 0,064/0,074 против 0,072/0,086 м; RMS углового ускорения 1,841/1,500 против 1,960/1,689; stop-go 7,7/6,7 против 11,3/8,0. Но у seed 15 исследователь в режиме ухода от стража 63% времени ехал только 0,095 м/с, повернул на месте 35% и был пойман за 16 с; страж 0,188 м/с. На seed 16 исследователь достиг 0,335 м/с, seed 17 — 0,203. У обоих ролей по одному матчу ниже 0,2; только агрегата недостаточно.

Серия `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace`, `results/series-20260930T170823Z/`, та же сборка: 1 цель/2 поимки, 0 контактов. Скорость исследователь/страж 0,210/0,318 против 0,197/0,240 в базе `series-20260930T114535Z/`; RMS 0,070/0,048 против 0,097/0,076 м; RMS углового ускорения 1,970/1,464 против 1,988/1,315; stop-go 2,7/3,0 против 12,0/10,3. Guardian seed 13 стал менее плавным по угловому ускорению (1,597→2,530 рад/с²), у исследователя seed 14 скорость лишь 0,159 м/с до ранней поимки. Суммарно шесть матчей дали 3 цели/3 поимки без контактов и улучшение большей части метрик, но порог каждого робота и плавность стража неустойчивы. Кандидат оставлен для проверки scenario 3 с новыми seed, не объявлен основным подтверждённым решением. Следующий шаг — scenario 3, затем разбор разворотов исследователя в EVADE и скачков угловой команды стража; при регрессии откатить.

**Снижение дисперсии MPPI по скорости отклонено.** На сырой опоре MPPI часты stop-go (scenario 2 baseline: исследователь 10/6/18 по seed 15–17, страж 7/6/11), все три исследователя ниже 0,2 м/с, хотя коллизий нет. Гипотеза: снижение `velocity_std` с 0,22 до 0,15 м/с вокруг тёплого старта уменьшит почти нулевые скорости и межцикловые колебания, не обрезая допустимый диапазон 0–0,5 м/с. Угловая дисперсия, ограничения приводов, critic и шаблоны обхода не менялись. Команда: `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 90 --scenario 2 --trace --build`, серия `results/series-20260930T163629Z/`, ревизия `7e72380+dirty.1bf672397636`. 68 тестов, compileall, Compose config и diff check прошли. База `series-20260930T115658Z/`: 2 цели/1 поимка/0 контактов, скорость исследователь/страж 0,190/0,230 м/с, RMS до глобального пути 0,072/0,086 м, RMS углового ускорения 1,960/1,689 рад/с², stop-go 11,3/8,0. Опыт: 1 цель/2 поимки/0 контактов; скорость 0,179/0,247 м/с; RMS 0,089/0,082 м; RMS углового ускорения 1,756/1,540 рад/с²; stop-go 5,3/4,7. Скорости исследователя по seed 15–17: 0,180/0,170/0,188 (все <0,2), стража 0,203/0,276/0,262. У seed 15 исследователь p95 отклонения вырос 0,170→0,291 м, stop-go 10→13; страж скорость упала 0,258→0,203 и stop-go выросли 7→13. В длинном seed 15 при статусе `OK` измерены средняя фактическая скорость 0,182 у исследователя и 0,204 у стража; восстановление было редким, поэтому недостаток скорости не объясняется только recovery. Опыт улучшил часть плавности и поимки, но ухудшил скорость/точность исследователя; `velocity_std=0.22` восстановлен. Для воспроизводимой проверки создан `benchmarks/report_motion.py`; `trace_motion.py` теперь включает в ряд статус планировщика и фактическую угловую команду, чтобы разбирать эпизоды ошибки/остановки. Следующий шаг — исследовать, почему MPPI выбирает низкую поступательную скорость при `OK`: анализировать валидные rollout, cost прогресса/выравнивания, фактический clearance и частоту перезаписи команды. Не сокращать speed variance снова без новой причины.

После возврата параметра `helm clean duel` удалил контейнеры дуэли, `helm build duel` успешно собрал текущий код; локальный образ `jr_image:latest` — `sha256:064cd344c8f66bfc9767b6c722e92ecedff33d912f28fe4bba38af5aa3a280ff`. Остались только служебные `ros-daemon` и `wait-init`. Живой матч именно на этом образе после возврата параметра не запускался; последние фактические результаты базового варианта — указанные серии до опытов. Локальный checkpoint отчёта и инструмента — `48565e5`.

**Разделение причины низкой скорости.** Проверка `mppi_local_guidance` напрямую в пустом коридоре длиной ~5 м, маршрут прямой, `max_speed=0.5`, `dt=0.2`, ограничения ускорений 0,5 м/с² и 2 рад/с², seed 13: при измеренной скорости 0/0,1/0,2/0,3/0,4 м/с первая команда стала 0,096/0,194/0,292/0,391/0,485 м/с. В 20 последовательных шагах с тёплым стартом средняя выбранная скорость 0,432 м/с, максимальное боковое смещение 0,056 м; знак угловой команды сменился 6 раз. После 20-го шага тест со *старым фиксированным* маршрутом потерял привязку; в реальном узле глобальный путь пересчитывается от текущей позы, поэтому это не свидетельство живого сбоя. Вывод: само ограничение ускорения и базовая функция MPPI способны разогнаться на свободной прямой; низкую скорость `OK` в дуэли следует искать в меняющейся геометрии пути, просвете, противнике и оценке кандидатов. Шесть смен знака даже на свободной прямой требуют отдельной проверки плавности.

**Сглаженная короткая опора MPPI отклонена.** Гипотеза: безопасное сглаживание ближайших 2,5 м маршрута A* уменьшит боковые рывки и отклонение без потери средней скорости и поимок. Изменение включалось `HSL_MPPI_REFERENCE=smoothed`; глобальный A* и проверки коллизий rollout не менялись. `68` тестов, `compileall`, Compose config и `git diff --check` прошли. Команда: `HSL_MPPI_REFERENCE=smoothed python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 90 --scenario 2 --trace --build`, образ собран, серия `results/series-20260930T124151Z/`, ревизия `7e72380+dirty.4a6d1dfe0bce`. База с теми же условиями `series-20260930T115658Z/`: 2 цели/1 поимка/0 контактов, средняя скорость исследователь/страж 0,190/0,230 м/с, средний по матчам RMS до глобального пути 0,072/0,086 м, RMS углового ускорения 1,960/1,689 рад/с², stop-go 11,3/8,0. Опыт: 3 цели/0 поимок/0 контактов, скорость 0,208/0,242 м/с, RMS до глобального пути 0,110/0,107 м, RMS углового ускорения 1,699/1,595 рад/с², stop-go 9,0/7,0. У исследователя скорости по seed 15–17: 0,188/0,249/0,188; у стража 0,245/0,219/0,263. У исследователя seed 17 RMS ошибки вырос 0,074→0,177 м и p95 0,148→0,369 м; на seed 16 страж перестал ловить. Planner p95 у одного робота доходил до 190,66 мс. Короткое сглаживание улучшило часть метрик плавности, но ухудшило точность и результат стража; скорости ниже 0,2 в двух заездах. Экспериментальные изменения planner/Compose/trace/README/AGENTS возвращены к `7e72380`; данные серии сохранены. Следующий шаг — разбирать причины большой ошибки/низкой скорости на сырой опоре MPPI и менять оценку/управление без потери поимок.

Решение пользователя после графиков: прямой MPPI сделать основным, исходный MPC и глобальную траекторию оставить в резерве. До изменения default на одной серии scenario 1 MPPI дал 2 цели/1 поимку и скорости 0,189/0,267 м/с (`series-20260930T064908Z/`), на scenario 2 — 3 цели и 0,222/0,235 (`series-20260930T065931Z/`). MPC с сырым длинным глобальным путём дал 0 целей/2 поимки/1 тайм-аут, 0,147/0,131 (`series-20260930T105537Z/`), со сглаженным — 3 тайм-аута и 0,170/0,150 (`series-20260930T111126Z/`). Это короткие группы, а не окончательная статистика побед. Изменены defaults `HSL_CONTROL_MODE=mppi` в Compose, launch, planner, шлюзе и runner. Оба узла исходного MPC остаются запущены и получают локальный путь для recovery; `HSL_CONTROL_MODE=mpc` и `HSL_MPC_PATH_SOURCE=global|smoothed` позволяют вернуться к контрольным вариантам. Исправлен случай, когда планировщик в recovery публиковал проверенный прямой путь без MPPI-команды, но статус оставался `OK`, из-за чего шлюз выбирал нулевую MPPI-команду: теперь статус `RECOVERY_FALLBACK` выбирает MPC.

**Проверка нового default.** `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace --build`, без `HSL_CONTROL_MODE`, `results/series-20260930T114535Z/`: 2 цели исследователя (54,3/55,7 с), 1 поимка (6,8 с), 0 контактов. Средние скорости исследователь/страж **0,197/0,240 м/с**, доли движения 0,825/0,867, planner OK 0,881/0,962. Два исследователя и два стража из трёх ниже промежуточного порога 0,2; баланс и скорость пока нестабильны. Живые параметры обоих planner/gate показали `mppi`, Lat-MPC у первого робота подписан на `/navigation/local_path`; у обоих `/cmd_vel` по одному издателю. После финиша оба `/cmd_vel=0`, `/match/allowed=false`. В trace первого заезда `RECOVERY_FALLBACK` появлялся у обеих ролей; сам факт статуса не доказывает движение через резерв — нужна отдельная проверка выбранной команды. Следующий шаг: измерить активность MPC в этом статусе, затем проверить scenario 2/3 и разбирать низкую скорость стража.

**Новые seed 15–17, scenario 2 и резерв.** `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 90 --scenario 2 --trace`, без пересборки, `results/series-20260930T115658Z/`: seed 15 — цель за 31,6 с, seed 16 — поимка за 24,4 с, seed 17 — цель за 64,8 с, контактов 0. Средние скорости исследователь/страж **0,190/0,230 м/с**; все три исследователя ниже 0,2, стражи — выше. По шести матчам после переключения — 4 цели, 2 поимки, 0 контактов, но пороги скорости/баланс не доказаны. Новая трасса recovery показывает свежесть MPC-команды в 100% fallback-отсчётов первого матча и ненулевую финальную команду в 100%/83,3% у первого/второго роботов (53/12 отсчётов); в остальных матчах резерв также выдавал команды. Значит исправление `RECOVERY_FALLBACK` проверено в живой дуэли. Пользователь уточнил новый приоритет: снижать латеральное отклонение **без потери скорости**. Текущий RMS отклонения до глобального маршрута по scenario 2 seeds 15–17: исследователь 0,082/0,059/0,074 м, страж 0,071/0,083/0,104 м; p95 до 0,23 м у стража seed 17. Следующий изолированный эксперимент — штрафовать большие отклонения от чистого маршрута в MPPI, сохраняя член прогресса и затем сравнивая те же seed по RMS/p95, скорости, исходам и контактам.

**Эксперимент отклонения, проверка идёт.** В `_evaluate` добавлен квадратичный штраф за среднее превышение латеральной ошибки над 0,35 радиуса робота; он действует только когда глобальный коридор свободен для PathAlign. Базовый штраф малой ошибки и поощрение продвижения не изменены, ограничения команд и скорости не изменены. Гипотеза: уменьшатся RMS и p95 отклонения, особенно выбросы >0,2 м, без снижения средней скорости. `68` тестов и `compileall` прошли. Нужны сборка и повтор scenario 2, seed 15–17, 90 с, сравнение с `series-20260930T115658Z/`. Если скорость падает или появляются контакты, откатить штраф.

**Штраф для обеих ролей не принят.** Повтор `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 90 --scenario 2 --trace --build`, `results/series-20260930T120629Z/`: 3 цели, 0 поимок, 0 контактов; скорость исследователь/страж **0,281/0,223** против **0,190/0,230** в базе. Исследователь стал быстрее во всех трёх заездах (0,281/0,302/0,261 м/с), RMS отклонения 0,076/0,062/0,071 против 0,082/0,059/0,074; stop-go 5/1/3 против 10/6/18. Но страж в seed 16 замедлился 0,228→0,176 м/с, доля поворота на месте выросла 0,113→0,254, он перестал ловить; в seed 15 его RMS вырос 0,071→0,123 м, p95 0,149→0,308. Seed 17 улучшился по всем показателям, но общий критерий обеих ролей не выполнен. Гипотеза: подвижная цель стража делает его глобальный путь слишком быстро меняющейся ссылкой для дополнительного штрафа. Следующий изолированный опыт — оставить эту поправку только для исследователя, у стража вернуть прежнюю цену ошибки; повторить обе карты и сравнить каждую роль, скорость и плавность.

**Ролевая поправка, scenario 2.** `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 15 --active-s 90 --scenario 2 --trace --build`, `results/series-20260930T121653Z/`: штраф больших ошибок включён только у исследователя. 3 цели, 0 поимок, контактов нет. Средняя скорость исследователь/страж **0,236/0,237** м/с против **0,190/0,230** в базе. Средний по матчам RMS ошибки до сырого глобального маршрута исследователь **0,088** против **0,072** м, страж **0,072** против **0,086** м. Средний RMS углового ускорения 1,846/1,480 против 1,960/1,689 рад/с²; stop-go 6,3/3,3 против 11,3/8,0. Исследователь seed 15 всё ещё ниже 0,2, страж выше во всех трёх. Изменение улучшает среднюю скорость и плавность, но ухудшает отклонение исследователя и не дало поимки на этих seed. Проверить scenario 1, затем выбрать между откатом штрафа и изменением опорного пути: сырой A* с клеточными углами может противоречить плавной быстрой езде на поворотах.

**Ролевая поправка, scenario 1 и вывод.** `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace`, `results/series-20260930T122720Z/`: 3 цели, 0 поимок, 0 контактов. Средние скорости исследователь/страж **0,233/0,226** против **0,197/0,240** в базовом `series-20260930T114535Z/`. Средний RMS до сырого глобального маршрута **0,089/0,083** против **0,097/0,076** м. Исследователь во всех трёх матчах ≥0,2; страж seed 13 — 0,191. Stop-go исследователь/страж 12/8 против 12/10,3; RMS углового ускорения 1,880/1,782 против 1,988/1,315. В совокупности двух стартов ролевой штраф ускорил исследователя и в одном старте уменьшил его отклонение, но дал **0 поимок в шести матчах**, ухудшил некоторые метрики стража и не обеспечил 0,2 каждому роботу в каждом матче. Вариант **отклонён как основной**; следующий опыт — сглаживать безопасный опорный маршрут для MPPI вместо давления на следование сырой клеточной линии. Исходная геометрия A* остаётся в глобальном планировщике, безопасность локального rollout сохраняется.

## Эксперимент: MPPI как контроллер за защитным шлюзом (30.09, проверка продолжается)

**Попытка настройки исходного MPC отклонена.** После сохранения прямого MPPI в коммите `32f3400` испытан профиль `HSL_MPC_PROFILE=tuned`: единственное изменение к исходному Lat-MPC — `a_lat_max=0,08` вместо `0,03` м/с². Команда `HSL_CONTROL_MODE=mpc HSL_MPC_PROFILE=tuned python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace --build`, `results/series-20260930T070903Z/`: seed 12/13 — тайм-ауты, seed 14 — поимка на 62,9 с; 0 целей, 1 поимка, 0 контактов. Средняя скорость исследователя/стража **0,176/0,163 м/с**, ниже 0,2 у обоих во всех матчах. В trace seed 12 медианная `v_curve` 0,132/0,118 м/с и доля `v_curve<0,2` 63/76%: увеличение порога бокового ускорения не устраняет замедление при кривизне плотного временного пути. На тех же seed прямой MPPI дал 2 цели/1 поимку и 0,189/0,267 м/с. Вывод: этот профиль отклонён; дальше убрать Lat/CC-MPC из активного контура, оставив защитный шлюз и прямой MPPI, включая восстановление. Проверка новой конфигурации ещё требуется.

**Новая гипотеза пользователя: длинный глобальный путь для Lat-MPC.** Прежде чем удалять исходный контроллер, добавлен переключатель `HSL_MPC_PATH_SOURCE=global|local` (по умолчанию `local`). Только вход `pacemaker_path_topic` Lat-MPC меняется на `/navigation/global_path`; локальный MPPI-путь остаётся в защитном шлюзе и RViz. Настройка `a_lat_max=0,08` удалена из рабочего кода после отрицательного результата, исходный YAML сохранён. Риск гипотезы: глобальный маршрут может не учитывать новые препятствия и резкие углы; безопасность и движение надо проверять в Gazebo, а не выводить из существования локального пути. Следующий шаг: собрать и сравнить ту же серию seed 12–14, scenario 1, с прямым MPPI и исходным локальным MPC.

**Сырой глобальный A* в MPC не помог.** `HSL_CONTROL_MODE=mpc HSL_MPC_PATH_SOURCE=global python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace --build`, `results/series-20260930T105537Z/`: seed 12 — поимка 41,9 с; seed 13 — тайм-аут; seed 14 — поимка 22,3 с. Скорости исследователь/страж 0,147/0,131 м/с; все шесть ролевых заездов ниже 0,2. Достижений площадки нет. Контактов со стенами нет, при поимке seed 14 — по одному контакту роботов. В живом графе вход Lat-MPC действительно `/navigation/global_path`, `/cmd_vel` имеет одного издателя — шлюз. Для первых двух заездов медианная `v_curve` обоих контроллеров осталась на нижнем пределе 0,1 м/с; в seed 12 доля `v_curve<0,2` — 81/90%. Сырой A* состоит из соседних клеток с углами, и длинный горизонт `curve_lookahead=2,0` видит их заранее. Следующая проверка — безопасно сглаженный длинный маршрут для MPC, а не вывод, что длинный вход сам по себе бесполезен.

**Подготовлен эксперимент сглаженного длинного пути.** `HSL_MPC_PATH_SOURCE=smoothed` публикует `/navigation/mpc_path`: видимые отрезки A* сокращаются на дальности до 2 м, углы скругляются квадратичной кривой только если каждый сегмент проходит `safe_segment`, затем путь дискретизируется шагом не более 0,1 м. В recovery используется короткий проверенный локальный путь. Основной MPPI и шлюз по-прежнему используют свой локальный путь. `68` тестов прошли, включая проверку проходимости сглаженного пути вокруг препятствия. Это пока только алгоритм; результат дуэли, скоростной профиль и безопасность нового пути не доказаны.

**Сглаженный длинный путь в дуэлях.** `HSL_CONTROL_MODE=mpc HSL_MPC_PATH_SOURCE=smoothed python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace --build`, `results/series-20260930T111126Z/`: три тайм-аута, 0 целей, 0 поимок, 0 контактов. Средние скорости исследователь/страж **0,170/0,150 м/с**, все шесть ролевых заездов ниже 0,2. На seed 12 `v_curve` исследователя выросла до медианы 0,127 м/с, стража осталась 0,1; на seed 13 у обоих медиана 0,1. У обоих p95 вычисления планировщика на seed 12 превысил 200 мс (217/209), так что сглаживание добавило заметную нагрузку. По этому сравнению вариант не принят как решение. Длинный вход без смены регулятора и профиля кривизны не дал требуемой езды.

**Телеметрия для запрошенных графиков.** `benchmarks/trace_motion.py` теперь может сохранять временной ряд фактической скорости и знакового расстояния до ближайшего сегмента глобального и управляющего пути; функция графика находится в `benchmarks/plot_control_comparison.py`. На одинаковых условиях scenario 1, seed 13, физические роли исследователь/страж: прямой MPPI `results/series-20260930T112819Z/` достиг цели на 39,5 с, скорости 0,167/0,220; MPC со сглаженным длинным путём `results/series-20260930T113038Z/` достиг цели на 59,0 с, скорости 0,141/0,156. Контактов нет. Это сравнение двух отдельных матчей; оно не заменяет серии. Следующий шаг — построить такой же ряд для сырого глобального MPC, затем оставить лучший вариант управления и продолжить проверку на разных стартах.

**Третий ряд и итоговый график.** `HSL_CONTROL_MODE=mpc HSL_MPC_PATH_SOURCE=global python3 benchmarks/run_duel_series.py --runs 1 --start-seed 13 --active-s 90 --scenario 1 --trace`, `results/series-20260930T113504Z/`: сырой глобальный путь, поимка на 15,2 с, скорости исследователь/страж 0,169/0,158, контактов 0. Три режима на одном seed показаны в `results/control-comparison-seed13-three-modes.png` и `.svg`: верхние панели — измеренная скорость, нижние — знаковое латеральное отклонение до ближайшего сегмента **общего глобального** маршрута, по ролям. Полупрозрачные линии — отдельные отсчёты, плотные — среднее в окне 1 с. RMS глобального отклонения исследователя: прямой MPPI 0,106 м, сглаженный глобальный MPC 0,104 м, сырой глобальный MPC 0,163 м; стража: 0,062/0,161/0,026 м. Последняя малая ошибка сырого MPC относится к короткой 15-секундной поимке, поэтому не доказывает превосходство в длительном следовании. Оба длинных варианта MPC уступают прямому MPPI по средней скорости стража, а в трёхматчевой серии сглаженный вариант не дал ни одного целевого события. Решение о контроллере принимать по повторным исходам, контактам и нагрузке, а не только одному графику.

**Второй старт, тот же прямой MPPI.** `HSL_CONTROL_MODE=mppi python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 2 --trace`, `results/series-20260930T065931Z/`: seed 12/13/14 — исследователь достиг цели за 21,8/36,6/24,9 с, поимок нет. Средняя скорость исследователя/стража 0,222/0,235 м/с, доля движения 0,837/0,948, planner OK 0,969/0,978. Контактов нет. Скорость исследователя во всех трёх заездах выше промежуточного порога 0,2; страж в одном ниже. По двум стартам и шести коротким заездам прямой MPPI улучшил движение, но баланс ролей и полная устойчивость не доказаны. Следующий отдельный цикл — настройка параметров исходного Lat-MPC на тех же seed и стартах; затем сравнение с прямым MPPI и решение о его включении по умолчанию.

Причина: официальный Nav2 MPPI возвращает `TwistStamped`, а наша адаптация передаёт временную траекторию как `Path` другому Lat-MPC. Локальные пути получают частые короткие шаги и высокую оценку кривизны; изменение геометрии и касательной в проверенных сериях не исправило скорость и иногда привело к контакту. Для контролируемого сравнения добавлен режим `HSL_CONTROL_MODE=mppi` (по умолчанию остаётся `mpc`). В новом режиме планировщик публикует первую команду выбранной проверенной траектории в `/navigation/mppi_cmd_vel`, а единственный шлюз выбирает её при статусе `OK`; для recovery остаётся штатный MPC. MPPI ограничивает прогноз ускорениями и командной скоростью из исходных YAML CC/Lat-MPC (0,5 м/с², 2 рад/с², 0,5 м/с), не добавляя ограничения в исходный MPC и не предполагая скорость соперника. Шлюз продолжает проверять разрешение движения и свежесть pose/scan/path/intent/команды; до старта и после финиша должен оставаться ноль. Это **местная адаптация, не запуск C++ плагина Nav2**. `108` тестов, `compileall`, `git diff --check`, `docker compose ... config --quiet` прошли. Нужны сборка, проверка реальных ROS-топиков и короткая дуэль, затем серия на тех же seed; режим нельзя считать безопасным или успешным до этих прогонов.

**Короткий тест режима.** `HSL_CONTROL_MODE=mppi python3 benchmarks/run_duel_series.py --runs 1 --start-seed 12 --active-s 45 --scenario 1 --trace --build`, `results/series-20260930T064502Z/`: поимка за 11,9 с; скорости исследователь/страж 0,095/0,330 м/с, контактов 0. Живой ROS-граф подтвердил ровно одного издателя обоих `/cmd_vel` — соответствующие шлюзы. У исследователя оптимизатор чаще выдавал малую скорость и повороты, у стража прямой путь и высокую скорость; эта короткая поимка не подтверждает общую устойчивость. После прогона `dt` модели прямого MPPI согласован с периодом публикации 0,2 с (в пробном образе было 0,15 с); нужен повторный билд и сопоставимая серия. Штатный режим `mpc` не меняет параметры.

**Сопоставимая серия с `dt=0,2`.** `HSL_CONTROL_MODE=mppi python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace --build`, `results/series-20260930T064908Z/`: seed 12 — поимка за 8,5 с, seed 13/14 — цели за 34,4/52,0 с. Средние скорости исследователь/страж **0,189/0,267 м/с** против **0,172/0,145** на прежнем MPPI→Path→MPC (`series-20260929T221103Z/`); у стража >0,2 во всех матчах, у исследователя в двух из трёх. Контактов со стенами 0, по одному контакту роботов у обоих только при поимке seed 12. В этом матче исследователь: 0,132 м/с, уклонение 30% времени, recovery 7% времени уклонения, а страж — 0,358 м/с. Остальные матчи: исследователь 0,214/0,220, страж 0,204/0,240. Planner OK в среднем 0,933/0,975. Это улучшение короткой серии и доказательство отсутствия прежней связи `v_curve≈0,1` с фактической командой в прямом режиме, но **порог 0,2 для исследователя в каждом заезде и устойчивость на других стартах не подтверждены**. Следующая проверка — scenario 2/3, затем работа над уклонением при ранней угрозе; полный 20×360 ещё не проводился.

## Проверка пространственной касательной для плотного MPPI-пути (30.09)

Гипотеза: шлюз и watchdog брали начальное направление из фиксированного числа точек, хотя шаг MPPI определяется временем и в пространстве может быть миллиметровым. Пробно они измеряли направление по 0,06 м вдоль пути; trace использовал ту же оценку. Новый тест воспроизвёл ситуацию, когда первые шесть точек почти неподвижны и старый watchdog не видел направления. `108` тестов, `compileall`, `git diff --check` и сборка прошли. Проверка: `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace --build`, `results/series-20260930T062148Z/`, scenario 1, seeds 12–14, 90 с. Исходы: 1 цель / 1 поимка / 1 тайм-аут. Скорости исследователь/страж **0,170/0,141 м/с** против **0,172/0,145** в MPPI базе `series-20260929T221103Z/`. Исследователь коснулся стены на seed 14 в `(−0,942; 0,801)` на 81,7 с; в базе контактов нет. Доля отсечений шлюзом не снизилась последовательно по трём seed. Поэтому код изменения касательной и пробные тесты **откачены**; геометрический дефект обнаружен, но предложенное исправление в связке с текущим контроллером не улучшило езду. Заранее проверена проба ограничить rollout MPPI ускорениями собственного MPC: тест прямого пути показал, что начальные шаги становятся слишком короткими для текущего контракта `Path`; этот незавершённый вариант откачен без дуэли. Полный пакет `nav2_mppi_controller` в текущем образе отсутствует (`ros2 pkg prefix` возвращает `Package not found`). Следующий шаг — проверять MPPI как единый источник команды скорости за защитным шлюзом или строить пространственный локальный путь специально под Lat-MPC; прежнюю временную траекторию как `Path` не считать достаточным переносом Nav2.

## Повторная проверка Nav2 и интерфейса штатного MPC (30.09)

**Наблюдение.** На MPPI с исходными YAML контроллера серия `results/series-20260929T221103Z/` дала 1 цель / 1 поимку / 1 тайм-аут, скорости 0,172/0,145 м/с, 0 контактов. В seed 12 локальные пути имели медианную максимальную кривизну 8,24/5,59 1/м и множество сегментов <2 см; Lat-MPC через свой `v_curve = sqrt(a_lat_max/max|κ|)` обычно опускал скорость до 0,1 м/с. Прореживание этих сегментов уменьшило кривизну, но в `results/series-20260929T222148Z/` скорость почти не выросла, исчезли цели исследователя и появились контакты со стеной. Прореживание удалено; тесты после отката: 106 passed.

**Сверка оригинального алгоритма.** В [Nav2 Humble MPPIController](https://github.com/ros-navigation/navigation2/blob/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_mppi_controller/src/controller.cpp) результат `optimizer_.evalControl(...)` сразу возвращается как `TwistStamped`; оптимизированная траектория поступает в визуализатор. [Optimizer](https://github.com/ros-navigation/navigation2/blob/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_mppi_controller/src/optimizer.cpp) фильтрует последовательность команд, берёт команду для текущего шага и сдвигает её. В нашем `mppi.py` вместо команды публикуется префикс предсказанной траектории как `Path`; затем независимый Lat-MPC интерпретирует её как пространственную опорную линию, измеряет кривизну и ведёт робота с другой скоростью. Это **неэквивалентная передача между двумя контроллерами**. [Nav2 RPP](https://github.com/ros-navigation/navigation2/blob/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp) тоже возвращает `TwistStamped`, но её геометрическая дуга имеет равномерные пространственные шаги и постоянную кривизну, что лучше соответствует входному `Path` штатного MPC. Это инженерный вывод из исходников и трасс; его ещё нужно подтвердить сравнительной дуэлью с текущими YAML.

**Проверенный эксперимент, затем откат.** Возвращалась геометрия RPP из checkpoint `ba7a3a9`: пересечение глобального пути с окружностью lookahead, короткая касательная дуга с шагом около 0,05 м и проверка каждого сегмента на препятствия, границу и соперника. Она публиковалась основным локальным `Path`, MPPI был резервом. Добавленные 4 теста прошли вместе с остальными (`110 passed`), `compileall` и сборка прошли. Команда сравнения: `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace --build`.

**Неудачный первый запуск кандидата.** `results/series-20260930T054147Z/`, та же команда: seed 12 показал тайм-аут, 0,162/0,0 м/с; seed 13 — тайм-аут, 0,142/0,140 м/с; seed 14 прерван. В логах планировщика обнаружен `UnboundLocalError`: когда RPP принимал дугу, строка диагностики читала локальную переменную `diagnostics`, заданную только в ветке MPPI. Процесс одного контура падал, поэтому эти матчи **нельзя использовать для сравнения RPP и MPPI**. Ошибка была исправлена перед повтором серии, описанным ниже; после отката всего кандидата её кода в дереве нет.

**Повтор после исправления падения.** `results/series-20260930T055351Z/`, scenario 1, seeds 12–14, 90 с, роли чередовались: 1 цель / 1 поимка / 1 тайм-аут, 0 контактов; средние скорости исследователь/страж **0,153/0,155 м/с**, во всех шести отчётах <0,2. Сравнение с MPPI базой `series-20260929T221103Z`: те же 1/1/1, 0 контактов, скорости **0,172/0,145 м/с**. Эксперимент не дал роста обеим ролям. Логи во втором матче показывали многочисленные `rpp_collision` и `rpp_carrot_behind`, тогда как `rpp_arc` встречался единично: большая часть циклов снова выполнялась резервным MPPI. В трассах `v_curve` осталась примерно 0,1 м/с, геометрия пути менялась почти на каждом обновлении. **Переключатель RPP и его пробный модуль откатаны**, штатный MPC и основной MPPI сохранены; после отката прошли 106 тестов, `compileall`, `git diff --check`, `helm build duel`, контейнеры удалены. Новый образ после отката ещё не проверен дуэлью; доказательством работы этой конфигурации остаётся прежняя MPPI серия. Следующий шаг — проектировать пространственный локальный путь специально под вход Lat-MPC либо заменить весь контур контроля на полноценный Nav2 контроллер с одним владельцем `/cmd_vel`; не выдавать гибрид MPPI→Path→MPC за полноценный Nav2 MPPI. Требуется новая итерация с проверкой обоих роботов на тех же seed, затем длинная серия 20×360 с.

## Актуализация промпта и ревизия состояния (30.09)

По запросу пользователя обновлён `docs/DUEL_IMPROVEMENT_PROMPT.md`: приоритет — ревизия всего локального контура, исходные параметры MPC, корректная адаптация Nav2, короткие плавные пути и устранение застреваний. Временные потолки скорости и обязательный диапазон исходов 40–60% из промпта удалены. Пороги средней скорости 0,2, затем 0,3 м/с сохранены как критерии результата. Этот шаг меняет документацию; алгоритмы, launch и незакоммиченный кандидат не изменены, новые дуэли и тесты не запускались.

**Дополнение к журналу последних испытаний.** Сводки сверены с `results/series-*/summary.json` и текущим diff. Все серии ниже: scenario 1, seeds 12–14, лимит 90 с, чередование ролей, команда `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace`. Скорости указаны в порядке исследователь/страж.

| Серия | Гипотеза/изменение | Цели / поимки / тайм-ауты | Средние скорости, м/с | Контакты | Вывод |
| --- | --- | --- | --- | --- | --- |
| `series-20260929T192049Z` | База MPPI, локальный префикс 1,2 м | 1 / 1 / 1 | 0,165 / 0,177 | 0 | Скорость недостаточна |
| `series-20260929T194329Z` | PathAngle critic для уменьшения ошибки направления | 0 / 1 / 2 | 0,157 / 0,164 | 0 | Регрессия, эксперимент удалён |
| `series-20260929T195708Z` | Штраф большой ошибки начальной касательной | 2 / 0 / 1 | 0,182 / 0,179 | 0 | Кандидат остаётся незакоммиченным; преимущество обеих ролей не доказано |

Последний кандидат записан в отчёте как `4fa5e6c+dirty.da1ea2067c67`; доли движения 0,758/0,794, поворотов 0,207/0,183, доступность planner 0,935/0,959. Ни один из трёх исследователей и два из трёх стражей не выполнили 0,2 м/с. До этой серии в ходе разработки были выполнены 102 теста и сборка; это исторический результат, не новая проверка при правке промпта. Диагностическая серия `193325Z` имела сбой trace из-за отсутствующего импорта QoS; её нельзя считать полной трассировкой. После исправления диагностический seed 12 (`193559Z`) дал поимку за 19,3 с, скорости 0,165/0,227 без контактов; он не заменяет серию.

**Подтверждённые расхождения, ещё не исправленные в коде.** В `hsl_mpc.launch.py` остались overrides: CC `v_cmd_max=0,3` вместо исходных 0,5; Lat `a_lat_max=0,08` вместо 0,03, `v_min=0,12` вместо 0,1, `curve_lookahead=1,0` вместо 2,0. Поэтому прежние слова о полностью исходной конфигурации MPC и «штатном пределе 0,3» неточны: исходники подмодуля не менялись, эффективные настройки менялись. README пока описывает действующий override; после исправления запуска его надо синхронизировать. `_reference_prefix` действительно обрезает точки позади начальной оси, что может обрезать допустимый разворот с движением вперёд; влияние на застревание остаётся гипотезой. Классификация `gate_stale_*` в trace использует время callback и не задаёт явно sim time; выводы о причинах остановок требуют сверки со stamps и часами шлюза.

**Следующий шаг:** сохранить текущий кандидат, восстановить исходные настройки MPC и проверить их в обоих узлах; исправить диагностику времени; проверить обрезку дуг и привязку маршрута, согласование rollout с динамикой MPC. Сравнить результат на том же наборе условий, затем на новых seed и других сценариях. Полная серия 20×360 с и устойчивое выполнение минимальной средней скорости не подтверждены. На момент обновления duel-контейнеров нет, работают только служебные wait-init/ros-daemon. Более старые фразы ниже о том, что два полных стека ещё не запускались, относятся к историческому состоянию и не описывают текущую архитектуру.

## Повторная база и восстановление штатных параметров MPC (30.09, в работе)

**Повторная база.** Гипотеза: штраф начальной касательной, оставленный в checkpoint `1c78b1b`, неустойчив на тех же seed. Образ собран командой `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace --build`. Из-за прерывания оболочки runner записал только seed 12–13 в `results/series-20260929T213116Z/`; уже собранный неизменённый образ для seed 14 был затем запущен `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 14 --active-s 90 --scenario 1 --trace` и записан в `results/series-20260929T214053Z/`. Это три фактических матча, но **не одна непрерывная серия** и их dirty fingerprint в metadata различается из-за правок рабочих файлов после сборки; runtime параметрами подтверждён старый образ MPC. Роли: 12 — исследователь/страж, 13 — страж/исследователь, 14 — исследователь/страж. Исходы: поимка за 25,9 с, тайм-аут 90 с, поимка за 65,0 с. Скорости исследователя/стража соответственно 0,167/0,188; 0,171/0,157; 0,152/0,168 м/с; средние **0,163/0,171**. Во всех шести отчётах скорость <0,2 м/с, контактов нет. Исходы отличаются от первоначального прогона кандидата `series-20260929T195708Z` (2 цели/1 тайм-аут) при тех же seed: асинхронность/изменчивость существенно влияет на малую серию, поэтому штраф не доказал улучшение. По исправленным трассам остановки из-за большой ошибки локальной касательной наблюдаются часто; явное устаревание pose/scan на seed 12 не стало ведущей причиной. Время трассы переведено на sim time, а возраст Pose/Scan/Path/Intent считается по stamp, как в шлюзе. **102 теста прошли** до базы.

**Текущая правка, ещё без испытаний на новом образе.** В `hsl_mpc.launch.py` удалены переопределения CC `v_cmd_max=0,3` и Lat `a_lat_max=0,08`, `v_min=0,12`, `curve_lookahead=1,0`; после сборки ожидаются значения исходных YAML 0,5 / 0,03 / 0,1 / 2,0. `v_ref=0,3` теперь приходит из YAML. Шлюз больше не обрезает угловую команду MPC на ±1,2 (исходный Lat-MPC ограничивает ±1,5), но сохраняет разрешение движения, проверки свежести и поворот по ошибке локального пути. Это нужно проверить в обоих работающих узлах и дуэли. **103 теста прошли**, `git diff --check` прошёл, Compose `config --quiet` прошёл (не заданный для этого вызова `VEHICLE_ID` дал предупреждения без ошибки). README и AGENTS обновлены. Ревизия первоисточника Nav2 Humble и границы Python-адаптации записаны в [NAV2_MPPI_ADAPTATION.md](NAV2_MPPI_ADAPTATION.md); это результат чтения исходника, а не доказательство поведения робота.

**Проверка исходных параметров MPC завершена.** `helm build duel` прошёл, образ `sha256:17df6aeee95d7ee4f42d4c9de3c5d043d3e44fff6529505e104b05d3dc772c4e`. В обоих работающих узлах `ros2 param get` подтвердил CC `v_ref=0,3`, `v_cmd_max=0,5` и Lat `a_lat_max=0,03`, `v_min=0,1`, `curve_lookahead=2,0`. Дуэльная серия `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace`, `results/series-20260929T214722Z/`: seed 12 — поимка 40,1 с, seed 13 — тайм-аут, seed 14 — цель 51,9 с; контактов нет. Средняя скорость по ролям **0,184/0,157 м/с**, все шесть ниже 0,2. Доли движения 0,714/0,887, поворотов 0,256/0,096, planner OK 0,934/0,977. По сравнению с повторной базой 0,163/0,171 исследователь стал быстрее, страж медленнее; исходы также изменились. Нельзя приписать каждое изменение только параметрам из-за стохастической/асинхронной дуэли на трёх матчах, но исходный диапазон восстановлен фактически. В trace первого матча `v_curve` в конце 0,10–0,17 м/с; отдельная 12-секундная выборка на третьем матче дала медиану `v_curve=0,128`, эффективной `v_ref=0,121` м/с (73/72 сообщения). Следовательно, медленное поступательное движение согласуется с реакцией штатного MPC на кривизну короткого пути; для устойчивого вывода нужна запись распределений за весь матч. У seed 12 половина командных отсчётов имела новую геометрию пути; у исследователя 58 из 418 отсчётов — отсечение шлюзом, 78 — путь разворота; у стража отсечений и путей разворота в трассе нет, но средняя скорость только 0,159 м/с. После серии дальнейшая работа — исправить локальную геометрию и модель rollout при сохранении штатных ограничений MPC.

**Следующий шаг:** поправить пропущенное первое управление в rollout MPPI и обрезку допустимой U-дуги, добавить распределение `v_curve/v_ref` в trace, затем испытать тот же набор seed. После этого исследовать ложное продвижение при близких ветвях пути и реакцию MPC на смену геометрии. Полная серия 20×360 с и порог скорости не подтверждены.

## Ревизия rollout и ложного продвижения (30.09, в работе)

**Сдвиг управления и допустимый разворот.** В исходной адаптации MPPI первый элемент sampled controls не влиял на траекторию: после измеренного первого шага применялся сразу второй элемент. Исправлено соответствие с [Nav2 Humble motion model](NAV2_MPPI_ADAPTATION.md). Из `_reference_prefix` удалено обрезание дуги по проекции её конца на начальную ось: допустимый разворот с положительной поступательной скоростью может закончиться позади исходного курса. Длина опубликованной дуги по-прежнему ограничена 1,2 м; полная траектория и её сегменты проходят collision check. Добавлены тесты явной последовательности управления и U-дуги. **105 тестов, compileall и `git diff --check` прошли; `helm build duel` прошёл.** Серия `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace`, scenario 1/seeds 12–14, `results/series-20260929T215835Z/`: цель исследователя на seed 12 за 50,7 с, seed 13–14 тайм-ауты, поимок нет, контактов нет; скорости **0,168/0,141 м/с**, доля движения 0,778/0,928, доля поворота 0,202/0,056, planner OK 0,954/0,981. Предыдущая серия при исходных настройках MPC: 1 цель/1 поимка/1 тайм-аут, 0,184/0,157 м/с. Следовательно, пара правок не дала улучшения и пока является **регрессией обеих ролей**, хотя устраняет проверенные геометрические ошибки. Заново замеряем штатное замедление: в seed 12 медиана `v_curve=0,10` у исследователя, 0,118 у стража; ниже 0,2 в 65,7%/92,7% выборок. Меняется почти каждый опубликованный путь. Независимая 12-секундная выборка 38 локальных путей дала медиану максимальной кривизны 13,54 1/м, p90 176,36 1/м; у нескольких троек точек соседние шаги были лишь 0,0001–0,015 м. Это согласуется с падением скорости в исходном MPC и указывает на необходимость нормализовать геометрию публикуемого пути, не отключая защиту.

**Ложная проекция на петлю.** В `_pruned_route` ограничен поиск начальной привязки, однако `_project_batch` до сих пор искал ближайший сегмент по всей петле. Контрольный маршрут `[(0,0),(1,0),(1,1),(0,1),(0,0.1),(-1,0.1)]` при позе `(0.02,0.08)` давал ложный прогресс **3,9 м**, хотя робот почти не сдвинулся. Текущий незаконченный кандидат ограничивает проецируемый участок накопленной длиной rollout плюс 0,35 м; применено и к финальному отбору образца. **106 тестов и compileall прошли**. Дуэли на этом кандидате ещё не проводились. Следующий шаг — сборка и серия на тех же seed, затем исправление слишком плотных и неровных точек локального `Path` с повторной проверкой всей траектории по препятствиям.

**Проверка ограничения проекции.** После фикса `_project_batch` учитывает фактически пройденную длину rollout +0,35 м и в промежуточной оценке, и при выборе итоговой траектории. **106 тестов, compileall, `git diff --check` и `helm build duel` прошли.** Серия `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace`, scenario 1/seeds 12–14, `results/series-20260929T221103Z/`: цель исследователя за 55,1 с, тайм-аут, поимка за 25,2 с; контактов 0; скорости **0,172/0,145 м/с**, доли движения 0,749/0,919, поворотов 0,215/0,056. К предыдущему варианту вернулась поимка, но скорости обеих ролей ниже и порог 0,2 не достигнут ни разу. Исправление оставлено из-за доказанной ошибки ложного прогресса, статистическая польза для исходов не доказана. Новая трасса первого seed: медиана максимальной кривизны опубликованного пути 8,24/5,59 1/м (исследователь/страж), p90 77,78/63,57; медианная доля отрезков <2 см 0,30/0,15; `v_curve` ниже 0,2 м/с в 71%/89,8% отсчётов. Следующая гипотеза: слишком плотные точки создают всплески кривизны для штатного Lat-MPC.

**Кандидат регуляризации геометрии, ещё без дуэли.** После короткого безопасного префикса локального пути отбрасываются почти совпадающие точки с шагом 0,06 м. Полученные хорды дополнительно проверяются через `_safe_path` с теми же препятствиями и запасом; при отказе остаётся исходный плотный путь. Продвижение и тёплый старт MPPI по-прежнему используют полную траекторию. Тест проверяет устранение искусственного всплеска кривизны, **107 тестов и compileall прошли**. Нужны сборка и дуэль на тех же seed; нельзя считать геометрическую проверку доказательством фактической плавности или безопасности.


## Отказ от прореживания точек после дуэлей (30.09)

Серия на checkpoint `7cfd302` с `_spaced_reference`: `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace`, scenario 1/seeds 12–14, `results/series-20260929T222148Z/`, образ `sha256:5a5c0d4d3abd6f1f761c734f304fa8a8957d3d0205ae617924827cccecdeace0`. Исходы — **три поимки**, целей исследователя нет; скорости исследователь/страж **0,170/0,160 м/с**, во всех шести отчётах <0,2. Исследователь на seed 12 дважды коснулся стены в `(−1,05; 1,40)` на 72,2/73,7 с; у стража контактов нет. Это регрессия по сравнению с предыдущей серией `series-20260929T221103Z` (1 цель/1 поимка/1 тайм-аут, 0 контактов, 0,172/0,145 м/с). Геометрическая метрика действительно улучшилась: на seed 12 медианная максимальная кривизна пути 8,24→4,75 1/м у исследователя и 5,59→2,90 у стража; медианная доля сегментов <2 см стала 0 у обоих. Однако `v_curve` у обоих осталась примерно 0,1 м/с, контакты появились и исследователь перестал выполнять задачу. Следовательно, прореживание точек скрывает часть высокочастотной геометрии, но не решает низкую скорость и ухудшает следование в узком месте. **Вызов `_spaced_reference`, сама функция и её тест удалены**; проверка исходной плотной траектории остаётся. Контейнеры серии после завершения удалены `helm clean duel`. Следующая гипотеза: MPPI семплирует скорости и повороты, недостижимые для штатного MPC в ближайший момент; нужно согласовать модель разгона и предел собственного контроллера с его исходными YAML, не меняя ограничения выходных команд и не предполагая скорость соперника. После исправления нужны тесты, сборка и новые дуэли.

## Адаптация локального планировщика Nav2 MPPI (29.09, в работе)

**Исходники и граница адаптации.** Изучены официальные исходники Nav2 Humble: `nav2_mppi_controller/src/optimizer.cpp`, `path_follow_critic.cpp`, `path_align_critic.cpp`, `obstacles_critic.cpp` в репозитории [navigation2](https://github.com/ros-navigation/navigation2/tree/humble/nav2_mppi_controller). MPPI в Nav2 является контроллером и возвращает `cmd_vel`; у нас сохранён штатный MPC, поэтому в `hsl_planning/mppi.py` перенесён принцип локальной оптимизации: шумовые последовательности управлений → модель движения → оценки следования глобальному пути, прогресса, препятствий и гладкости → softmax-обновление → проверенная на столкновения траектория `Path`. Глобальный A* и модуль `mpc_motion_control` не изменены. Искусственные ограничения скорости для этапа исследования не добавлялись. Это адаптация алгоритма, не запуск исходного C++ плагина Nav2.

**Предшествующая проба порога выхода из шлюза (откачена).** Изменение порога гистерезиса `0,75→0,55` проверено на сценарии 1, seed 12–14, 90 с, `results/series-20260929T163143Z/`: 3 поимки / 0 целей, 0 контактов, средние скорости 0,167/0,212 м/с; исследователь ниже 0,2 во всех трёх матчах. Такой перекос роли и скорость не подходят. Изменение возвращено к коммиту `ba7a3a9` перед работой над MPPI.

**MPPI, первая дуэльная серия.** Гипотеза: семплирование и проверка локальных траекторий устранят ломаное следование RPP. Изменение: MPPI с горизонтом 2,4 с, 256 траекториями, 2 итерациями, `dt=0,15`; выход — локальный `Path` для прежнего MPC. Команды: `python3 -m pytest -q tests/test_navigation.py helm_launch/tests/tests.py` (**99 passed**), `helm build duel` (успешно), `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace`. Сценарий 1, seed 12–14, роли чередуются; результаты `results/series-20260929T181326Z/`: seed 12 — цель за 81,3 с, seed 13 и 14 — тайм-ауты, поимок нет, контактов нет. Средняя скорость за матч по ролям **0,148/0,148 м/с**, доля поворота на месте 0,293/0,285, `planner_ok_fraction` 0,948/0,947. Средний p95 вычисления планировщика 270/294 мс; локальный путь по трассам имеет медианную длину 0,398–0,552 м, команда шлюза отсекалась в 0,238–0,355 отсчётов. До адаптации RPP на тех же seed давал 2 цели / 1 поимку, 0 контактов, 0,194/0,210 м/с (`results/series-20260929T162119Z/`). **Вывод: первая адаптация MPPI — регрессия; критерии скорости и обеих ролей не достигнуты.** После серии выполнено `helm clean duel`.

**MPPI, вторая дуэльная серия.** Гипотеза: больший горизонт и более устойчивое MPPI-усреднение повысят прогресс. Изменение: горизонт 3,0 с, batch 192, повышен вес продвижения, добавлены безопасные шаблоны поворота перед движением, предпочтение softmax-усреднённой последовательности с проверкой её траектории. Команды: те же **99 тестов**, `compileall`, `git diff --check`, `helm build duel`, затем `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace`; результаты `results/series-20260929T182736Z/`. Seed 12 — поимка за 44,6 с; seed 13–14 — тайм-ауты, цели исследователя нет; контактов 0. Скорости исследователь/страж **0,152/0,156 м/с** против 0,148/0,148 в первом MPPI-варианте; средний `planner_ok_fraction` 0,929/0,937 против 0,948/0,947. На seed 12 p95 расчёта упал с 249/289 до 161/171 мс, но у обоих есть периоды `RECOVERY_ESCAPE` (4,6–8,9% времени по шести роботам). Вывод: вычисление ускорено, поимка появилась, но цели нет и порог скорости не достигнут. После серии — `helm clean duel`.

**Исправление трактовки трассы.** В `benchmarks/trace_motion.py` длина пути и ошибка курса до сих пор записывались только когда `/cmd_vel.linear.x ≤ 0,02`. Значения 0,398–0,552 м и большие ошибки из первой серии относятся к остановкам, часто к recovery, и не являются статистикой всех локальных путей. Для следующего цикла нужно считать геометрию каждого опубликованного `Path` и отдельно анализировать отсечения шлюза (0,233–0,407 во второй серии). Проверка `path_turning_decision` использует хорду всего пути, если отклонение кривой меньше 0,035 м; это может делать лишние развороты на коротких дугах. Следующее изменение — оценивать начальную касательную и проверить с теми же seed. Запас до стены MPPI сейчас больше прежнего RPP на 0,06 м; уменьшать его только с повторной проверкой контактов.

**MPPI и проверка касательной, третья серия.** Гипотеза: шлюз останавливал робота по направлению хорды короткой кривой, когда начальная касательная допускала движение. В `path_turning_decision` направление теперь считается по ближайшим точкам пути; добавлен регрессионный тест короткой дуги. Удалена добавочная зависимость скорости шлюза от расстояния до конца локального `Path`: предел поступательной команды оставлен штатному MPC. В трассе теперь отдельно записываются геометрия и курс **каждого** опубликованного пути. Проверки: **100 passed**, `compileall`, `git diff --check`, `helm build duel`; затем та же команда серии, результаты `results/series-20260929T184043Z/`. Seed 12 и 13 — поимки за 26,9 и 15,3 с; seed 14 — тайм-аут, целей нет; контактов 0. Скорости исследователь/страж **0,154/0,205 м/с** против 0,152/0,156 во второй MPPI-серии; доля поворота **0,422/0,195** против 0,288/0,259. Страж улучшился, но исследователь ещё чаще разворачивается и ниже 0,2 во всех трёх матчах. У него доля отсечения шлюзом в среднем 0,470; у стража 0,240. Медиана длины всех локальных путей у исследователя около 0,55 м, медианы ошибки начальной касательной 0,568–0,910 рад. Логи третьего матча показывают повторные `no_safe_sample` и четырёхсекундные остановки с recovery. Вывод: проверка касательной полезна для стража, но слабое место исследователя — отсутствие безопасных образцов в тесных местах. После серии — `helm clean duel`.

**MPPI с прежним запасом до стены, четвёртая серия.** Гипотеза: дополнительный запас 0,06 м вызывал ложные отказы MPPI в узких местах. Удалена только добавка в вызове MPPI; остались `local_safety_margin=0,14/0,12 м` для исследователя/стража и проверка каждой дуги на swept-footprint. Проверки: **100 passed**, `compileall`, `git diff --check`, `helm build duel`; та же команда трёх заездов, `results/series-20260929T185009Z/`. Seed 12 — поимка за 11,0 с; seed 13 — тайм-аут; seed 14 — цель за 22,7 с. Контактов **0**. Средние скорости исследователь/страж **0,181/0,211 м/с** против 0,154/0,205 в предыдущем варианте; доля поворота 0,273/0,151 против 0,422/0,195. В seed 14 оба достигли ~0,22 м/с. Это лучшая MPPI-серия, но исследователь ниже 0,2 в 2/3 матчей, есть тайм-аут, и улучшение на трёх seed недостаточно для вывода о стабильности. Логи seed 13 всё ещё содержат `no_safe_sample` и пути без продвижения; одного снижения запаса недостаточно. После серии — `helm clean duel`.

**Диагностика после замечания пользователя о длинной и обратной локальной траектории.** На работающей дуэли глобальный маршрут был 5,26 м. В 10-секундном окне локальный `Path` достигал 1,50–1,53 м; его конец оказывался позади текущего курса в 6/27 обновлений у исследователя и 11/27 у стража. Встречались как оптимизированные MPPI-пути, так и прямые recovery-пути назад. Поэтому `_pruned_route` теперь ищет проекцию только на первом метре уже привязанного глобального маршрута; в контроллер публикуется короткий безопасный префикс оптимизированной дуги, тогда как весь горизонт остаётся для тёплого старта MPPI. Recovery сперва ищет проверенную короткую дугу и только при отказе использует прямой уход либо разворот на месте. При `no_safe_sample` добавлена повторная оптимизация с горизонтом 1,65 с. Для проверки в трассу добавлены длина каждого опубликованного пути и доля концов позади курса.

**Проверка префикса 1,0 м:** `101 passed`, `compileall`, `git diff --check`, `helm build duel`; сценарий 1, seed 12–14, 90 с, `results/series-20260929T191147Z/`. Две поимки и тайм-аут, целей нет, контактов 0. Средние скорости исследователь/страж **0,162/0,166 м/с**, поворот на месте 0,249/0,202, `planner_ok` 0,953/0,950. Максимальная опубликованная локальная длина в шести трассах ≤1,0 м; доля концов позади текущего курса 0–0,033. Геометрический дефект существенно уменьшен, но скорость и успех исследователя ухудшились относительно четвёртой серии (1 цель/1 поимка/1 тайм-аут, 0,181/0,211 м/с). Контроллер смотрит около 1 м вперёд; возможно, публикация ровно 1 м лишает его запаса. Следующая изолированная проба — префикс 1,2 м при тех же критериях безопасности, с повторной Gazebo-проверкой. Более длинный горизонт 3 с и префикс — разные величины.

**Префикс 1,2 м оставлен как текущий вариант:** те же проверки (101 passed, `git diff --check`, `helm build duel`) и сценарий 1, seed 12–14, 90 с, `results/series-20260929T192049Z/`. Seed 12 — тайм-аут; seed 13 — цель исследователя за 63,0 с; seed 14 — поимка за 62,0 с. Столкновений 0. Средние скорости **0,165/0,177 м/с**, доля поворота на месте 0,251/0,129, `planner_ok` 0,954/0,968. Все шесть трасс имеют максимальную длину локального пути ≤1,2 м и долю концов позади текущего курса 0,003–0,011. Относительно 1,0 м восстановился успех исследователя и улучшились скорость/поворот стража, при том что локальный путь остаётся короче наблюдавшихся прежде 1,5 м. Однако это всё ещё хуже по скорости, чем четвёртая MPPI-серия с длинным локальным путём; 0,2 м/с не достигнуто ни одним роботом ни в одном матче. Нужна отдельная работа над частыми остановками/поворотами, а не ещё одно увеличение длины пути. После серии выполнено `helm clean duel`.

**Прерванная диагностическая проба короткого горизонта:** `results/series-20260929T190020Z/`, только seed 12 из трёх (серия остановлена после нового замечания пользователя): тайм-аут, 0 контактов, скорости 0,153/0,175 м/с. Этот одиночный результат не подтверждает улучшения. Доработка короткого retry осталась в текущем коде и требует дальнейшей проверки.

**Следующий цикл.** На префиксе 1,2 м разобрать причины остановок и поворотов в трассах (`gate_clipped` 0,096–0,221, `RECOVERY_ESCAPE` до 0,05), затем уменьшать застревания/recovery и добиваться гладкости и средней скорости ≥0,2 м/с у обеих ролей. После локальной стабилизации нужны другие сценарии и полная серия 20×360 с; нынешние три заезда не подтверждают конечную цель.

**Критическая правка источника наблюдений (29.09):** найдено несоответствие автономной дуэли требованиям: адаптер подтверждал видимость соперника по LiDAR, но затем подменял наблюдение точными координатами из Gazebo и скоростью по simulator odometry. Удалена эта утечка из навигационного контура обоих роботов. Теперь адаптер оценивает центр стража/исследователя по кластерам динамических LiDAR-точек, отфильтровывает собственную поверхность и статические попадания карты, проверяет прямую видимость по карте, а скорость получает из последовательных оценок центра. Simulator ground truth остаётся у собственного симуляционного положения и referee/metrics; в decision manager он для соперника не передаётся. Добавлены тесты кластеризации/оценки центра, фильтрации стен и закрытой видимости; `python3 -m pytest -q tests/test_navigation.py helm_launch/tests/tests.py` — **117 passed**. `helm build duel` и `helm up duel` успешны. Runtime smoke test на Gazebo: оба флага движения до старта были `false`, в движении видимость LiDAR была `true`, опубликована оценка соперника; короткий ручной прогон ~19 с сим-времени завершился без wall/robot contacts, `planner_ok=0,985/0,974`, средние скорости explorer/guardian 0,201/0,181 м/с. Результаты: `results/duel-20260929T124225686012Z.json` и `results/opponent/duel-20260929T124228005784Z.json`. Матч остановлен; после остановки оба motion-флага снова `false`. Это проверка запуска и наблюдений, без goal/capture и без оценки баланса. **Все предыдущие дуэльные серии использовали точное положение соперника из симулятора и поэтому не засчитываются как результат автономной проверки; их нужно повторить с LiDAR-only наблюдениями.**

**Первая автономная серия с LiDAR-only соперником:** сценарий 1, seeds 12–14, лимит 90 с, трассы — `results/series-20260929T124355Z/`. Все 3 матча завершились достижением цели исследователем, поимок нет, так что работа стража пока не подтверждена. Средние скорости explorer/guardian 0,158/0,171 м/с; ниже 0,2 были 3/3 и 2/3 матчей соответственно. Столкновений было одно: страж со стеной в seed 14 около (0,745; 0,399), на 35,4 с. В двух остальных матчах контактов не было. `planner_ok` в среднем 0,974/0,980, но кривые приняты редко (по role-trace 0,119/0,085); доля обрезанных шлюзом команд 0,345/0,334. Результат не проходит порог скорости и запрет контактов. Следующий шаг — проверить локальный запас до стены и отдельно разобраться, почему LiDAR-оценка/перехват не дают стражу сойтись с исследователем; полную 20×360 с серию не запускать.

**Упреждение поиска после потери LiDAR-видимости:** когда цель ненадолго пропадает за стеной, guardian теперь переносит последнюю измеренную позицию вперёд по последней измеренной скорости. Горизонт ограничен одной секундой после тайм-аута обнаружения; скорость соперника не ограничивается предполагаемым верхним пределом. Тесты decision/planning/helm — **118 passed**, `py_compile`, `git diff --check`, `helm build duel` успешны. Повтор сценария 1 на тех же seed 12–14 (`results/series-20260929T130627Z/`) дал **2 guardian_capture / 1 explorer_goal и 0 контактов**, против 0/3 поимок и одного контакта в предыдущей LiDAR-only серии. Скорости explorer/guardian — 0,160/0,184 м/с; ниже 0,2 остались 3/3 и 2/3. На guardian seed 12 доля `SEARCH` снизилась 0,343→0,115, `PURSUE` выросла 0,530→0,793; seed 13 завершился поимкой за 27,9 с, seed 14 — за 53,0 с. Это обнадёживающая, но короткая трёхматчевая проверка, не доказательство 50/50 или требуемой скорости. Пассивная truth-сверка LiDAR-оценок в трассировщике (только диагностика, не вход управления) дала средний по матчам p90 ошибки позиции 0,150 м для explorer и 0,142 м для guardian. Для локального управления остаётся частое ограничение MPC-шлюзом (среднее 0,343/0,250) и редкое принятие кривых (0,106/0,107); следующий цикл — плавность и прирост средней скорости без потери нулевых контактов.

**Уточнение пользователя (28.09):** сначала добиться уверенного движения без столкновений и изломов при минимальной **средней скорости за матч 0,2 м/с**, затем поднять нижний порог до **0,3 м/с**. Эти значения не ограничивают максимальную команду: впоследствии она может достигать 1 м/с, только если движение остаётся безопасным и плавным. Проба командного предела 0,4 м/с дала один стеновой контакт и отклонена; исходники возвращены к пределу 0,3 м/с, образ пересобран и проверен коротким матчем. Промежуточный порог средней пока не подтверждён; результаты испытания приведены ниже.

**Последняя итерация — проверка горизонта перехвата:** временно увеличил горизонт ограниченной экстраполяции измеренного движения исследователя с 2,0 до 2,5 с. Это не вводило предположения о максимальной скорости соперника. Сценарий 1, seed 12–14, 90 с: `results/series-20260929T112642Z/`; 3 достижения цели, 0 поимок и 0 контактов. Средняя скорость исследователь/страж — 0,166/0,161 м/с против 0,162/0,191 м/с в предыдущем запуске с горизонтом 2 с (`results/series-20260929T111852Z/`). У стража доля поступательного движения упала 0,755→0,666, доля поворотов выросла 0,228→0,321; у исследователя изменения малы. Проба отвергнута, горизонт возвращён к 2 с. Данные показывают, что увеличение упреждения здесь ухудшило движение стража и не принесло поимок; далее нужно разбирать выбор доступной точки сближения и причины лишних поворотов. Тесты после отката и запись этой итерации — в текущем изменении.

**Расширен подбор формы безопасной кривой:** в диагностике предыдущих прогонов локальный planner редко принимал кривую; наиболее частыми причинами отказа были `route_curve_clearance` и `route_handle_curve_clearance`, при этом пустые/устаревшие пути были редки. После трёх прежних вариантов ручки Безье добавлены две крайние длины. Они пробуются только после отказа по clearance/кривизне; предел кривизны 2,6 1/м и требуемый запас до стены/границы `robot_radius + 0,18 м` сохранены, каждое звено по-прежнему проходит swept-check. Новый регрессионный случай показывает безопасную дугу, которую прежние ручки не находили. **116 тестов** прошли, `git diff --check`, `helm build duel` и `docker compose --profile duel config --services` успешны (Compose сообщает только о незаданном `VEHICLE_ID`).

Проверка сценария 1, seed 12–14, 90 с, `results/series-20260929T114456Z/`: 2 достижения цели/1 поимка, **0 столкновений**, скорости исследователь/страж 0,186/0,208 м/с; доли движения 0,764/0,841, поворотов 0,224/0,146, `planner_ok=0,974/0,974`. Контроль на прежней версии без крайних ручек (`results/series-20260929T111852Z/`) дал 3 цели/0 поимок, скорости 0,162/0,191 м/с и повороты 0,290/0,228. Затем сценарий 2, seed 3–5, `results/series-20260929T115357Z/`: 1 цель/2 поимки, **0 столкновений**, скорости 0,168/0,175 м/с, повороты 0,300/0,201, `planner_ok=0,976/0,964`. Контроль тех же seed (`results/series-20260929T110912Z/`): 2 цели/1 поимка, скорости 0,155/0,155 м/с, повороты 0,340/0,341, `planner_ok=0,956/0,979`, тоже без столкновений. Среди шести новых матчей исходы ровно 3/3; средние по двум сценариям скорости всё ещё только 0,177/0,192 м/с, а ниже 0,2 остались 5/6 заездов исследователя и 4/6 стража. Кандидат оставлен: скорость и доля поворотов улучшились в обеих парах серий, а контактов нет, но это короткие заезды и не подтверждение скорости/баланса на длинной выборке. В трассах доля кривых колеблется 0–0,222; принятые запасные формы встречались в логах стража, значит проблема редких кривых не устранена полностью. Runtime-граф после сборки подтверждает два отдельных decision manager, планировщика, MPC gate и наблюдателя метрик; у `/cmd_vel` и `/opponent/cmd_vel` по одному издателю, scripted patrol отсутствует. Следующий цикл — проверить, где сглаживается скорость исследователя (особенно `moving_speed_mps` и clipping MPC/gate), сохранив запрет столкновений; оценочную серию 20×360 с пока не запускать до достижения рабочего минимума 0,2 м/с.

**Проверка дальности локальной кривой — расширение всем роботам и исследователю отклонено:** после высокой доли gate-clipping в scenario2 seed 3–5 проверено увеличить поиск точки кривой с 1,8 до 2,2 м для всех режимов. Серия `results/series-20260929T120801Z/`: 2 цели/1 поимка, 0 контактов; скорости 0,180/0,168 м/с, доли движения 0,706/0,720, поворотов 0,265/0,253, `planner_ok=0,959/0,968`. Контроль перед изменением, где 2,2 м разрешалось только стражу в `PURSUE` (`results/series-20260929T115357Z/`), дал 1 цель/2 поимки, 0 контактов, скорости 0,168/0,175, повороты 0,300/0,201, `planner_ok=0,976/0,964`. Более длинный поиск улучшил исследователя, но ухудшил скорость, повороты и поимки стража; в двух трассах стража кривые не возникали вовсе.

Отдельная проба «2,2 м исследователю и `PURSUE`, 1,8 м стражу в `SEARCH/CAPTURE`», `results/series-20260929T121637Z/`, дала 3 поимки/0 целей, 0 контактов; скорость исследователь/страж 0,152/0,177 м/с, движение 0,579/0,733, повороты 0,390/0,235, `planner_ok=0,959/0,970`. Шлюз исследователя отсекал до 51% команд, а кривые использовались не более чем в 3,4% обновлений в двух матчах. Проба также отвергнута. Возвращена проверенная конфигурация: 2,2 м только стражу в `PURSUE`, 1,8 м в остальных режимах. Обе пробы показывают, что увеличение дальности само по себе не создаёт доступную плавную дугу при большом угле или тесном проходе. Следующий цикл — менять генерацию локального пути/перехода курса, сохраняя swept-clearance, и измерить отдельно время поворота и gate-clipping. Серию 20×360 с не запускать, пока обе роли не выполняют рабочий минимум средней скорости 0,2 м/с.

**Текущая итерация:** по новому замечанию пользователя локальный путь должен проходить обычные повороты плавной кривой с одновременными поступательной и угловой скоростями; поворот на месте остаётся для тесных и небезопасных случаев. Также замечено кружение стража около исследователя в тупике; требуется исключить недоступную точку перехвата/поимки. Базовая версия со стабильной прямой локальной геометрией испытана в `results/series-20260928T161253Z/` (сценарий 2, seed 0–2, 90 с, трассы): 1 цель/2 поимки, 0 контактов, средние скорости исследователя/стража 0,213/0,168 м/с. Доли поворота на месте 0,187/0,264; обе роли не выполнили порог 0,2 в каждом матче. Новая версия строит ограниченную по кривизне и проверенную на препятствия кубическую кривую от текущего курса к видимому коридору, сохраняет её геометрию между циклами MPC и вычисляет разрешение шлюза по локальной касательной; 95 локальных тестов прошли. Gazebo-серия `results/series-20260928T162223Z/` на **прежнем лабиринте**: 1 цель/2 поимки, 0 контактов, средние 0,217/0,183 м/с; доли поворота 0,192/0,238. У всех трёх исследователей средняя ≥0,2, у одного стража — ниже. Результат слегка лучше базы, но пока не подтверждает цель во всех сценариях.

**Новый лабиринт по запросу пользователя:** в `maze.world` разделены три внутренние стены, открыты проходы шириной 1 м на x≈−1, x≈0 и y≈2 справа; короткая перегородка у левого старта удалена для второго выхода из этой зоны. Изменены и состояние SDF, и определения коллизий/визуалов, поэтому аналитическая карта и физика Gazebo совпадают. XML разбирается, по геометрии минимальный зазор от оси каждого нового прохода до стен 0,477 м, все 4 стартовые позиции имеют зазор >0,25 м. Образ `duel` пересобран; в Gazebo до удаления короткой перегородки подтверждены все шесть новых link и collision, в ROS `/map` центры трёх проходов имеют 0 (свободно), точки оставшихся фрагментов — 100 (занято). Серия `results/series-20260928T163213Z/` (сценарий 2, seed 0–2, 90 с, трассы) дала 2 цели/1 поимку и **0 контактов**; средние скорости 0,185/0,191 м/с, ниже порога 0,2. По трассе в seed 0 шлюз обнулял 37,6% активных команд исследователя при доступном MPC, средний выход на колёса 0,160 м/с. Это новая геометрия, напрямую сравнивать старые скорости нельзя. Дополнительный заезд сценария 3 до удаления короткой перегородки (`results/series-20260928T163718Z/`, seed 20, 90 с) дал цель за 75,8 с, 0 контактов, скорости 0,182/0,184 м/с. После удаления перегородки образ пересобран, Gazebo больше не содержит `Wall_14`, в ROS `/map` новая точка выхода (−2; 2,5) свободна (0), прежние фрагменты стен заняты (100). Повторный сценарий 3: `results/series-20260928T164402Z/`, seed 20, 90 с, трассы: поимка за 23,4 с, **0 контактов**, скорости 0,132/0,198 м/с. У исследователя поворот без поступательного движения занял 47,2% матча; шлюз обнулил 126 из 250 активных команд при доступном MPC. Значит геометрия проходов работает, но плавность локального управления остаётся недостаточной.

Геометрический поиск пути на сетке 0,1 м с зазором корпуса 0,28 м показывает, что после всех четырёх проходов кратчайший путь из левого старта стража (−2,5; 1,5) к (−0,5; 2,5) сократился с 4,45 до 2,77 м, а путь от старта исследователя к старту стража в сценарии 1 — с 6,23 до 4,57 м. Эти числа проверяют связность лабиринта, а не фактическое движение MPC.

**Страж у стены:** из `DecisionPolicy` убран собственный `STOP` по одной только близости и курсу: соперник может находиться за стеной, а исход подтверждает судья с проверкой видимости. `capture_goal` теперь отбрасывает точки у стены/вне арены и точки без маршрута A*, а среди оставшихся выбирает кратчайший доступный подход. Это должно уменьшить кружение около недоступной точки поимки; 95 локальных тестов прошли, контрольная серия на новом мире идёт. Отдельный регрессионный тест проверяет, что близость 0,4 м сама по себе не останавливает стража.

**Проба более плавного поворота до расширенной проверки:** после серии на окончательном лабиринте `results/series-20260928T164402Z/` увеличен разрешённый предел кривизны проверенной локальной кривой с 1,15 до 2,6 1/м; для MPC предел бокового ускорения уменьшен с 0,12 до 0,08 м/с² и минимальная скорость на повороте с 0,2 до 0,12 м/с. Это позволяет строить кривые в более коротких коридорах, сохраняя проверку коллизий и разворот на месте при тесном повороте. Трасса теперь считает долю публикуемых кривых. 96 локальных тестов прошли. Первый парный заезд на том же лабиринте и seed 20 (`results/series-20260928T165026Z/`, сценарий 3) дал цель через 58,5 с, 0 контактов, скорости 0,212/0,219 м/с против 0,132/0,198 в единственном прежнем заезде; доля поворота на месте исследователя снизилась с 0,472 до 0,216, у стража с 0,200 до 0,140. Кривые использовались в 13,7%/6,8% обновлений пути. Серия сценария 2 `results/series-20260928T165302Z/` (seed 0–2, 90 с) дала **0 контактов**, средние 0,219/0,210 м/с и доли поворота 0,184/0,171; первый заезд имел долю кривых 17,5%/27,9%. Все три раза выиграл исследователь, значит баланс ролей на этом сценарии не достигнут. Порог 0,2 выполнен всеми исследователями и двумя из трёх стражей; более сильный вывод требует длинной серии. Проверка сценария 1 `results/series-20260928T165847Z/` идёт.

**Критический результат и текущая правка:** серия сценария 1 `results/series-20260928T165847Z/` (seed 10–12, 90 с) дала 2 поимки/1 тайм-аут, средние 0,190/0,220 м/с, но в seed 12 у исследователя **один контакт со стеной** при (0,139; 0,671) на 5,7 с. Значит вариант с кривизной 2,6 и прежним зазором **не прошёл проверку безопасности**. Для движущейся кривой добавлен отдельный запас до сырой стены и границы карты `robot_radius + 0,18 м` на старте и по всем точкам; прямая с разворотом остаётся запасным вариантом. Локальный тест узкого коридора и всего 97 тестов прошли. Нужна пересборка и повтор seed 12; если контакт останется, откатить расширенную кривизну/параметры MPC.

**Повтор после увеличения запаса:** образ пересобран, `results/series-20260928T171027Z/` (сценарий 1, seed 12, 90 с) завершился достижением цели за 53,7 с, **0 контактов**, скорости 0,213/0,184 м/с. Одинаковый seed не гарантирует идентичную физическую дуэль из-за асинхронности Gazebo/ROS, поэтому это подтверждает только отсутствие контакта в одном повторе, а не полное устранение риска. У стража скорость в этом повторе ниже порога 0,2. Дополнительная серия `results/series-20260928T171311Z/` (сценарий 1, seed 12–14, 90 с) дала 2 поимки/1 тайм-аут, **0 контактов во всех шести отчётах**, средние 0,196/0,211 м/с. Только один короткий матч исследователя ниже 0,2; у стража все три не ниже 0,2. Итого после изменения зазора четыре независимых заезда сценария 1 без контактов, но 20×360 с по разным сценариям ещё не выполнены. Следующие шаги: отдельно проверить, не скребёт ли прямой локальный путь у стены, и дорабатывать доступность точки перехвата.

**Проверка точки перехвата:** прогнозная цель поведения `PURSUE` теперь проверяется на нахождение внутри арены, занятость и возможность движения соперника от наблюдаемой позиции к прогнозу без пересечения известной стены. Если прогноз ведёт сквозь стену, планировщик выбирает достижимую позицию сближения около видимого соперника через `capture_goal`; если такую найти не удаётся, прежний механизм выбора доступного frontier остаётся запасным. Это использует только измеренную скорость соперника, без допущения о её верхней границе. Синтетический тест стены и все 98 тестов прошли, образ пересобран. Серия сценария 1 `results/series-20260928T172508Z/` (seed 10–12, 90 с) дала 2 поимки/1 тайм-аут, **0 контактов**, средние 0,182/0,211 м/с. У стража 95-й процентиль цикла планировщика в первом матче 125 мс, 0 превышений 200 мс; у исследователя было одно превышение. Серия сценария 2 `results/series-20260928T173207Z/` (seed 0–2, 90 с) дала 2 цели/1 поимку, **0 контактов**, средние 0,182/0,193 м/с. Вместе по двум сценариям 3 цели/3 поимки и 1 тайм-аут, но выборка слишком мала для критерия 50/50, многие отдельные матчи ниже 0,2 м/с, а 20 полных матчей по 360 с не выполнены. Эффект на долю поимок относительно прежней версии не доказан; пока изменение оставлено ради запрета физически невозможной точки за стеной. Для дальнейшей работы нужны разбор причин частых доворотов и испытания длинных матчей на разных сценариях.

**Прерванная оценочная серия и новая гипотеза:** `python3 benchmarks/run_duel_series.py --runs 20 --start-seed 0 --active-s 360 --scenario 2` началась в `results/series-20260928T173828Z/`. После пяти **полностью завершённых** матчей (seed 0–4) зафиксированы 3 цели/2 поимки, **0 контактов**, средние скорости 0,192/0,197 м/с; `summary-partial.json` отдельно указывает, что это не 20-матчевая итоговая серия. Шестой матч остановлен безопасной командой `helm stop_match`, контейнеры удалены через `helm clean duel`. В длинном пятом матче страж догнал исследователя лишь на 184,9 с при средней 0,167 м/с; пассивная трасса в конце показывает 88 из 96 команд с поступательным движением, но средний `cmd_vel.linear.x` всего 0,150 м/с. Причина локализована в законе `approach_speed`: при дистанции 0,6 м страж запрашивал около 0,137 м/с и мог не догнать движущегося соперника. Проба сопоставила скорость с измеренной радиальной скоростью убегания соперника и увеличила темп сближения, ограничивая его в пределах 0,5 м; верхняя скорость соперника не предполагалась. 99 локальных тестов прошли, но Gazebo-серия `results/series-20260928T175434Z/` (сценарий 2, seed 0–2, лимит 210 с) выявила **физический контакт обоих роботов при поимке в seed 2**. Среди трёх завершённых матчей 2 цели/1 поимка, средние 0,202/0,192 м/с; `summary-partial.json` помечает прерывание перед четвёртым матчем. Проба **полностью отклонена**: исходник закона скорости и его тест возвращены к коммиту `0d43339`, контейнеры удалены. Следующий шаг — учитывать тормозной путь и текущую относительную скорость без роста контактов; до этого не считать новую полную серию пройденной.

**Вторая проба скорости поимки, пока без Gazebo:** вместо линейного `approach_speed` используется профиль `sqrt(2 * 0,25 * max(0, distance − 0,48))` с минимальным ползучим ходом 0,08 м/с и прежними верхними ограничениями. Он основан на собственной возможности торможения MPC (`a_min = −0,5 м/с²`) с двукратным запасом, а не на максимальной скорости соперника: около 0,245 м/с при 0,6 м, 0,10 при 0,5 м и 0,08 у порога поимки. Проба направлена на устранение затяжного преследования без повторения контакта; 99 локальных тестов прошли. Образ собран. Первый повтор проблемного seed 2 (`results/series-20260928T180424Z/`, сценарий 2, 90 с) дал поимку на 36,7 с, **0 контактов**, скорости 0,196/0,214 м/с; одного повтора недостаточно. Серия seed 0–2 `results/series-20260928T180613Z/` (90 с) дала 1 поимку/1 цель/1 тайм-аут, **0 контактов**, средние скорости 0,190/0,200 м/с; по одному матчу каждой роли ниже порога 0,2. Это немного выше предыдущей серии на тех же стартах (0,182/0,193), но выборка мала и один матч не решил задачу. Серия сценария 1 `results/series-20260928T181223Z/` (seed 10–12, 90 с) дала 1 цель/1 поимку/1 тайм-аут, **0 контактов**, средние скорости 0,187/0,204 м/с. Вместе с двумя прогонами сценария 2 на этом профиле — семь независимых матчей без контактов; скорости исследователя ещё часто ниже 0,2 м/с, а доли побед по малой выборке недостаточны для критерия. Профиль оставлен для длинной серии; при контакте или ухудшении откатить.

**Стеновой контакт в длинной серии и новая проверка пути:** коммит `cba3f83` был испытан командой `python3 benchmarks/run_duel_series.py --runs 20 --start-seed 0 --active-s 360 --scenario 2`, каталог `results/series-20260928T222900Z/`. После четырёх завершённых заездов (seed 0–3) — 3 цели/1 поимка, средние 0,200/0,190 м/с; в seed 3 страж коснулся стены на 53,7 с при позе в кадре `map` (−0,460; 0,601), рядом с углом внутренней перегородки. `summary-partial.json` помечает прерывание до пятого результата; `helm stop_match` и `helm clean duel` безопасно остановили симуляцию. Контакт опровергает уверенную езду текущей версии, несмотря на семь предыдущих коротких матчей без контактов. `safe_segment` раньше допускал движение почти параллельно стене, если робот уже оказался внутри требуемого зазора. Теперь обычный локальный путь требует `robot_radius + 0,12 м` до стены/границы, а при старте внутри зазора допускает только увеличение расстояния по мере движения. Тест параллельного движения у стены и все 100 локальных тестов прошли. Образ собран; повтор сценария 2 seed 3 `results/series-20260928T224146Z/` (120 с) дал поимку за 43 с, **0 контактов**, но скорости 0,166/0,164 м/с, `OK` 0,946/0,951 и `RECOVERY_ESCAPE` около 5%/4%: прямой запрет снизил эффективность. Проба повысила штраф A* за проход ближе `robot_radius + 0,20 м` к стене (вес 8 вместо 4), рассчитывая вернуть глобальный маршрут к центру коридора. 100 локальных тестов прошли, но контроль seed 3 `results/series-20260928T224730Z/` был слишком коротким (поимка за 9 с) для вывода; серия seed 0–2 `results/series-20260928T224845Z/` (120 с) дала 2 цели/1 тайм-аут и **0 контактов**, однако средние скорости только 0,183/0,169 м/с, все три матча каждой роли ниже 0,2. Дополнительный штраф A* **отклонён и откатан**; усиленная проверка локального зазора остаётся. Нужно улучшать сам локальный путь/контроль, не делать глобальный маршрут излишне длинным.

**Отклонённая проба формы кривой:** `curved_guidance` проверял пять сочетаний длин касательных вместо одного. 101 локальный тест прошёл, но Gazebo-серия `results/series-20260928T230232Z/` (сценарий 2, seed 0–2, 90 с) дала 1 цель/2 поимки, 0 контактов и средние только 0,161/0,168 м/с; доля кривых в первых двух матчах была 0%, шлюз обнулял до 43% команд. Значит главным ограничителем был строгий запас `robot_radius + 0,18 м` у стен, а не форма кривой. Пять вариантов и их тест **удалены**.

**Следующая совместная проба планировщика и шлюза, пока без Gazebo:** вернулся один исходный кубический изгиб с прежними пределом кривизны и проверкой сегментов, но допуск движущейся кривой к стене уменьшен до `robot_radius + 0,10 м`. Для реальной защиты `hsl_mpc_gate` теперь использует свежие точки скана и накопленной статической карты в кадре `map`: строит прогноз текущей дифференциальной команды на 0,6 с, снижает скорость у препятствия ближе 0,41 м до 0,16 м/с (с пропорциональным изменением угловой скорости) и блокирует поступательное движение в пределах 0,30 м, если оно не увеличивает зазор. Точки наблюдаемого соперника исключаются из стенового облака, чтобы не мешать регламентной поимке; планировщик уже учитывает соперника отдельно. 101 локальный тест прошёл, включая остановку перед стеной, безопасный уход от неё и отклонение кривой к препятствию. Требуются сборка, проверка периода шлюза и физические дуэли: при контакте откатить/исправить до дальнейшего повышения скорости.

**Итог этой пробы:** образ собран; `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 3 --active-s 60 --scenario 2 --trace`, `results/series-20260928T231526Z/`: тайм-аут, 0 контактов, средние скорости исследователя/стража лишь 0,130/0,111 м/с; доля команд с положительным MPC, обнулённых шлюзом, 213/617 и 261/601. Доля кривых 0,117/0. Время вычисления шлюза p95 3,23/2,98 мс, так что причина не задержка вычислений. Этот вариант **отклонён**: защита препятствий в шлюзе и её тест удалены, запас для кривых возвращён к `robot_radius + 0,18 м`; усиленная проверка прямого сегмента `robot_radius + 0,12 м` и обязательный выход из близости к стене сохранены для отдельной проверки. Сборка и повтор Gazebo требуются; нижний порог средней скорости 0,2 м/с не достигнут.

**Повтор без дополнительной защиты шлюза:** `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 3 --active-s 120 --scenario 2 --trace --build`, `results/series-20260928T232002Z/`: исследователь достиг площадки за 49,4 с, 0 контактов у обеих ролей, средние скорости 0,155/0,128 м/с, доли поступательного движения 0,637/0,560, доли поворота 0,350/0,426; планировщик `OK` 0,972/0,980. Доля кривых 0,043/0. Изоляция пробы шлюза показывает, что он не единственная причина низкой скорости: даже безопасная строгая траектория часто заставляет разворачиваться. Нельзя выводить улучшение/ухудшение исхода из одного стохастического seed. Следующая проба отдельно уменьшит запас для кривых до `robot_radius + 0,12 м`, сохраняя строгую проверку всех их сегментов; при контакте вернуться к большему запасу.

**Итог пробы запаса кривых 0,12 м:** `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace --build`, `results/series-20260928T232256Z/`. Seed 12: цель за 37,8 с, скорости 0,185/0,193; seed 13: тайм-аут, 0,156/0,146; seed 14: поимка за 17,1 с, 0,134/0,228 м/с (порядок скоростей везде исследователь/страж). Контактов во всех трёх матчах нет; средние скорости по ролям 0,158/0,189, ниже промежуточного порога. Доля кривых по трассам — 0–8,7%, у стража в двух матчах 0%. Ослабление запаса **отклонено**: вернулся `robot_radius + 0,18 м`; оставлена более строгая проверка прямого пути. Физическая серия подтверждает только ограниченную безопасность трёх коротких матчей, не отсутствие столкновений в длинной серии. Следующий цикл должен менять построение и удержание локальной траектории, а не только допуск: при `OK` у планировщика доля поворота на месте всё ещё достигает 0,35–0,42 и порог скорости не выполнен. Нужны повторные Gazebo-заезды, затем 20 × 360 с после достижения безопасности и скорости.

**Проба удержания пути при боковом отклонении до 0,22 м:** вместо 0,12 м расширен допуск в `reusable_local_guidance`, но прямое соединение с сохранённым путём и все его остаточные сегменты продолжали проходить проверку препятствий. `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 3 --active-s 90 --scenario 2 --trace --build`, `results/series-20260928T233452Z/`: seed 3 — поимка за 42,7 с, скорости 0,105/0,130; seed 4 — цель за 34,7 с, 0,164/0,169; seed 5 — поимка за 18,8 с, 0,116/0,176 м/с. Контактов нет, средние по ролям 0,130/0,157 м/с, доля поворота 0,425/0,311. В seed 3 доля обновлений с новой геометрией пути 0,594/0,634 против 0,555/0,753 в одиночном прежнем матче того же seed; уменьшение недостаточно устойчиво, а доля остановленных шлюзом команд составила 0,563/0,459. Изменение допуска **отклонено**, вернулся 0,12 м. Следующий эксперимент — разрешать геометрически проверенную кривую при большем начальном угле, если её кривизна и зазор действительно допустимы. Не считать расширение допуска решением проблемы изломов.

**Проба большего начального угла и кривизны:** предел начального отклонения расширен с 1,15 до 1,35 рад, кривизны с 2,6 до 4,0 м⁻¹ с прежним зазором до стен. Локальный тест подтвердил возможность кривой в свободном пространстве. `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace --build`, `results/series-20260928T234250Z/`: seed 12 — цель за 42,9 с и скорости 0,176/0,145; seed 13 — цель за 67,8 с и 0,127/0,144; seed 14 — поимка за 15,6 с и 0,137/0,233 м/с. Во всех трёх матчах контактов нет; средние по ролям 0,152/0,168 м/с, доли поворота 0,356/0,283. В длинном seed 13 доля кривых обеих ролей 0%, в seed 12 — 5%/0%; геометрический выигрыш не проявился. Изменение **отклонено**, пределы 1,15 рад и 2,6 м⁻¹ возвращены, тест пробы удалён. Следующий шаг — измерить причины отклонения кривой в живом планировщике (угол, запас в начале, препятствие на кривой, кривизна), затем менять форму траектории по установленной причине. Не поднимать командную скорость до устранения лишних разворотов.

**Диагностика отказов кривой на исходной геометрии:** необязательный счётчик причин в `curved_guidance` и журнал планировщика каждые 10 с симуляции добавлены без изменения ROS-интерфейса. `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 13 --active-s 90 --scenario 1 --trace --build`, `results/series-20260928T235317Z/`: поимка за 67,5 с, 0 контактов, скорости 0,180/0,206 м/с. Из логов контейнеров: исследователь отклонил кривую из-за длины видимого прямого участка <0,65 м **88 раз из 108** попыток, страж — **115 из 137**; недостаточный зазор по самой кривой 7/4 раза, на старте 1/2, превышение кривизны 0/1. Принята только одна кривая у исследователя. Результат направляет следующую пробу: разрешить короткую кривую от 0,35 м с пропорционально короткой касательной, оставив проверку всех точек/сегментов, зазор `robot_radius + 0,18 м` и кривизну ≤2,6 м⁻¹. Эта проба ещё требует Gazebo-проверки.

**Итог короткой кривой:** `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 13 --active-s 90 --scenario 1 --trace --build`, `results/series-20260929T000020Z/`: исследователь достиг цели за 46,7 с, 0 контактов, но скорости 0,148/0,155 м/с и доли поворота 0,333/0,348. Счётчики планировщиков: у исследователя 31 отказ `short_distance`, 11 `start_clearance`, 7 `heading_large` и только 1 принятая кривая из 58 попыток; у стража 12 `short_distance`, 3 `curve_clearance`, 2 `heading_large`, 1 принятая кривая из 25. В трассах доля кривых 0,033/0 и шлюз обнулил 0,361/0,372 активных команд. Следовательно, снижение минимальной длины до 0,35 м лишь переместило часть отказов к физическому запасу/углу и не улучшило движение; проба **отклонена**, порог 0,65 м и исходная касательная возвращены. Диагностические счётчики оставлены. Следующая гипотеза: текущий поиск *только прямой видимости* даёт слишком короткую траекторию перед каждым изгибом. Нужна локальная траектория через следующий поворот глобального маршрута с проверкой всей кривой и достаточным запасом для MPC; один прямой участок с попыткой притянуть к нему текущий курс этого не решает.

**Проба кривой через изгиб глобального маршрута:** если прямой обзор короче 0,75 м, планировщик пробует до восьми более дальних точек текущего маршрута и касательных по соседним точкам, строит кубическую кривую и проверяет её целиком с зазором `robot_radius + 0,18 м`, кривизной ≤2,6 м⁻¹ и проверкой соперника. Синтетический L-поворот с препятствием подтвердил обход: прямая заканчивалась через 0,2 м, проверенная кривая продолжалась до 1,4 м, минимум сырого зазора 0,40 м; добавлен тест отказа при новом препятствии на кривой. **101 тест** прошёл. Первый Gazebo-матч `results/series-20260929T000928Z/` (сценарий 1, seed 13, 90 с) дал цель за 45,3 с, 0 контактов, скорости 0,168/0,209 м/с, долю кривых 0,176/0,075; p95 цикла планировщика 111/113 мс, превышений 200 мс нет. Серия `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 3 --active-s 90 --scenario 2 --trace --build`, `results/series-20260929T053156Z/`: seed 3/4/5 — **три цели, ноль поимок**, ноль контактов; средние скорости 0,170/0,170 м/с, доли поворота 0,285/0,314, `OK` 0,968/0,983. Кривые в сценарии 2 появлялись лишь в 0–4,1% обновлений; доля обнулённых шлюзом команд 0,290–0,394, p95 планировщика 96–126 мс, превышений бюджета нет. Изменение оставлено как геометрически безопасный маршрутный обход в сценарии 1, но **улучшение движения/баланс роли не подтверждены**. Следующая работа: уменьшить число бесполезных проверок кандидатов по диагностическим причинам `route_start_clearance`, `route_heading_large`, `route_curve_clearance`; затем повторить одинаковые сценарии, отслеживая долю принятых маршрутных кривых, скорость, захват и задержки. Не повышать максимум скорости; нужны повторные поимки стража и не менее 20 × 360 с только после устойчивого движения.

**Проба адаптивной касательной:** когда стандартная кубическая кривая отвергается по зазору или кривизне, теперь проверяются ещё две длины касательной, а дальний список сокращён до шести точек для ограничения времени расчёта. Тесты подтверждают, что альтернативная форма обходит синтетический угол с тем же запасом, а новое препятствие отменяет путь. 102 теста прошли. Gazebo `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace --build`, `results/series-20260929T054123Z/`: seed 12/13/14 дали 2 цели и 1 поимку, 0 контактов; скорости по ролям 0,174/0,187 м/с, движение 0,705/0,764, поворот 0,276/0,217, `OK` 0,975/0,976. Каждая роль была ниже 0,2 во всех матчах кроме одного стража; до цели/поимки 42,4/18,5/39,1 с. Кривые составляли 0–18,4% обновлений по роботу; p95 планировщика 99–119 мс, превышений 200 мс не было. Счётчики показали две `route_handle_accepted` у стража; чаще всего оставался `route_heading_large` (51 случая у исследователя). Проба выглядит безопасной, но порог скорости не достигнут и преимущество по сравнению с предыдущей серией не доказано. Следующая изолированная проба расширит начальное отклонение лишь до 1,35 рад при прежней проверке каждого участка и кривизне ≤2,6 м⁻¹; проверить частоту принятых дуг, столкновения и вычислительный бюджет.

**Итог расширения начального угла до 1,35 рад:** те же seed 12–14, сценарий 1, `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace --build`, `results/series-20260929T055030Z/`. Все три матча закончились поимкой, 0 контактов; средние скорости исследователь/страж 0,142/0,209 м/с, доли поворота 0,371/0,148. Кривые использовались в 0–9,3% обновлений у исследователя и 0–4,8% у стража; p95 цикла планировщика 91–116 мс, превышений 200 мс нет. У стража порог 0,2 пройден в двух матчах, у исследователя ни разу. Расширение **отклонено**, предел возвращён до 1,15 рад: оно усилило поимки, но ухудшило движение и успех исследователя; кривизна и проверка зазора остаются прежними. Адаптивная длина касательной сохранена. Следующая проверка касается реакции исследователя на близкого соперника и маршрута уклонения; не изменять стеновую безопасность и скорость одновременно.

**Проверка точки уклонения с продвижением к цели:** наблюдения предыдущей серии `results/series-20260929T054123Z/` показали, что исследователь проводил 21,5–79,5% времени в `EVADE`, хотя его первичная задача — цель. Протестирована точка, являющаяся суммой единичного вектора от стража и 0,35 единичного вектора к цели; оба вклада проверены модульным тестом. `python3 -m pytest -q tests/test_navigation.py helm_launch/tests/tests.py` — 103 теста. Gazebo, те же seed 12–14 и сценарий 1: `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 12 --active-s 90 --scenario 1 --trace --build`, `results/series-20260929T060218Z/`. Итог — 1 достижение цели, 1 поимка, 1 тайм-аут; контактов нет, скорости по ролям 0,150/0,157 м/с против 0,174/0,187 в предыдущей серии. Доля поворота — 0,355/0,336, у исследователя `EVADE` всё ещё занимал 42,0–55,9% времени; доли планировщика `OK` 0,969/0,979. Кривые составили 0–7,0% обновлений пути, шлюз ограничивал 29,8–45,4% команд, p90 ошибки курса пути — 1,90–2,58 рад. Смешанная точка **отклонена и удалена**: результат хуже базовой серии и не подтвердил большее продвижение; исходная точка строго от стража восстановлена. Адаптивные касательные кривых пока оставлены: в их отдельной серии `054123Z` было 2 цели/1 поимка без контактов, но скорости всё ещё не достигали 0,2 м/с. Следующий диагностический шаг — собрать причины принятия/отказа кривой из живого лога планировщика и сопоставить с большим p90 угла и частыми ограничениями шлюза; геометрию стен и защитный зазор не ослаблять.

**Безопасная кривая при выходе из тесного старта:** живые логи контрольного прогона `results/series-20260929T061553Z/` дали до 37 отказов `route_start_clearance` за 10 симуляционных секунд и частые команды шлюза на разворот. Контрольный seed 13 завершился целью за 51,8 с, 0 контактов, скорости 0,160/0,147 м/с. Изменена проверка кривой: если стартовая дистанция до стены или границы меньше резерва `robot_radius + 0,18 м`, кривая разрешается только при подтверждённом росте этого зазора вдоль пути; отдельная проверка каждого сегмента и минимальный физический запас остаются. Добавлен тест тесного старта. `python3 -m pytest -q tests/test_navigation.py helm_launch/tests/tests.py` — 103 теста. Gazebo-серия тех же seed 12–14, сценарий 1, `results/series-20260929T061936Z/`: 1 цель, 1 поимка, 1 тайм-аут, 0 контактов; скорости исследователь/страж 0,162/0,174 м/с, поворот 0,307/0,255, `OK` 0,969/0,970. Доли кривых в диагностических трассах — 0–9,4%, все шесть индивидуальных скоростей ниже 0,2 м/с. Изменение **оставлено**: оно исправляет безопасный выход от стены и в этой выборке не добавило контактов, но прирост скорости не подтверждён. Следующая гипотеза — когда свободный прямой участок длинный, но его начальное направление сильно расходится с курсом робота, планировщик сразу передаёт прямую линию, и шлюз разворачивает корпус на месте; проверить дугу через следующий доступный изгиб маршрута в таком случае, сохраняя `1,15 рад`, зазор и полную проверку сегментов.

**Дуга через изгиб при длинном, но плохо ориентированном обзоре:** `route_curve_guidance` теперь также рассматривает следующий изгиб, если прямая безопасна длиннее 0,75 м, но её курс отличается от курса робота минимум на 0,65 рад; если рассогласование меньше, сохраняется прежний путь. Геометрия дуги, ограничение 1,15 рад, запас и проверка всех сегментов не ослаблены. Добавлен синтетический случай: безопасная линия длиной 1,27 м ведёт на 45° к курсу, а дуга через следующий поворот остаётся безопасной. **104 теста** прошли. Первая сборка Gazebo `results/series-20260929T063355Z/` трижды не дождалась потоков позы (`gzserver exited before both pose streams were ready`); ручной `helm up duel` успешно поднял оба робота, после очистки повторена оценка на том же образе. Серия `results/series-20260929T063928Z/`, сценарий 1, seed 12–14: 3 достижения цели, 0 поимок/тайм-аутов, 0 контактов; средние скорости 0,175/0,172 м/с, доли поворота 0,257/0,292, `OK` 0,973/0,974. По трассам дуги заняли 7,1–19,5% обновлений против 0–9,4% в предыдущем варианте; шлюз ограничивал 26,9–31,1% команд против прежних 20,1–37,2%. Это наблюдаемый выигрыш локальной плавности/активности в одной серии, но скорости ниже 0,2, а все исходы в пользу исследователя. Дальше проверить неизменённую логику на сценарии 2 и выяснить, теряет ли страж время в `SEARCH` или при `PURSUE`; не ослаблять габаритные проверки и не подгонять балансы.

**Проверка сценария 2 и слабой роли стража:** на образе `647ae4d+dirty.8f3053c665e1`, `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 0 --active-s 90 --scenario 2 --trace`, `results/series-20260929T064700Z/`: исследователь достиг цели во всех матчах, поимок и контактов нет. Средние скорости исследователь/страж 0,177/0,149 м/с; поворот 0,288/0,331, `OK` 0,974/0,966. Страж в seed 0 искал цель 59,8% времени и преследовал 39,9%; в seed 1 — поиск 47,3%, преследование 52,2%; в seed 2 — поиск 18,4%, преследование 73,5%, захват 7,6%. Доля видимости соперника составляла 0,341/0,457/0,789 по матчам, так что одними потерями видимости провал поимок не объясняется. Доли кривых у стража 0–1,1%, у исследователя 8,4–16,3%; у стража остаётся 30,4–44,6% команд с ограничением шлюза и p90 ошибки курса 2,24–2,31 рад. Изменённая логика пока **оставлена для следующей проверки**: на сценарии 1 кривые участились и ограничение шлюза снизилось без контактов, но у стража в сценарии 2 нет ни одной поимки, и средняя скорость обеих ролей ниже 0,2. Следующий цикл должен стабилизировать преследование движущейся цели и устранить лишние повороты/поиск у стража, не предполагая фиксированную скорость исследователя.

**Порог попытки маршрутной дуги для стража:** уменьшен только порог рассогласования курса, при котором планировщик ищет дугу через изгиб маршрута (с 0,65 до 0,45 рад); кривизна, запас и 1,15 рад для входного участка не менялись. Тест подтвердил безопасную дугу при 0,588 рад; всего **105 тестов** прошли. Повтор `results/series-20260929T065715Z/`, сценарий 2, seed 0–2, `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 0 --active-s 90 --scenario 2 --trace --build`: одна поимка, две цели, 0 контактов. Средняя скорость исследователь/страж 0,166/0,161 м/с против 0,177/0,149 в серии 064700Z; поворот 0,313/0,305 против 0,288/0,331. Доля кривых у стража выросла с 0–1,1% до 2,1–8,0%; у исследователя снизилась с 8,4–16,3% до 4,9–7,7%. В пределах этой малой серии изменение **оставлено**: у стража чуть выше скорость, меньше поворотов и появилась поимка, но стабильный выигрыш не установлен и порог 0,2 не достигнут ни в одном отчёте. Следующий изолированный эксперимент — укоротить горизонт линейной экстраполяции перехвата с 3 до 2 с: оценка скорости противника по наблюдениям остаётся динамической, а больший упреждённый маршрут может уводить стража за быстро меняющуюся цель.

**Горизонт экстраполяции перехвата 2 с:** функция использует измеренную скорость соперника и сохраняет ограниченный прогноз; изменён только максимальный горизонт с 3 до 2 с. Добавлена проверка границы прогноза, **106 тестов** прошли. Серия `results/series-20260929T070851Z/`, сценарий 2, seed 0–2: 1 поимка, 2 цели, 0 контактов. Средняя скорость исследователь/страж 0,187/0,188 м/с против 0,166/0,161 на горизонте 3 с; доля поворота 0,207/0,218 против 0,313/0,305. `OK` 0,968/0,976. Доля стража в `SEARCH` сократилась до 8,6–18,5% с 18,4–59,8%; видимость соперника была 0,815–0,914. Доля кривых у стража — 0,8–14,9%, шлюз ограничивал 17,7–29,2% команд. Улучшение выраженное, но выборка только три матча: горизонт 2 с **сохранён как кандидат**, требуется проверка на сценарии 1 и длинные прогоны до оценки эффективности.

**Проверка горизонта 2 с на сценарии 1:** `results/series-20260929T071857Z/`, seed 12–14, та же версия `6ecdf85`: три достижения цели, 0 поимок, 0 контактов; средние скорости 0,176/0,164 м/с, поворот 0,273/0,305, `OK` 0,980/0,973. Результат подтверждает, что сценарий 1 остаётся слабым для стража и не переносит рост скорости из сценария 2; порог 0,2 не выполнен ни в одном отчёте. Горизонт 2 с оставлен как кандидат из-за сценария 2, но его общий эффект **не подтверждён**. Следом провести независимую серию сценария 2 на seed 3–5; отдельно продолжать искать причину частых поворотов стража в сценарии 1.

**Новые seed 3–5 для горизонта 2 с:** сценарий 2, `results/series-20260929T072520Z/`, без пересборки `6ecdf85`. 1 поимка (11,5 с), 2 достижения цели, контактов нет. Средняя скорость исследователь/страж 0,172/0,176 м/с, поворот 0,290/0,256, `OK` 0,962/0,978. Вместе с seed 0–2 получаем на горизонте 2 с 2 поимки/4 цели из шести коротких матчей, скорость 0,180/0,182 м/с и поворот 0,249/0,237. Для сравнения, горизонт 3 с на первых seed 0–2 дал 1 поимку/2 цели, 0,166/0,161 м/с и 0,313/0,305 поворота. Контактов не было в обеих выборках, но четыре из шести матчей на горизонте 2 с ниже 0,2 для обеих ролей, и сценарий 1 остался 0 поимок из трёх. Вывод: преимущество 2 с по движению на сценарии 2 подтверждается несколькими короткими матчами, но итог для слабого стража и требования скорости остаётся недостаточным. Следующий шаг — изучить доступные изменения кривизны/дальности безопасной маршрутной дуги для сокращения времени разворотов без повышения скорости; потом повторить обе карты, сохраняя seed и раздельные роли.

**Длина маршрутной дуги до 2,2 м:** `route_curve_guidance` рассмотрит более дальний безопасный waypoint (предел был 1,8 м), все точки, сегменты и кривизна по-прежнему проверяются; добавлен тест для точки на 1,95 м. **107 тестов** прошли. `results/series-20260929T073300Z/`, сценарий 1, seed 12–14 на версии горизонта 2 с: 2 поимки/1 цель, 0 контактов; скорости исследователь/страж 0,167/0,177 м/с, поворот 0,294/0,230, `OK` 0,967/0,971. Частота кривых в трассах 3,3–18,3%. Сравнение с горизонтом 2 с и пределом 1,8 м `results/series-20260929T071857Z/`: было 0 поимок/3 цели и 0,176/0,164 м/с, поворот 0,273/0,305. Это улучшает результат стража в данной короткой серии и не дало контактов, но у исследователя средняя скорость упала; предел 2,2 м **оставлен условно**. Следующий цикл посвящён избытку режима `EVADE` у исследователя, не меняя стеновую безопасность.

**Отклонённое снижение дистанции EVADE:** параметр 1,8→1,5 м сократил зону уклонения, сохранив те же безопасные пути `GOAL/EVADE`. `results/series-20260929T074525Z/`, сценарий 1, seed 12–14: 2 цели/1 поимка, 0 контактов, но скорость исследователь/страж упала до 0,132/0,166 м/с, а доля поворота выросла до 0,409/0,307 (против 0,167/0,177 и 0,294/0,230 при 1,8 м в серии 073300Z). Порог 1,5 м **отклонён**, значение 1,8 м возвращено в код, node defaults и оба YAML. Число целей не доказывает улучшение при существенно худшем движении. Следующая работа по исследователю — выбирать направление уклонения среди безопасных достижимых частей маршрута с сохранением прогресса к площадке, а не просто менять порог поведения.

**Перенос длинной дуги на сценарий 2:** с исходным `evade_distance=1,8`, горизонтом перехвата 2 с и длиной маршрутной дуги 2,2 м серия `results/series-20260929T075551Z/` (seed 0–2) дала 1 поимку/2 цели, 0 контактов; скорость исследователь/страж 0,170/0,165 м/с, поворот 0,287/0,258. Против версии 1,8 м на тех же seed `070851Z` исходы те же, но скорости были 0,187/0,188, поворот 0,207/0,218. Значит длинный диапазон глобально замедляет сценарий 2, хотя помогал стражу в сценарии 1. Следующая проба ограничит 2,2 м только поведением `PURSUE`, оставив исследователю и остальным поведениям 1,8 м; обе длины полностью проходят те же проверки безопасности.

**Длинная дуга только при `PURSUE`:** в planning node диапазон 2,2 м передаётся только для преследования; остальные режимы ограничены 1,8 м. Модульный тест проверяет короткий диапазон, синтетическая траектория подтверждает безопасную дугу 1,95 м; **108 тестов** прошли. На scenario1 `results/series-20260929T080430Z/`, seed 12–14: 2 поимки/1 цель, 0 контактов, средняя скорость исследователь/страж 0,149/0,206 м/с, доля поворота 0,356/0,148, `OK` 0,966/0,973. На scenario2 `results/series-20260929T081204Z/`, seed 0–2: также 2 поимки/1 цель, 0 контактов, скорости 0,156/0,163 м/с, поворот 0,334/0,289, `OK` 0,965/0,976. Сравнение с исходным диапазоном 1,8 м на scenario2 seed 0–2 (`070851Z`) — 1 поимка/2 цели и 0,187/0,188 м/с: новая выборка дала больше поимок, но меньше скорость и больше поворотов. В scenario1 скорость стража и захваты выросли заметно, скорость исследователя просела на коротких матчах с ранним захватом. Дальность 2,2 м **оставлена только для `PURSUE` как кандидат**; это не подтверждает средний порог 0,2 м/с для обоих и требует дальнейшего разбора навигации исследователя.

**Отклонённая точка отхода 1,2 м:** `evade_target_distance` был параметризован и снижен с 1,5 до 1,2 м при прежнем пороге режима 1,8 м. `results/series-20260929T082146Z/`, scenario1, seed 12–14: 3 поимки, 0 целей, 0 контактов; скорость исследователь/страж 0,143/0,223 м/с, поворот 0,369/0,074. При прежних 1,5 м (`080430Z`) было 2 поимки/1 цель, скорость 0,149/0,206 и поворот 0,356/0,148. Короткая точка отхода **отклонена**, длина 1,5 м восстановлена и сохранена параметром в обоих decision YAML. Она помогла стражу, но полностью убрала успех исследователя и ухудшила его скорость. Следующая архитектурная проба должна дать planner-у оба направления: временную безопасную реакцию от противника и долгосрочную цель; разрешать отход через глобальный маршрут только пока сохраняется достижимый путь к площадке.

**EVADE ведёт к площадке по безопасному глобальному маршруту:** decision manager при выборе `EVADE` теперь оставляет целью площадку, но передаёт планировщику clearance 1,0 м и вес противника 8,0. A* использует существующее правило выхода из уже нарушенной зоны: сначала выбирает разрешённый безопасный выход, затем продолжает путь к цели и не входит обратно в защитный радиус. Добавлен тест такой последовательности; **109 тестов** прошли. `results/series-20260929T083404Z/`, scenario1, seed 12–14: 3 цели/0 поимок, 0 контактов, скорости исследователь/страж 0,190/0,154 м/с, поворот 0,231/0,349, `OK` 0,979/0,980. Против прежней точки отхода 1,5 м, серии 080430Z (2 поимки/1 цель, скорости 0,149/0,206) исследователь стал заметно быстрее и успешнее, но страж замедлился. Изменение **кандидат**, пока проверено лишь на scenario1. Следующий шаг — прогнать scenario2 на том же образе; если смена ролей подтверждает сильный перекос, разделить тактику избегания и перехвата на уровне целевого пути.

**Проверка EVADE→площадка на другом старте:** `results/series-20260929T084331Z/`, scenario2, seed 0–2: 2 достижения площадки/1 поимка, 0 контактов. Средняя скорость исследователя/стража 0,165/0,143 м/с; `OK` 0,964/0,959; доля времени с поворотом 0,306/0,329. В паре коротких серий сценариев 1 и 2 исследователь выиграл 5/6 матчей, а страж — 1/6; средние скорости по ролям 0,178/0,149 м/с. Поэтому это изменение помогает исследователю безопасно продолжать к цели, но не удовлетворяет ни требование средней скорости 0,2 м/с, ни баланс поимок на выборке. Версия остаётся экспериментальной: следующий цикл должен усиливать перехват стража по наблюдаемой динамике исследователя, не вводя предположений о максимальной скорости противника, и уменьшать развороты/остановки. Нужна повторная парная серия после изменения; длинный критерий 20×360 с ещё не запускался.

**Сглаживание небольших сдвигов точки перехвата стража:** цель PURSUE фильтрует изменения до 0,8 м с `alpha=0,55`; крупное перемещение, например при резкой смене курса или выборе другой доступной точки, применяется сразу. Текущую velocity оценщик продолжает брать из одометрии соперника. На scenario2, seed 3–5, `results/series-20260929T085249Z/`: 2 цели/1 поимка, 0 контактов; скорости исследователь/страж 0,184/0,177 м/с, доли движения 0,699/0,779 и поворота 0,280/0,191. Контроль на тех же seed без сглаживания (`alpha=1.0`), `results/series-20260929T090041Z/`: те же 2 цели/1 поимка и 0 контактов, но скорости 0,156/0,160 м/с, доли движения 0,655/0,682 и поворота 0,333/0,299. Тест показывает полезный эффект фильтра на этой выборке для обеих ролей; ни одна роль пока не достигла требуемых 0,2 м/с в среднем. Изменение оставлено кандидатом. У стража геометрия пути всё ещё менялась часто, поэтому отдельный остаточный дефект — устойчивость глобального маршрута при движущейся цели.


**Перепроверка сглаженного перехвата и запаса стен:** `results/series-20260929T091235Z/`, scenario1 seed 12–14: 2 цели/1 поимка, скорость исследователь/стража 0,169/0,194 м/с; у исследователя один контакт со стеной на (1,664; 0,399). Общий запас +4 см обеим ролям (`results/series-20260929T092051Z/`) убрал контакт, но дал 3 цели/0 поимок и скорости 0,161/0,143 м/с; общий запас отклонён, поскольку страж замедлился и не ловил. Роль-зависимая проверка локальных прямых сегментов (+4 см исследователю, исходный +12 см стражу) на scenario1 seed 12–14 (`results/series-20260929T093604Z/`) вернула 2 поимки/1 цель и 0 контактов, скорости 0,162/0,220 м/с. Но на scenario2 seed 3–5 (`results/series-20260929T094049Z/`) исследователь достиг цели только в одном матче из трёх, средняя скорость 0,121 м/с и `planner_ok=0,763`; seed 3 провёл 55% времени в `RECOVERY_ESCAPE`. Поэтому +4 см исследователю тоже слишком строгий. При +2 см исследователю и +12 см стражу scenario2 seed 3–5 (`results/series-20260929T094618Z/`) дал 2 цели/1 поимку, 0 контактов, средние скорости 0,170/0,166 м/с и `planner_ok=0,975/0,974`; scenario1 seed 12–14 (`results/series-20260929T095613Z/`) — 2 цели/1 поимку, 0 контактов, скорости 0,156/0,193 м/с, `planner_ok=0,913/0,981`. Запас +2 см оставлен кандидатом для проверки: в сумме 4 цели/2 поимки без контактов на этих шести матчах, но средние скорости обеих ролей пока ниже 0,2 м/с и исследователь всё ещё часто восстанавливает путь около препятствий.
**Следующий цикл, сценарий 1:** независимая серия `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 10 --active-s 90 --scenario 1`, `results/series-20260928T080214Z/`: 2 поимки и 1 достижение площадки, средние скорости по ролям 0,195/0,211 м/с. Seed 10 завершился поимкой за 14,3 с, но у обоих роботов зарегистрирован один **контакт друг с другом** (стеновых контактов во всех трёх матчах нет). У стража точка сближения находилась в 0,39 м от соперника, а предел при `CAPTURE` не снижался с расстоянием; при радиусе корпуса 0,178 м оставался малый запас на задержку наблюдения/остановки. Точка перенесена на 0,42 м, скорость `PURSUE`/`CAPTURE` у видимого близкого соперника уменьшается до 0,08–0,35/0,65 м/с по расстоянию. Это не меняет оценку **скорости соперника**: она выводится из последовательных видимых положений. На новом образе повтор `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 10 --active-s 90 --scenario 1`, `results/series-20260928T115401Z/`: поимка за 14,7 с, 0 контактов, скорости 0,180/0,192 м/с. 92 теста и `git diff --check` прошли. Один повтор подтверждает исправление конкретного случая, но не надёжность на серии и не порог средней 0,2 м/с. Следующая проверка — независимая серия нескольких seed на том же образе и затем разбор коротких остановок/разворотов.

**Порог короткой дуги 0,65→0,45 м — отклонён:** цель была включить одновременный поворот/движение на коротком подходе перед изгибом; контроль кривизны и clearance не ослаблялись, ручка Безье адаптировалась к длине. Локальные тесты временно прошли 112/112. На scenario2 seed 3–5 (`results/series-20260929T100510Z/`) против порога 0,65 м (`094618Z`) кривые стали использоваться реже: около 3–4% вместо 6–11% обновлений; средние скорости упали с 0,170/0,166 до 0,158/0,146 м/с, доля поворота выросла с 0,276/0,281 до 0,309/0,315, исходы сменились с 2 цели/1 поимка на 3 цели. Везде 0 контактов. Изменение откатил. Трассы показывают, что дополнительный источник поворотов на месте — MPC gate: он задаёт нулевую линейную скорость, пока heading error не упадёт ниже гистерезисного порога 0,35–0,65 рад; следующий тест должен разрешать одновременное движение только по уже проверенным кривым, сохранив вращение на месте для прямых и тесных манёвров.


**MPC gate: совместное движение на проверенной кривой:** при локальной полилинии с отклонением от хорды более 3,5 см умеренная ошибка касательной до 0,9 рад больше не обнуляет поступательную скорость; MPC может одновременно рулить и ехать. Для прямого пути прежний порог разворота 0,65/0,35 рад сохранён, а при ошибке на кривой выше 0,9 рад тоже остаётся разворот на месте. Два теста проверяют ветви прямого и кривого пути; **113 тестов** прошли, образ `duel` пересобран. На scenario2 seed 3–5 (`results/series-20260929T101833Z/`): 2 цели/1 поимка, 0 контактов, скорости исследователь/стража 0,162/0,164 м/с, поворот 0,275/0,267; доля кривых выросла примерно 0,086→0,214 у исследователя и 0,068→0,086 у стража против варианта с исходным gate (`094618Z`). На scenario1 тех же seed (`results/series-20260929T102635Z/`): также 2 цели/1 поимка и 0 контактов; средние скорости 0,140/0,178 м/с, поворот 0,229/0,242 против 0,156/0,193 и 0,291/0,238 до изменения. Но `planner_ok` исследователя упал до 0,786 из-за двух матчей с длительным `RECOVERY_ESCAPE`. Кандидат оставлен для дальнейшей проверки: повороты стали направленнее и кривых больше на scenario2 без контактов, однако скорость и надёжность восстановления пока не достигли требований.
- Новый образ: `results/series-20260928T115540Z/`, сценарий 1, seed 10–12, три независимых матча по 90 с: две поимки и одна цель, **0 контактов во всех шести отчётах**, средняя скорость исследователя/стража 0,201/0,203 м/с. При этом по одному матчу каждой роли ниже промежуточного порога 0,2; среднее по трём матчам не доказывает нижний порог в каждом.

**Watchdog учитывает намеренный поворот стража:** обнаружен цикл, где позиция не сдвигается 4 с, хотя gate безопасно разворачивает робота к пути; watchdog сбрасывал маршрут и повторно запускал recovery. Проверка уменьшения ошибки курса по локальной траектории сбрасывает таймер только для роли guardian в `PURSUE/CAPTURE`; исследователь по-прежнему обязан наращивать линейный прогресс, чтобы не задерживать уклонение. Универсальный вариант отвергнут: scenario1 seed 12–14 (`results/series-20260929T104558Z/`) дал 3 поимки/0 целей, скорость исследователь/стража 0,129/0,197 м/с, 0 контактов. На повторе seed 14 (`104121Z`) страж поймал за 17 с, `RECOVERY_ESCAPE` исследователя 3,2% против 27,8% при gate без такого учёта. Ролевой вариант на scenario1 seed 12–14 (`results/series-20260929T105230Z/`) дал 2 цели/1 поимку, 0 контактов; скорости 0,189/0,198 м/с, доли движения 0,761/0,753, поворота 0,204/0,230, `planner_ok=0,952/0,981`. На scenario2 seed 3–5 (`results/series-20260929T110114Z/`) — также 2 цели/1 поимка, 0 контактов; скорости 0,171/0,171 м/с, `planner_ok=0,905/0,976`. Это лучший пока короткий кандидат: 4 цели/2 поимки, ноль контактов на двух сценариях, обе роли близки к 0,2 м/с в scenario1; усреднённо по двум сериям исследователь/страж всё ещё 0,180/0,185 м/с. **20 матчей по 360 с ещё не запускались**, порог 0,2 пока не подтверждён. Следующий шаг — повторить роль-зависимый watchdog независимыми seed/другим сценарием, затем исправить причины recovery исследователя в scenario2 и только после этого запускать длинную серию.
- Тот же образ: `results/series-20260928T120049Z/`, сценарий 2, seed 0–2: две цели и одна поимка, **0 контактов**, но средняя скорость обеих ролей 0,174 м/с; по два матча ниже 0,2. Планировщик `OK` 0,960/0,981, двигательная активность 0,961/0,990. Причина недостатка в основном — время разворотов и скорость поступательного движения в сложном маршруте, а не пустые пути. Для проверяемой пробы крейсерский предел продольного MPC и шлюза поднят с 0,3 до **0,4 м/с**, модель достижимости собственного стража для перехвата согласована с ним; замедление при сближении оставлено. Эта версия **ещё не проверена в Gazebo**. Если появятся контакты или изломы, исправить или откатить перед дальнейшим увеличением. Следующий шаг — сборка, одинаковые seed сценария 2 и другой старт, сравнение контактов, плавности и средней скорости.

**Одинаковый safety margin при выборе и исполнении recovery:** исследователь использует +0,14 м для прямого локального сегмента, но `recovery_step` раньше проверял направление по стандартным +0,12 м. Теперь recovery проверяет ровно тот же запас, что и путь исполнения. На scenario2 seed 3–5 (`results/series-20260929T110912Z/`) против предыдущей ролевой-watchdog серии (`110114Z`) исходы те же 2 цели/1 поимка, 0 контактов. В худшем seed доля `RECOVERY_ESCAPE` исследователя снизилась с 24% до 7,5%, `planner_ok` вырос 0,745→0,903, скорость 0,116→0,145 м/с. В целом по серии скорость снизилась 0,171→0,155 м/с, поворот вырос 0,225→0,340, `planner_ok` исследователя улучшился 0,905→0,956. Значит исправлена конкретная несогласованность между проверенным и следующим локальным шагом, но улучшение скорости на общей выборке не подтверждено; нужна scenario1 проверка и дальнейший разбор лишних поворотов.
- Проба 0,4 м/с собрана и запущена: `results/series-20260928T121926Z/`, сценарий 2, seed 0–2, две поимки и одна цель, 0 контактов, но средние скорости 0,191/0,176 м/с, по два матча каждой роли ниже 0,2. В первом матче скорости 0,241/0,224, в остальных 0,154–0,175. Дополнительный повтор seed 1 с пассивной трассой `results/series-20260928T122557Z/` завершился целью за 40,5 с, скоростями 0,243/0,180 м/с и **одним контактом исследователя со стеной**. Поэтому 0,4 м/с как рабочий предел **отклонён и в исходниках возвращён 0,3**; нужно пересобрать перед следующим запуском. Трасса за 85 с реального времени включает время вне активного матча, но во время её выборки шлюз обнулял 23,0% команд первого робота и 33,0% команд второго при положительном выходе MPC; почти не было нулевого выхода самого MPC. Резких смен направления глобального пути 1/4. Следующая гипотеза — повороты на стыках коротких прямых локальных отрезков и порог шлюза, а не частое глобальное перепланирование. Нужно измерить геометрию и зазор локального отрезка у стен, затем сгладить безопасные повороты **без ослабления защиты вслепую**.

**Повтор scenario1 после согласования recovery clearance:** `results/series-20260929T111852Z/`, seed 12–14: 3 цели/0 поимок и 0 контактов, средние скорости исследователь/стража 0,162/0,191 м/с, `planner_ok=0,955/0,972`. У стража `PURSUE/CAPTURE` занимали 69–92% матча, видимость соперника 58–91%; длинного recovery у него не было. Предыдущая серия того же watchdog-кандидата `105230Z` дала 2 цели/1 поимку и 0 контактов со скоростями 0,189/0,198. Следовательно, повтор подтвердил безопасное движение и исправление recovery, но исходы остаются стохастичными, а успешность захвата не устойчива. Новая гипотеза — проверить более умеренный горизонт перехвата 2,5 с (предыдущий 3→2 улучшал поворот, но в коротких сериях не повысил скорость стража); модель продолжит использовать только наблюдаемую скорость соперника.
- После отката образ `duel` пересобран. Контрольный независимый матч `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 10 --active-s 90 --scenario 1`, `results/series-20260928T123614Z/`: поимка за 13,4 с, **0 контактов**, скорости 0,197/0,203 м/с, `OK` 0,973/0,963. Исследователь чуть ниже промежуточного порога. `python3 -m pytest -q tests/test_navigation.py helm_launch/tests/tests.py` — 92 теста; `git diff --check` — без ошибок. Это короткая smoke-проверка текущего образа, не доказательство надёжности или 50/50 на ≥20 полных матчах.
- Отдельная **отклонённая** проба плавной кривой для неглубоких поворотов: кубическая траектория от текущего курса к видимой точке с проверкой зазора каждого участка и ограничением кривизны; при невозможности — прежний прямой путь. Два новых локальных теста проходили, но Gazebo-серия `results/series-20260928T124652Z/` (сценарий 2, seed 0–2, предел команды 0,3 м/с) дала средние 0,163/0,160 м/с против 0,174/0,174 без кривой; в seed 0 при поимке был контакт роботов друг с другом. Исходы 2 поимки/1 цель, стеновых контактов нет. Кривая и два теста **полностью удалены** из исходников, дерево кода вновь соответствует коммиту `c233466`. Эксперимент показывает, что геометрически безопасная центральная линия ещё не гарантирует безопасного следования MPC и лучшей фактической скорости. Следующая итерация должна измерять исполненную траекторию и зазор роботов, а не только форму публикуемого пути.
- Образ после удаления кривой пересобран. Обнаружено, что в Docker-контекст попадали `__pycache__` и `.pyc` из `src/`, поэтому неизменённые исходники могли повторно запускать многоминутную сборку. Добавлен `.dockerignore` для этих файлов, `results/` и каталогов Git. Первая сборка с новыми исключениями успешно компилировала все пакеты; **повторная без изменения исходников заняла 4,71 с** (`/usr/bin/time -f 'elapsed=%e seconds' sh -c 'helm build duel > /tmp/hsl-build-cache.log 2>&1'`) и использовала кеш. Это изменение контекста не влияет на алгоритм движения.
- `benchmarks/trace_motion.py` уточнён: подписка на transient-local `/match/allowed` ограничивает статистику активным матчем; выводятся медиана/p90 длины локального отрезка и ошибки направления, а также медиана угла, при котором шлюз обнулил положительную команду MPC. Первый прогон до исправления QoS (`results/series-20260928T130617Z/`) дал пустую трассу, потому что подписчик пропустил уже опубликованное разрешение движения; это не доказательство отсутствия движения. После исправления диагностический повтор (`results/series-20260928T131012Z/`, сценарий 2, seed 1) закончился поимкой за 68,4 с, 0 контактов, скорости 0,144/0,149 м/с. За 100 с реального времени трасса включила 640 активных команд каждого: шлюз остановил положительный выход MPC в 238/253 командах (37,2/39,5%), нулевой выход MPC почти отсутствовал; медианы длины локального отрезка 0,205/0,409 м, p90 0,609/0,803 м, медианы ошибки направления 1,288/0,997 рад. Глобальных резких смен 1/8. Это указывает на короткие безопасные прямые и необходимость разворота к ним как главный измеренный ограничитель средней скорости; без проверенной траектории корпуса ослаблять защитный разворот нельзя. Трасса выводилась в консоль и не сохранена отдельным файлом, сводные числа здесь; в следующий раз перенаправлять вывод трассы в файл результатов.
- Вторая **отклонённая** проба: `local_guidance` продолжал поиск после первого недоступного промежуточного узла глобального маршрута и брал более дальнюю точку, если прямая к ней проходила геометрическую проверку. Синтетический тест подтвердил возможность такого обхода. Серия `results/series-20260928T132157Z/` (сценарий 2, seed 0–2) дала 1 цель/2 поимки, 0 контактов и средние 0,185/0,181 м/с против 0,174/0,174 на прежней версии, но этого мало для порога; две попытки запустить трассу вручную опоздали к матчам. Поэтому в `run_duel_series.py` добавлена опция `--trace`: трассировщики стартуют **до** разрешения движения и записываются рядом с результатами.
- Заезд новой версии с автоматической трассой `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 1 --active-s 90 --scenario 2 --trace`, `results/series-20260928T133154Z/`: поимка через 86,7 с, средние 0,150/0,136 м/с; **2 стеновых контакта стража**, роботных контактов нет. Трассы `00-trace-first.json` и `00-trace-second.json` содержат 883/867 активных команд: медианы длины локального пути выросли до 0,314/0,512 м, p90 до 1,007/1,352 м, но шлюз остановил 320/339 команд с положительным MPC; выигрыша в средней скорости нет. Новая логика и её тест **удалены**, безопасная базовая версия собрана из кеша за 4,95 с; остаётся только полезная опция трассировки. Следующая задача — понять, где фактически проходит корпус относительно стены и почему MPC отклоняется от проверенной центральной линии; не увеличивать скорость или длину пути вслепую.
- Проверочная серия на базовом планировщике с автоматической трассой `results/series-20260928T134022Z/`, сценарий 2 seed 0–2: seed 0 дал **тайм-аут 90 с** без достижения задач при скоростях 0,111/0,128 м/с; seed 1/2 — поимки через 11,8/43,6 с, целей исследователя нет. Всего 0 контактов, средние по ролям 0,127/0,178 м/с. В тайм-ауте исследователь имел `NO_GLOBAL_PATH` 14% времени, его медиана локального пути 0,207 м; у стража `OK` 98,8%, но шлюз обнулил 398 из 901 активных команд, медиана локального пути 0,487 м. Следовательно, низкая скорость имеет и проблему построения глобального маршрута у исследователя, и слишком частые развороты стража при доступном пути.
- Ещё одна **отклонённая** проба контроллера: при ошибке направления менее 1 рад шлюз мог пропустить до 0,16 м/с поступательно, только если прогнозируемая на 0,8 с дуга корпуса оставалась свободной по свежим точкам скана; иначе сохранялся разворот на месте. Тесты (93) прошли. Серия `results/series-20260928T135225Z/`, те же seed 0–2 и 90 с, дала 1 цель/2 поимки, 0 контактов, но средние 0,156/0,185 м/с, у каждой роли 2 из 3 матчей ниже 0,2. По сопоставимым заездам уменьшение остановок шлюза не подтвердилось, а дополнительная обработка скана усложнила цикл управления. Код и тест пробы **полностью откатили**, базовый образ пересобран из кеша за 5,24 с. Не повышать предельную скорость; следующий полезный шаг — разбирать причины коротких локальных отрезков и `NO_GLOBAL_PATH` на тайм-ауте, затем предложить траекторию, которую MPC действительно может безопасно выполнить.
- `benchmarks/planner_probe.py` обновлён под текущий планировщик и запускается до движения через `run_duel_series.py --probe-status NO_GLOBAL_PATH`. В независимом матче `results/series-20260928T140058Z/` (сценарий 2, seed 0) зонд поймал отказ исследователя при `EVADE`: собственная поза (−2,162; 2,157), соперник (−1,738; 1,213), цель около (−3,0; 3,45), стартовая и целевая клетки свободны, но A* пуст. Матч завершился поимкой за 77,1 с, скорости 0,132/0,142 м/с, 0 контактов. Следующий повтор того же seed `results/series-20260928T140507Z/` цели достиг без `NO_GLOBAL_PATH`, поэтому отказ непостоянен.
- Расширенный зонд поймал второй отказ в `results/series-20260928T140750Z/00-probe-first.json`: поза (−2,153; 2,079), соперник (−2,086; 1,086), собственная клетка свободна, соседняя **слева свободна и прямой переход к ней безопасен**, но A* к цели и первому frontier пуст, первые 30 альтернативных frontiers недостижимы, а `recovery_heading` вернул `None`. Причина остановки локализована: поиск восстановления пробовал минимум 0,2 м, тогда как доступен более короткий выход. В этом матче страж поймал исследователя за 57,1 с, 0 контактов, скорости 0,147/0,159 м/с.
- `recovery_step` теперь выбирает вместе **направление и проверенную длину** из 0,55/0,3/0,2/0,15/0,1 м; прежний watchdog ошибочно ставил цель на 0,6 м по направлению, проверенному иногда только на 0,2 м. Тест тесного кармана подтвердил выход при длине не больше 0,15 м. Образ собран, серия `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 0 --active-s 90 --scenario 2 --trace --probe-status NO_GLOBAL_PATH`, `results/series-20260928T141601Z/`: 2 поимки/1 цель, **0 контактов**, у обоих планировщиков 0% `NO_GLOBAL_PATH`, скорости по ролям 0,178/0,188 м/с. У исследователя в seed 2 `RECOVERY_ESCAPE` 5,2%, но длинного простоя не было; 2 из 3 матчей каждой роли ниже 0,2. По другой стартовой позиции `results/series-20260928T142310Z/`, сценарий 3 seed 3–4: 1 цель/1 поимка, 0 контактов и 0% `NO_GLOBAL_PATH`, скорости по ролям 0,160/0,168 м/с. В seed 3 оба достигли 0,206/0,219 м/с, в seed 4 — лишь 0,113/0,116. Промежуточный порог **не подтверждён**, баланс и 20 полных матчей тоже. Следующая итерация: сократить частые развороты и удержать безопасность, отдельно разобрать медленный seed 4 сценария 3 и короткий seed 1 сценария 2; не повышать скорость без проверки исполнения MPC около стен.

## Текущий цикл: плавность и отсутствие столкновений (28.09)

- Пользователь показал пустой RViz2 при запуске `duel`, остановку стража у стены и изломы даже на прямой. В конфигурации `duel.rviz` после `Window Geometry` обнаружены ошибочные повторные строки камеры; их удалили, вернули верхний обзор. Проверка живого RViz2 после перезапуска: панель Displays, карта, обе позиции, скан и слои путей отображаются (снимок `/tmp/hsl-rviz-fixed.png`).
- Диагностический матч сценария 2 после предыдущего исправления границ: **поимка стражем за 16,4 с**, оба робота остались внутри арены (246–247 отсчётов; минимальные зазоры центров до её края 0,200/0,201 м), но у каждого был 1 контакт со стеной. Средняя поступательная скорость исследователя/стража 0,195/0,235 м/с, `NO_LOCAL_PATH` 2,3/1,2% времени. Результаты: `results/duel-20260928T061816162377Z.json`, `results/opponent/latest.json`, `results/latest_outcome.json`. Это не подтверждает нужную плавность и ноль контактов.
- Причина частых изломов: прежний локальный планировщик каждые 0,2 с публиковал новую дугу из дискретных угловых скоростей; поперечный MPC при каждом изменении геометрии сбрасывал индекс ближайшей точки. Теперь `local_guidance` выбирает дальнюю видимую точку глобального маршрута, проверяет каждые 5 см просвет до стен/границы/соперника и выдаёт прямой отрезок с равномерными точками. Перед большим отклонением от нового направления шлюз сначала разворачивает робота на месте. При отсутствии прохода выбирается безопасный поворот в сторону большего зазора; watchdog после 4 с без поступательного прогресса задаёт короткий восстановительный манёвр. Новый локальный способ ещё **не подтверждён в Gazebo**.
- По просьбе пользователя скорость пока ограничена 0,2 м/с в продольном MPC и шлюзе. Критерии текущего этапа: прямая без изломов, плавные повороты и отсутствие столкновений; скорость повышать только после этого. Стек `/Path → /cmd_vel` использует `swarm_controller` (продольный и поперечный MPC) плюс защитный `hsl_mpc_gate`, а не `debug_follower`. Предсказание перехвата применяет известный предел скорости **своего** робота и скорость соперника, оценённую по последовательным видимым положениям. Скрытая скорость Gazebo больше не передаётся напрямую в навигацию; при первом наблюдении или большом разрыве оценка сбрасывается. Будущая скорость соперника не предполагается известной.
- Первый повтор сценария 2 с прямым локальным путём и пределом 0,2 м/с: поимка за **64,9 с**, оба внутри арены, **0 контактов со стенами и друг с другом**. Однако средняя скорость за матч 0,111/0,123 м/с, хотя при поступательном движении около 0,197/0,192 м/с; у исследователя 24,2% времени `NO_LOCAL_PATH` в режиме `EVADE`, у стража 10,8% `RECOVERY_TURN`. Поэтому нулевые контакты в одном матче ещё не означают решения: путь для выхода из зоны соперника был слишком строгим, а восстановление начиналось поздно.
- Следующая правка допускает короткий объезд внутри зоны опасности при уже близком сопернике, сразу задаёт короткий безопасный поступательный выход при потере локального пути и пробует восстановление на 0,3 м, если 0,55 м недоступны. **91 тест**, `compileall`, `git diff --check`, проверка YAML RViz2 прошли. Исходники локально подменены в двух контейнерах планировщика для нового матча сценария 3; после проверки требуется собрать окончательный образ и повторить без ручной подмены. До этого ноль столкновений и исходный баланс не заявлять.
- Сценарий 3 после этой правки: поимка стражем через **60,4 с**, 672 отсчёта позы каждого робота внутри арены, минимальные запасы центров до края 0,355/0,387 м, **0 столкновений**. Скорость за матч 0,131/0,151 м/с; поступательная активность 0,842/0,990, RMS углового ускорения 1,051/1,444 рад/с², `NO_LOCAL_PATH` у обоих 0%, `RECOVERY_ESCAPE` 1/2% времени. Исследователь всё ещё стоял 15,5% времени из-за `NO_GLOBAL_PATH` (в основном при `GOAL`). В исходники добавлен безопасный короткий выход и для этого случая; его нужно проверить в следующем матче и затем пересобрать образ. Результаты: `results/latest.json`, `results/opponent/latest.json`, `results/latest_outcome.json` на момент 28.09 09:48 МСК.
- Сценарий 1 с последней правкой, код локально подменён в двух контейнерах планировщика: поимка за **18,6 с**, 234–235 отсчётов каждого робота внутри арены, **0 столкновений**. Минимальный запас центров до края 0,401/0,498 м; средние скорости 0,148/0,156 м/с, планировщик `OK` 98,5/95,7% времени, RMS углового ускорения 1,229/1,161 рад/с². В этом коротком матче резервный путь при `NO_GLOBAL_PATH` не понадобился; следовательно, сама новая ветка ещё не подтверждена Gazebo. Начата пересборка образа для проверки без подмены файлов.
- Образ с этой правкой собран и проверен независимой серией `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 0 --active-s 90 --scenario 2`, `results/series-20260928T065827Z/`, ревизия `78be21e+dirty.27a8973d5f4b`: seed 0/2 — поимки за 60,9/47,8 с, seed 1 с перестановкой ролей — фактическая цель за 48,6 с; **0 контактов у обоих во всех трёх матчах**, сбоев нет. Средние скорости 0,148/0,158 м/с при физическом пределе 0,2, поступательные доли 0,771/0,819, активность с разворотами 0,897/0,989, `OK` 0,899/0,983. Seed 0 у исследователя имел `NO_GLOBAL_PATH` 23,8% времени при `GOAL`, тогда как seed 1 — 99,6% `OK`; успех зависит от ситуации. Следующая гипотеза — в тесном кармане безопасен выход всего на 0,2 м, а старый поиск пробовал минимум 0,3 м. Добавлен шаг 0,2 м, тест тесного кармана; **92 теста** прошли. Требуется повтор seed 0 на том же сценарии, затем окончательная сборка и повтор без ручной подмены.
- Ручной повтор сценария 2/seed 0 с новым шагом 0,2 м и исходниками, подменёнными только в контейнерах планировщиков: поимка за **26,3 с**, 0 контактов; у исследователя `OK` 97,5%, `RECOVERY_ESCAPE` 1,4%, **`NO_GLOBAL_PATH` 0%** (до изменения 23,8% в независимом матче). У стража `OK` 97,3%, `RECOVERY_ESCAPE` 1,5%. Скорости 0,151/0,139 м/с, RMS углового ускорения 1,494/1,323 рад/с². Время исхода меняется между пусками даже при том же seed из-за расписания автономных узлов. Окончательный образ с шагом 0,2 м пересобирается; нужен повтор без подмены.
- Окончательный образ с коротким восстановлением проверен без подмены: `python3 benchmarks/run_duel_series.py --runs 2 --start-seed 0 --active-s 90 --scenario 2`, `results/series-20260928T071344Z/`, ревизия `78be21e+dirty.7902d7c3896f`. Seed 0 — тайм-аут 90 с (без фактической цели и поимки), seed 1 с перестановкой ролей — поимка за 15,4 с; **0 контактов у обоих во всех матчах**. Скорость за матч 0,097/0,148 м/с по ролям; у исследователя в seed 0 `NO_GLOBAL_PATH` всего 0,9%, `RECOVERY_ESCAPE` 8,7%, активность 93,3%. Это устранило длительный простой, но не гарантировало быстрое достижение задачи. После пяти независимых матчей с нулём контактов на пределе команды 0,2 м/с пользовательский порядок разрешает пробу большей скорости; предел продольного MPC, шлюза и модель достижимости своего робота для перехвата подняты до **0,3 м/с**. Эта версия ещё требует сборки и проверки контактов/плавности.
- Серия с пределом 0,3 м/с: `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 0 --active-s 90 --scenario 2`, `results/series-20260928T072320Z/`, образ `78be21e+dirty.7f6b43d6e78b`: 2 поимки за 30,0/29,0 с, 1 фактическая цель за 47,4 с, **0 контактов во всех трёх матчах**. Средние скорости по ролям 0,149/0,150 м/с — практически не выросли относительно 0,148/0,158 на пределе 0,2. `OK` снизился до 0,835/0,837 из-за `RECOVERY_ESCAPE` около 13–17% времени, хотя двигательная активность 0,975/0,980. Найдена конкретная геометрическая ошибка: восстановительная цель считалась достигнутой ближе 0,06 м, а `local_guidance` отбрасывал цель ближе 0,12 м; между порогами повторно запускался выход. Для восстановления снижен порог отбора точки до 0,04 м, локальный тест добавлен (**92 теста**). Пока нет Gazebo-проверки этой правки; скорость дальше не повышать до неё.
- Ручной повтор сценария 2/seed 0 на пределе 0,3 м/с с этой правкой, исходники двух планировщиков локально подменены в контейнерах: поимка за **47,2 с**, **0 контактов**, скорость 0,183/0,170 м/с. `RECOVERY_ESCAPE` сократился до 0,6/2,5% против 16,6/16,9% в первом матче прежнего образа того же сценария; `OK` вырос до 98,6/96,6%, двигательная активность 98,8/98,9%, RMS углового ускорения 1,535/1,413 рад/с². Следующий шаг — пересобрать образ и запустить независимые seed без подмены. Из-за стохастического расписания одного парного повтора недостаточно для причинного вывода о скорости, но устраняемый зазор порогов подтверждён локальным тестом.
- Окончательный образ с исправленным остатком восстановительного пути проверен независимой серией `python3 benchmarks/run_duel_series.py --runs 3 --start-seed 0 --active-s 90 --scenario 2`, `results/series-20260928T073805Z/`, ревизия `78be21e+dirty.ebb86e53a121`: фактическая цель seed 2 за 46,0 с, поимки seed 0/1 за 32,1/13,8 с, **0 контактов в шести отчётах**. Доступность планировщика по ролям 0,979/0,972 против 0,835/0,837 до исправления; доля двигательной активности 0,976/0,971. Средняя скорость за матч 0,166/0,186 м/с; в первом, более длинном матче 0,219/0,197 м/с, скорость во время движения 0,283/0,263 м/с. Короткий матч seed 1 с начальным разворотом снизил агрегат исследователя. Баланс на трёх seed — 1/2, а не доказанные 50/50. Начата пара матчей сценария 3 на той же скорости перед новым повышением предела.
- Проверка другого старта: `python3 benchmarks/run_duel_series.py --runs 2 --start-seed 3 --active-s 90 --scenario 3`, `results/series-20260928T074333Z/`, тот же образ. Страж поймал исследователя через 26,5/40,5 с, в обоих матчах **0 контактов**. Планировщик `OK` 0,981/0,960 по ролям, активность 0,982/0,984. Но исследователь двигался поступательно лишь 55,5% времени, разворачивался 42,7%, средняя скорость 0,100 м/с; в режимах `EVADE` и `GOAL` развороты составили 17,7–20,6 и 19,2–27,8 процентных пункта матча соответственно. Пока рано повышать предел свыше 0,3: сначала измерить частоту смены направления маршрута и причины лишних поворотов на сценарии 3, сохраняя ноль контактов.
- Диагностический повтор сценария 3/seed 0 на том же образе: исследователь **достиг площадки за 78,0 с**, 0 контактов, скорости 0,169/0,176 м/с, развороты 25,4/23,3% времени. `benchmarks/trace_motion.py` за 100 с реального времени увидел только **одну резкую смену направления глобального пути при смещении <0,2 м** и ни одного переворота цели поворота локального пути; 24,4% команд исследователя были остановлены шлюзом из-за большого угла к следующему прямому отрезку. Следовательно, высокая доля разворотов в основном связана с фактическими поворотами маршрута и уклонением, а не с частым прыганием между равнозначными путями. Проба предела 0,45 м/с была подготовлена, но после уточнения пользователя отложена до уверенного достижения средней скорости 0,2 м/с без контактов и лишних манёвров; рабочий предел оставлен 0,3 м/с. Скорость соперника по-прежнему оценивается только по наблюдениям.

## Текущий цикл: локальный путь, перехват, уклонение и сценарии (27.09)

- Пользователь уточнил приоритеты: глобальный путь выглядит приемлемо, локальный даёт лишние манёвры, опасное сближение со стеной и недостаточную плавность; препятствие, не отражённое на карте, должно вызывать обход/смену пути. Стражу нужна прогнозная точка перехвата движущегося исследователя, исследователю — явное уклонение и безопасный обход к цели. Требуются автоматический RViz2 в графическом `duel` (в headless без GUI) и несколько сценариев с разными стартами стража и командами `start_match1`, `start_match2` и т. п. Сценарные координаты пока выбираются из свободных точек текущего лабиринта; вопрос об обязательных координатах задан пользователю.
- Реализованы проверка зазора до стен по всей локальной траектории, штраф резких смен угловой скорости, сброс пути при новых препятствиях и watchdog отсутствия поступательного прогресса с повторным выбором коридора. Страж использует ограниченное по времени предсказание точки встречи из скорости видимого исследователя; исследователь при угрозе выбирает направление от стража и штрафует опасную часть пути к цели. Это пока подтверждено лишь локальными тестами, не исходами Gazebo.
- Добавлен RViz2 в профиль `duel`: при наличии `DISPLAY` показывает карту, две модели, глобальные/локальные пути и сканы; пакетные серии задают `HSL_RVIZ_ENABLED=false`. Пользователь подтвердил правило GUI/headless. Три сценария используют старты стража в `maze.world` (2.5,2.5), (3.5,1.5), (1.5,2.5) при прежнем старте исследователя; команды `helm start_match1/2/3` пересоздают мир и запускают свежий матч. Полигон площадки передаётся обоим decision manager и судье согласованно с позой. Эти старты предварительно проверены растеризацией SDF и связностью свободных клеток с учётом радиуса 0,25 м; реальный Gazebo-тест ещё идёт.
- После правок `python3 -m pytest -q tests/test_navigation.py helm_launch/tests/tests.py` — 87 тестов, `compileall`, `git diff --check` и Compose config — успешно. Образ пересобран с RViz2. Контрольная серия `python3 benchmarks/run_duel_series.py --runs 3 --active-s 120 --scenario 1`, seed 0–2, роли менялись, `results/series-20260927T204849Z/`: 2 фактические поимки (44,3 и 24,6 с), 1 достижение цели (33,6 с), без тайм-аутов и сбоев. Средняя скорость за полный матч 0,253/0,295 м/с для исследователя/стража, доля движения 0,557/0,653, разворотов 0,376/0,322, контактов 3,67/3,0, планировщик `OK` 0,947/0,988. Контрольная прежняя серия 4×120 с дала 0,202/0,274 м/с и 4,75/4,0 контактов; сравнение ориентировочное, так как размер/seed серий отличаются. Скорость исследователя во всех трёх матчах и стража в двух ниже 0,3 м/с, контакты остаются. В первом матче p95 вычисления планировщика достиг 261 мс, 49/190 итераций превышали 200 мс; новое измерение зазора по всей локальной траектории стало вероятным источником нагрузки. Следующий цикл — локализовать вычислительные затраты, снизить контакты, затем повторить те же seed и проверить сценарии 2/3.
- Проверена гипотеза о цене поиска ближайшей стены: вместо перебора сырых точек в каждой пробе локальной траектории и A* построен `cKDTree` при обновлении наблюдений; при отсутствии SciPy сохранён прежний путь. В образе подтверждено наличие `cKDTree`, на хосте запасной путь прошёл те же 87 тестов. Повтор `python3 benchmarks/run_duel_series.py --runs 3 --active-s 120 --scenario 1`, те же seed 0–2, `results/series-20260927T210128Z/`: снова 2 поимки и 1 цель, без тайм-аутов. Средние скорости 0,255/0,286 м/с, контакты 1,0/1,67; p95 планировщика в шести отчётах 96–123 мс против 226–370 мс до оптимизации, хотя отдельные максимумы около 1 с остаются из-за поиска пути. Контакты заметно сократились в этой малой стохастической выборке, но причинность требует повторов; порог скорости по-прежнему не достигнут в 3/3 и 2/3 матчей. Следующий шаг — проверить сценарий 2 и автозапуск RViz2, затем изучить оставшиеся точки контакта и затраты на развороты.
- Проверка пользовательской команды `GAZEBO_HEADLESS=true helm start_match2`: команда действительно пересоздала мир, запустила оба контура и RViz2 при наличии `DISPLAY`. В RViz2 видны карта, оба робота и скан; процесс `rviz2` работает, карта прорисовалась, хотя в логе есть предупреждение GLSL. Заезд завершился фактическим достижением цели исследователем через 18,0 с, 0,334/0,296 м/с, без контактов. **Пользователь заметил робота вне лабиринта на RViz2.** Проверка ROS после завершения заезда показала стража в `(4,964; −1,267)` в `map`, за пределами карты (граница `y=−1,0`, `x≈4,365`). Это реальный дефект: свободные клетки из LiDAR добавлялись за конечной границей известной карты, и `SEARCH` мог проложить туда путь. Матч остановлен через `helm stop_match`.
- Исправление для следующего теста: при наличии `OccupancyGrid` её геометрические границы передаются в `VoxelWorld`; наблюдённые сканом свободные клетки, A* и локальная траектория ограничены границей с запасом 0,1 м сверх радиуса планировщика. Без статической карты прежнее исследование наблюдённого пространства остаётся доступным. Тест воспроизводит луч LiDAR за край карты и проверяет, что поисковая точка и путь туда не появляются; 88 локальных тестов проходят. Образ пересобирается, **повтор сценария 2 после исправления ещё не проверен**.
- Повтор после этого исправления выявил более глубокую ошибку в конфигурации: внешний прямоугольник стен `maze.world` простирается по миру примерно от `x=−3` до `x=3` и от `y=0` до `y=5`. В прежнем сценарии 2 страж **появлялся уже за стеной** при `x=3,5`; первый робот стартовал перед нижним входом при `y=−0,18`. Карта с отступом вокруг стен размечала внешнюю область как свободную, поэтому проверка только на занятость ячейки и связность от прежнего старта ошибочно сочла эти позы допустимыми. На повторе оба робота оказались в свободных ячейках карты около `y=−0,73`, но за внешним контуром стен. Вывод: границы прямоугольника OccupancyGrid недостаточны; нужна именно область игры внутри внешних стен. Это исправляет прежнее утверждение в начале журнала, что старты 2/3 были валидированы.
- Конфигурация изменена: первый робот теперь стартует внутри в мире `(−0,34; 0,4)` через отдельный `DUEL_SPAWN_Y`, не меняя одиночный Gazebo. Старт стража: сценарий 1 `(2,5; 2,5)`, 2 `(1,5; 1,5)`, 3 `(−2,5; 1,5)`; все три имеют расстояние до ближайшей стены не менее 0,477 м и лежат в связной внутренней области от нового старта при инфляции 0,25 м. `benchmarks/scenarios.py` формирует площадки и `DUEL_ARENA_BOUNDS` в кадре `map`; оба планировщика получают этот параметр и ограничивают свободные точки, A* и локальную траекторию внутренним прямоугольником стен с запасом. Это геометрия сценария, не зашитая в алгоритмы. Требуется пересборка и реальная проверка новых стартов, траекторий, RViz2 и исходов; прежние серии с внешними стартами не использовать для финальной оценки.
- Первый запуск новых стартов остановлен до движения: ROS отклонил `arena_bounds` из-за того, что пустой массив по умолчанию получил тип `BYTE_ARRAY`, а Compose передал `DOUBLE_ARRAY`; оба процесса планировщика завершились. Объявление исправлено типизированным массивом `float`, новая сборка подтвердила два живых планировщика и значения `[-2.66,-0.4,3.34,4.6]` в обоих пространствах имён.
- Следующий реальный `GAZEBO_HEADLESS=true helm start_match2` также был **остановлен вручную как неудачный диагностический заезд** после 106,1 с симуляции: оба робота имели `NO_LOCAL_PATH` более 91% времени, средние скорости 0,017/0,015 м/с; один контакт со стеной у исследователя. Причина по текущим позам: контроллер увёл центры на 5–9 см за заданный запас от внешней границы (при этом внутренняя область лабиринта не покинута), а локальный планировщик отвергал даже поворот и шаги обратно внутрь. Добавлено разрешение манёвров, увеличивающих расстояние до границы, по аналогии с выходом из консервативно занятой стартовой ячейки; тест воспроизводит эту ситуацию. **89 тестов** проходят. RViz2 теперь направлен на центр всего лабиринта сверху; повтор после сборки ещё требуется. Прежние результаты с внешними стартами неприменимы к оценке новых стартов.
- Последняя контрольная серия после оптимизации A*: `python3 benchmarks/run_duel_series.py --runs 4 --active-s 120 --build`, seed 0–3, `results/series-20260927T173014Z/`, образ `78be21e+dirty.d51d3820eb97`. Исходы: seed 0 — тайм-аут без цели и поимки; seed 1 — цель за 24,9 с; seed 2/3 — поимки за 23,5/62,3 с. Средняя скорость 0,202/0,274 м/с, активность 0,702/0,973, контакты 4,75/4,0. В seed 0 исследователь прошёл только 7,2 м за 120 с (0,060 м/с): планировщик сообщил `OK` 99,7% времени, но робот физически простаивал в `GOAL` 77,8% времени и получал ненулевую команду лишь 22,3%. После 24,2 с новых контактов не было. Это указывает на бесполезный локальный путь/промежуточную цель, а не на ограничение MPC или механическое заклинивание. A* в этом матче укладывался максимум в 205,7 мс; ускорение поиска не устранило стагнацию.
- Следующий шаг: изменить локальный перебор и проверку прогресса, чтобы при новых препятствиях сбрасывать устаревший путь и выбирать другой. Затем проверить перехват и уклонение, сценарии и RViz2 в реальной дуэли; повторить 20 матчей по 360 с лишь после устранения стагнации. Прежняя 20-матчевая попытка была остановлена после 10 матчей из-за критического простоя (подробности ниже).

## Итерация: независимая серия двух автономных стеков (27.09)

- Контрольная серия ускоренного профиля: `python3 benchmarks/run_duel_series.py --runs 4 --active-s 120`, seed 0–3, `results/series-20260927T162740Z/`, образ `78be21e+dirty.f4bc43c63573`. Исходы: поимка seed 0 через 26,4 с; достижения цели seed 1/2/3 через 26,6/89,3/57,7 с. Средняя скорость обеих ролей 0,243/0,243 м/с, доля поступательного движения 0,495/0,545, разворотов 0,373/0,332, общая активность 0,869/0,878, контактов 4,5/5,25. Для сравнения предыдущая серия с более спокойным профилем дала 0,250/0,270 и 1/3 исходов. Повышение ограничений скорости не дало устойчивого ускорения и сместило малую выборку к исследователю; **возвращены** `v_ref=0.55`, `v_cmd_max=0.65`, `a_lat_max=0.12` и соответствующие лимиты decision manager. Контроллерный эксперимент не используется как финальная настройка.
- В seed 0 ускоренной серии исследователь большую часть первых 26,4 с находился в `GOAL` (60,4%) и `EVADE` (39,3%), но `planner_ok_fraction` был лишь 0,614, средняя скорость 0,150 м/с. В seed 2 страж имел `planner_ok_fraction=0,998`, но двигался поступательно 34,8% времени и разворачивался 23,4%; остальные примерно 42% времени не было заметной физической активности. Высокое `OK` само по себе не гарантирует полезную команду. Следующие направления разбора: построение выхода из зоны соперника в узком проходе для исследователя и нулевая команда при `SEARCH`/`PURSUE` для стража. Сначала проверить причины в диагностике, затем менять алгоритм.
- Гипотеза для отказов при `EVADE`: если страж уже находится внутри защитного радиуса, прежний A* разрешал только каждый следующий шаг **строго дальше** от стража. В коридоре прямой выход может быть закрыт стеной, а безопасный обход сначала чуть приближает робота к сопернику. Добавлен тест с таким коридором: прежний алгоритм возвращал пустой маршрут, новый строит обход. В A* внутри уже нарушенного защитного радиуса разрешён дорогой по стоимости краткий обход, при этом после выхода повторный вход запрещён и сохраняется минимальная дистанция до стража; локальный перебор согласован с этим правилом. Газебо-проверка: `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 0 --active-s 120 --build`, `results/series-20260927T163905Z/`, образ `78be21e+dirty.200c8b68e660`. Исследователь достиг цели через 91,8 с; скорости 0,255/0,185 м/с, `planner_ok_fraction` 0,995/0,999, контакты 10/6, активность 0,987/0,704. В прежнем раннем проигрыше seed 0 у исследователя было `planner_ok_fraction=0,614`, но это два неидентичных по трассе матча, поэтому улучшение ещё требует серии. Из 91,8 с страж около 29,6% времени не двигался заметно, несмотря на почти постоянный `OK`.
- Для диагностики стража добавлено сравнение фактического движения Gazebo и выходной `/cmd_vel`: `commanded_motion_fraction`, `commanded_while_still_fraction`, а также `idle_by_behavior_fraction`. Оно позволит отделить остановку контроллера/шлюза от физического упора при ненулевой команде и от режима без движения. **81 тест**, `compileall`, `git diff --check` проходят; метрика ещё не проверена в Gazebo. Следующий шаг — повтор seed с пересборкой и анализ этих полей, затем исправление обнаруженного источника простоя.
- Диагностический повтор seed 2 с новыми метриками: `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 2 --active-s 120 --build`, `results/series-20260927T164630Z/`, образ `78be21e+dirty.b739d0478c63`. Страж поймал исследователя через 22,4 с; скорости 0,266/0,269 м/с, контактов 0/2, общая активность 0,941/0,960. Метрики работают: доля ненулевой команды 0,946/0,973, ненулевая команда при физической неподвижности 0,008/0,013, простой по режимам у стража `SEARCH=0,013`, `PURSUE=0,009`. В этом коротком матче длительного простоя не возникло, поэтому его причина ещё не установлена. Следующий шаг — длинная серия на этом образе, анализ матчей с низкой активностью и проверка исходов на 360-секундном лимите.
- Попытка полной оценочной серии: `python3 benchmarks/run_duel_series.py --runs 20 --active-s 360`, seed 0–19, перестановка ролей, `results/series-20260927T165129Z/`, образ предыдущего пилота. **Сознательно остановлена после 10 завершённых матчей** при подготовке seed 10, потому что seed 8 выявил критическое застревание; контейнеры очищены `helm clean duel`. Из 10: 6 фактических достижений цели и 4 поимки, без тайм-аутов и технических сбоев. Средняя скорость за активный матч 0,252/0,244 м/с, скорость только во время поступательного движения 0,449/0,435 м/с, суммарная активность 0,863/0,894, доступность планировщика 0,884/0,991; ниже строгого порога 0,3 м/с — 8 исследователей и 10 стражей. Эти 10 матчей не удовлетворяют требованию финальных 20; серия нужна для диагностики.
- Критический seed 8 этой серии: страж поймал исследователя лишь через **183,1 с**, скорости 0,043/0,049 м/с. У исследователя `planner_ok_fraction=0,166`, ненулевая команда 0,151 времени; 94,9% времени он пытался идти к цели. У стража планировщик сообщал `OK` 99,8% времени, но команда была ненулевой 19,4% времени, а физический простой в `PURSUE` занял 80,7% матча. Контактов всего 3/4, ненулевая команда при неподвижности 0,2/0,4%; основной дефект — отсутствие полезной команды и пути, а не физическое заклинивание или слишком медленный MPC. Гипотеза: когда целевая точка преследования недоступна и заменяется ближайшим фронтиром у самого робота, планировщик повторно выдаёт короткий путь/поворот и статус `OK`; её необходимо подтвердить и устранить.
- Изменение для этой гипотезы: при недоступной цели `reachable_target` сначала выбирает фронтир не ближе 0,6 м от робота; тот же фильтр используется при запасном выборе после отказа A*. Если дальних фронтиров нет, прежний выбор остаётся запасным, чтобы не терять путь полностью. Добавлен тест случая, когда рядом с роботом и дальше есть две свободные граничные клетки: теперь выбирается дальняя. Это защищает от одной конкретной причины статуса `OK` без движения, но не доказывает, что именно она вызвала весь seed 8. **83 теста**, `compileall`, `git diff --check` прошли. Повтор: `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 8 --active-s 360 --build`, `results/series-20260927T171948Z/`, образ `78be21e+dirty.e750765992cc`. Исследователь достиг цели за 40,0 с; скорости 0,276/0,276 м/с, активность 0,961/0,988, доступность планировщика 0,993/0,998, ненулевая команда стража 0,995. Длительного простоя в этом матче не было, но стохастический повтор seed не доказывает исправления причины. Запущена проверка seed 8–11 на том же образе с полным лимитом 360 с: `results/series-20260927T172318Z/`.
- Контрольная серия seed 8–11: `python3 benchmarks/run_duel_series.py --runs 4 --start-seed 8 --active-s 360`, `results/series-20260927T172318Z/`, тот же навигационный образ. **Четыре поимки** через 32,9/15,1/23,1/15,2 с, ни одной цели/тайм-аута; средние скорости 0,256/0,261 м/с, активность 0,863/0,970, доступность планировщика 0,886/0,985, контакты 1,75/1,5. Длительное застревание seed 8 не повторилось, но баланс по этой малой выборке явно в пользу стража. В seed 8 исследователь имел `NO_GLOBAL_PATH` 10,5% времени и `STALE_INPUT` 7,3% (преимущественно `EVADE`); два расчёта A* длились более 2 с, максимум 3,52 с. Поэтому оптимизирован A*: уже обработанные клетки не расширяются повторно, предел расширений снижен с 30000 до 8000 (демонстрационная карта намного меньше). **83 теста**, `compileall`, `git diff --check` прошли, но новое ограничение поиска **ещё не проверено в Gazebo**. Следующий шаг — серия после пересборки, сравнение задержек и частоты пустых путей.
- Во время серии после первых трёх матчей обнаружен эпизод `planner_ok_fraction=0,765` у исследователя при 34,9% времени в `EVADE` (seed 1, поимка на 45,8 с); ненулевая команда 67,1%, активность 66,9%, то есть потеря возникает до контроллера. В исходники наблюдателя **после сборки текущего образа** добавлены `planner_status_fraction` и `planner_failure_by_behavior_fraction`, чтобы в следующем образе различать `NO_GLOBAL_PATH`, `NO_LOCAL_PATH`, `NO_TARGET_OR_FRONTIER` и задержанные входы по режимам. Это изменение не влияет на алгоритмы текущей серии. **82 теста**, `compileall`, `git diff --check` проходят; новая разбивка ещё не проверена в Gazebo.
- Промежуточный срез той же серии после seed 0–5: 3 фактических достижения площадки, 3 поимки, без тайм-аутов и технических сбоев. Средняя скорость за весь активный матч 0,263/0,262 м/с, во время поступательного движения 0,453/0,433 м/с; доля поступательного движения 0,581/0,607, разворотов 0,342/0,368, суммарная активность 0,923/0,975, доступность планировщика 0,949/0,990. По строгому определению скорости порог 0,3 не достигнут; уточнение пользователя по его смыслу ожидается. Это только промежуточный срез, итоговые выводы делать после 20 матчей.

- Серия после добавления штрафа за близость к стене: `python3 benchmarks/run_duel_series.py --runs 4 --active-s 120`, seed 0–3, `results/series-20260927T160221Z/`, образ предыдущего пилота. Исходы: страж поймал исследователя в seed 0/2/3 через 90,7/16,2/56,1 с; исследователь достиг площадки в seed 1 через 59,7 с; тайм-аутов нет. Средняя скорость исследователя/стража **0,250/0,270 м/с**, доля поступательного движения 0,547/0,635, доля разворотов 0,389/0,343, общая активность 0,936/0,978, среднее число контактов 5,5/5,0, доступность планировщика 0,967/0,993. По сравнению с предыдущей серией 2/2 скорости улучшились (0,211/0,239), но баланс на этих четырёх seed сместился к стражу (1/3), а строгий порог скорости 0,3 м/с всё ещё не достигнут в среднем. Пилот seed 2 на том же образе дал поимку за 34,2 с, серия — за 16,2 с: точный ход матча зависит также от расписания автономных узлов; seed фиксирует выборы алгоритма, но не всю симуляционную гонку сообщений.
- Диагностика режимов: `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 0 --active-s 120 --build`, `results/series-20260927T161325Z/`, образ `78be21e+dirty.7fc5614fd033`. Страж поймал исследователя за 76,8 с; скорости 0,265/0,275 м/с, поступательные доли 0,682/0,659, развороты 0,295/0,320, контакты 2/8. Новая метрика в Gazebo работает: исследователь провёл **82,4%** времени в `GOAL`, **17,4%** в `EVADE`; из 29,5% времени разворота 29,4 пункта пришлись на `GOAL`, лишь 0,1 — на `EVADE`. У стража режимы `SEARCH`/`PURSUE`/`CAPTURE`: 49,2/43,7/7,0%; развороты по ним 18,9/12,6/0,5 пунктов. Вывод: лишнее время разворотов исследователя нельзя объяснить частым переключением на уклонение; сначала проверять локальную геометрию пути и ограничение скорости в повороте. Следующий шаг — контролируемое увеличение допустимой скорости на кривизне с сохранением проверки контактов.
- Гипотеза: малая боковая акселерация MPC (`a_lat_max=0.12`) и крейсерская `v_ref=0.55` ограничивали скорость в кривых участках, даже когда планировщик выдавал поступательный путь. Установлены `a_lat_max=0.18`, `v_ref=0.6`, предел MPC и маршевых режимов decision manager 0,7 м/с; режим поимки оставлен на 0,35 м/с для точной ориентации. Парный повтор seed 0–1 с пересборкой: `python3 benchmarks/run_duel_series.py --runs 2 --start-seed 0 --active-s 120 --build`, `results/series-20260927T162120Z/`, образ `78be21e+dirty.f4bc43c63573`. Исследователь **достиг площадки дважды** за 37,0 и 32,0 с; средняя скорость ролей 0,281/0,298 м/с, общая активность 0,936/0,984, контактов в среднем 2,5/3,0 за матч. В seed 0 скорость 0,291/0,338 и контакты 3/0; в seed 1 — 0,271/0,258 и контакты 2/6. По сравнению с предыдущей серией четырёх seed скорости выросли (0,250/0,270), но выборка мала и обе победы достались исследователю. Для проверки баланса запущена новая серия seed 0–3 на том же образе: `python3 benchmarks/run_duel_series.py --runs 4 --active-s 120`, `results/series-20260927T162740Z/`. 79 локальных тестов, `compileall`, `git diff --check` прошли.

- Диагностический повтор seed 2 с метриками разворотов и координатами касаний: `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 2 --active-s 120 --build`, образ `78be21e+dirty.aa33001d9577`, `results/series-20260927T154634Z/`. Итог — тайм-аут без достижения цели и без поимки. Средняя поступательная скорость исследователя/стража 0,215/0,163 м/с; доля поступательного движения 0,489/0,376, разворотов на месте 0,394/0,341, вся активность 0,883/0,716. Контактов со стенами 10/38, доступность планировщика 0,896/0,997. Страж многократно касался одной стены в точке около `(0,393; 3,356)` с 26-й до 61-й секунды; это явное застревание, которое высокий `planner_ok_fraction` не выявляет. `turning_fraction` измеряет все фактические развороты, пока без классификации их цели. По уточнению пользователя разворот при смене маршрута или переходе от достижения цели к убеганию может учитываться как осмысленная активность; отдельно сохраняется физическая средняя поступательная скорость, чтобы не скрывать остановки.
- Гипотеза следующего цикла: глобальный A* и локальный перебор выбирают траектории слишком близко к препятствию, несмотря на формально допустимый путь. Добавлена мягкая стоимость малой дистанции до сырых точек препятствий (без запрета узких проходов), локальный перебор вознаграждает увеличение зазора при выходе из раздутой занятой зоны. Тест выбора более удалённого от стены маршрута добавлен; **79 тестов**, `compileall` и `git diff --check` прошли. Повтор seed 2: `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 2 --active-s 120 --build`, `results/series-20260927T155658Z/`, образ `78be21e+dirty.f1fe9e9d4aa8`. Страж поймал исследователя через **34,2 с**; средняя скорость 0,244/0,280 м/с, поступательная доля 0,611/0,679, доля разворотов 0,282/0,303, суммарная активность 0,893/0,983, контактов 1/4. Для прежнего seed 2 было 10/38 контактов за 120 с; нормированная частота контактов снизилась с 4,9/19,0 до 1,7/7,0 в минуту, но один более короткий матч не доказывает устойчивого улучшения. Серия seed 0–3 на одном образе запущена для проверки повторяемости и баланса.

- Повторная серия после удержания маршрута: `python3 benchmarks/run_duel_series.py --runs 4 --active-s 120`, seed 0–3, `results/series-20260927T153514Z/`, тот же навигационный образ. Снова **2 цели исследователя** (64,7 и 117,9 с) и **2 поимки стража** (31,7 и 46,5 с), без тайм-аутов. Средняя скорость исследователя 0,211 м/с, стража 0,239 м/с; все 8 отчётов ниже 0,3 м/с. Доля поступательного движения 0,469/0,561, доступность планировщика 0,983/0,995, среднее число контактов 8,0/7,25. Удержание маршрута улучшило скорости относительно предыдущей серии 0,178/0,174 м/с, сохранив баланс в этих четырёх 120-секундных заездах, но столкновения остались частыми (seed 2: 18 у исследователя и 23 у стража, 47/44 повторных старта после остановки). Первые 20 с seed 0 в трассе: 2 смены начального направления глобального пути, 1 смена цели поворота, около половины команд поступательные. Новые `turning_fraction`/`active_motion_fraction` добавлены в исходники **после сборки этого образа**, поэтому нули в старом `summary.json` — техническая подстановка, а не измеренный факт; скрипт исправлен, чтобы при отсутствии поля выводить `null`.
- Следующий цикл: добавить координаты каждого контакта в отчёт, чтобы отделить повторные касания одного узкого места от разных ошибок маршрута; эта правка уже в исходниках, но не в образе. Уменьшить застревание и контакты, сохранив достигнутые задачи ролей, затем проверить новую метрику разворотов и влияние на скорость.

- Первая Gazebo-проверка удержания маршрута: `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 1 --active-s 120 --build`, образ `78be21e+dirty.d64d41828e0f`, `results/series-20260927T152857Z/`. Страж поймал исследователя через **20,7 с**; скорости 0,254/0,304 м/с, доля поступательного движения 0,574/0,635, доступность планировщика 0,942/0,990, столкновения 4/1. Для стража порог 0,3 достигнут впервые в полном заезде, исследователь ещё ниже. В прежних диагностических seed 1 без удержания пути исследователь достигал площадки через 71–115 с, поэтому изменение могло усилить стража; один повтор не доказывает смену баланса. Попытка 60-секундной трассы началась после короткого исхода и содержит только `WAIT`, не используется как доказательство уменьшения смен курса. Следующий шаг — серия seed с тем же образом, сравнение баланса, скорости и столкновений.

- Трасса направления глобального пути на seed 1 (`python3 benchmarks/run_duel_series.py --runs 1 --start-seed 1 --active-s 120`, `results/series-20260927T151822Z/`): исследователь достиг цели через 71,3 с при средней 0,148 м/с, страж 0,074 м/с, столкновения 7/5. В 75 с реального времени пассивный анализ зарегистрировал **27 смен начального направления глобального пути более чем на 1 рад при смещении робота менее 0,2 м** и 5 смен целевого угла локального поворота; 275/491 команд были путями поворота на месте. Часть нулевых команд в конце окна приходится на завершённую дуэль (`WAIT`). Смена коридора при обновлении карты — подтверждённая причина лишних разворотов в этом окне, хотя она не объясняет все остановки.
- Следующая гипотеза и изменение **ещё не проверены в Gazebo**: удерживать безопасный глобальный маршрут, сдвигая его начало по мере движения; перепланировать при смене поведения, заметном изменении цели, отклонении от пути, занятой клетке впереди или угрозе соперника. Удалён безусловный пересчёт A* раз в секунду. Это должно уменьшить смены стороны обхода без закрепления конкретного маршрута лабиринта. Добавлен модульный тест удержания/сброса пути; 77 тестов прошли. Следующий шаг — сборка, повтор seed 1, сравнение числа разворотов, скоростей и исхода.

- Повтор seed 1 с трассировкой угла: `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 1 --active-s 120`, `results/series-20260927T150751Z/`. Исследователь достиг площадки за 97,9 с; средние скорости 0,206/0,261 м/с, доли движения 0,487/0,699, столкновения 8/7. В 60-секундной трассе (реальное время) из 410 команд первого робота 274 были путями поворота на месте, 128 — поступательными, 8 — пустыми; из 274 поворотов 268 имели ошибку курса >0,5 рад, только 2 — <0,1 рад. Следовательно, основная остановка в этом окне — реальные развороты, а не ошибка шлюза или нулевая скорость MPC при выровненном курсе. Нужна проверка устойчивости направления глобального маршрута: частое изменение стороны обхода могло бы объяснить повторные развороты. Поздняя повторная трасса была запущена после исхода и показала только `WAIT`; в оценке движения она не используется.

- Диагностический seed 1 в назначении `explorer/guardian`: `python3 benchmarks/run_duel_series.py --runs 1 --start-seed 1 --active-s 120`, `results/series-20260927T150246Z/`. Исследователь коснулся площадки через 115,4 с; скорости 0,116/0,268 м/с, доля движения 0,287/0,684, доступность планировщика 0,997/0,998, столкновения 12/4. Пассивный `benchmarks/trace_motion.py` за 60 с реального времени объяснил остановки: у исследователя 388 из 405 команд (95,8%) имели локальный путь поворота на месте при статусе `OK`, только 16 команд были поступательными; у стража 181/405 (44,7%) были такими поворотами. MPC и защитный шлюз не являются главным источником нулевых команд в этом окне. Следующая проверка — распределение угла до точки на этих поворотах; если уже выровнен, менять оценку локальных кандидатов, а не ускорять MPC.

- Серия на неизменном образе после восстановления из защитной полосы: `python3 benchmarks/run_duel_series.py --runs 4 --active-s 120`, seed 0–3 с перестановкой ролей, ревизия скрипта `78be21e+dirty.376c8cdaf69e`, исходные данные `results/series-20260927T115629Z/`. **Два достижения площадки исследователем** (seed 1, 98,7 с; seed 3, 73,5 с) и **две поимки стражем** (seed 0, 105,0 с; seed 2, 57,5 с); нулевых исходов и технических сбоев нет. Средняя скорость по роли: исследователь 0,178 м/с, страж 0,174 м/с; все 8 индивидуальных отчётов ниже требования 0,3 м/с. Доля времени в движении 0,428/0,448, скорость во время движения около 0,416/0,390 м/с, доступность планировщика 0,928/0,996. Среднее число контактов 7,5/5,0; отдельные прогоны дали до 17 контактов. Баланс 2:2 пока относится только к четырём сокращённым матчам по 120 с, не к требуемым 20 матчам по 360 с. Следующая гипотеза: основной вклад в низкую среднюю скорость даёт ожидание/вращение при доступном пути и контакты со стенами; нужно измерить состояния шлюза, длину локального пути и команды MPC во время заезда и уменьшить остановки и контакты без потери обеих задач.

- Проверка восстановления из раздутой зоны: `python3 benchmarks/run_duel_series.py --runs 1 --active-s 120 --build`, seed 0, образ `78be21e+dirty.c14b31fa5977`. **Первый исход — поимка стражем через 19,5 с**, без столкновений и без телепортов. Исследователь прошёл 5,516 м за 20,8 с своего наблюдателя, средняя 0,265 м/с, скорость в движении 0,412 м/с, движение 64,4% времени, доступность планировщика 99%; страж 3,916 м за 19,6 с, 0,200 м/с, скорость в движении 0,377 м/с, движение 53,1%, доступность планировщика 98,5%. RTF 0,642. Это подтверждает фактическую поимку и устранение прежнего тупика на seed 0, но **оба робота ещё ниже требуемых 0,3 м/с**, цели исследователь не достиг. Средние длительности наблюдателей отличаются от времени первого исхода из-за задержки доставки команды остановки (у исследователя около 1,3 с); скорость считается по фактическому активному окну каждого наблюдателя. Следующий шаг — серия других seed без изменения образа и настройка времени в движении/скорости без роста столкновений.

- Заезд после завершения короткого кандидата у поворота: `python3 benchmarks/run_duel_series.py --runs 1 --active-s 120 --build`, seed 0, образ `78be21e+dirty.5a72ab3e0a64`: тайм-аут без цели/поимки, средние скорости 0,028/0,045 м/с, `planner_ok_fraction=0,164/0,225`. Начало было лучше (у исследователя 0,231 м/с и 52% времени в движении на 6,4-й секунде), но к 25-й секунде он снова застрял; к 98-й секунде два контакта и `planner_ok_fraction=0,183`. Пассивный снимок: поза `(0,539;0,979)`, стартовая ячейка `(4,7)` занята, до свободной полосы вниз две клетки, A* вернул пустой путь. Значит исправление поворота работает на коротком участке, но требуется восстановление маршрута из нескольких консервативно занятых клеток.
- **Следующая ещё не проверенная в Gazebo доработка:** карта хранит точки препятствий до расширения. Вблизи занятого старта A* может пройти не более трёх клеток через защитную полосу 0,19–0,23 м с дополнительной стоимостью, локальный перебор допускает выход на расстоянии до 0,5 м от старта только при достаточном физическом зазоре либо при его монотонном увеличении. Радиус корпуса модели 0,178 м; 0,19 м оставляет небольшой запас. Пассивный снимок у стены показал зазор 0,134 м у уже столкнувшегося робота и 0,284 м при движении вниз на 0,15 м. Синтетическая проверка такого выхода и остальные тесты прошли (76 тестов). Нужна сборка и реальная дуэль; столкновения и средняя скорость остаются критичными.

- Проверка только первого исправления просмотра поворота: `python3 benchmarks/run_duel_series.py --runs 1 --active-s 120 --build`, seed 0, образ `78be21e+dirty.19c032175b86`: тайм-аут без цели/поимки, средние скорости 0,013/0,026 м/с, доступность планировщика 0,999/0,998. Улучшения нет. Пассивный снимок показал глобальный маршрут из 44 клеток и локальный результат из двух точек с одинаковым положением: дальняя точка уже исправлена, но оценка кандидата только в конце горизонта предпочитала стоять при близкой промежуточной точке. Добавлено завершение кандидата при приближении к такой точке на 0,06 м. На **тех же живых ROS-данных** старая версия дала 2 точки и нулевой сдвиг, новая версия, загруженная в отдельный процесс диагностики, дала 5 точек и около 0,15 м безопасного продвижения. Рабочий узел не менялся во время заезда; новая версия ещё требует пересборки и проверки дуэлью. 75 тестов прошли.

- Дополнительная проверка того же собранного образа: `python3 benchmarks/run_duel_series.py --runs 1 --active-s 120`, seed 0; тайм-аут без цели/поимки, исследователь 0,014 м/с, страж 0,039 м/с. У исследователя планировщик был `OK` 99,8% времени, значит пустой **глобальный** путь уже не объясняет остановку. Создан `benchmarks/planner_probe.py` для пассивного снимка карты, маршрута и локального перебора. Во время заезда поза исследователя `(0,963;0,847)`, занятая стартовая ячейка `(6,6)`, глобальный маршрут из 45 клеток; первые точки `(0,9;0,9) → (0,9;0,75) → (1,05;0,75)`, то есть сначала вниз, затем вправо. Локальный план давал только поворот на месте: точка дальнего просмотра перепрыгивала первый поворот и указывала вправо через занятую область. Правило просмотра изменено: при расхождении направления первого шага и дальней точки больше 0,7 рад используется первый поворот. Для этого случая добавлен тест; всего 75 тестов прошли, `compileall` и `git diff --check` успешны. **Сборка и Gazebo-проверка этого исправления ещё впереди.**

- Повтор после исправления выхода из зоны и допускa поимки, а также повышения заданной скорости: `python3 benchmarks/run_duel_series.py --runs 2 --active-s 120 --build`, образ `78be21e+dirty.586148909f3f`. Seed 0 закончился тайм-аутом без цели/поимки: исследователь 1,853 м, средняя 0,015 м/с, скорость во время движения 0,412 м/с, движение 3,7% времени, `planner_ok_fraction=0,074`, 2 столкновения; страж 2,617 м, 0,022 м/с, во время движения 0,444 м/с, движение 4,9%, `planner_ok_fraction=0,999`, 1 столкновение. Это **ухудшение** относительно базовых 0,039/0,042 м/с на том же seed; высокая скорость во время движения не помогает при почти постоянной остановке. Seed 1 не стартовал: `gzserver` упал с кодом −11 после создания второго робота, поэтому исхода нет. Добавлена повторная инициализация мира до трёх раз при отсутствии поз; её проверка ещё идёт. Вне защитной зоны (дистанция до соперника >2 м) наблюдался `NO_GLOBAL_PATH`, так что прежняя гипотеза о единственной причине отказов отвергнута. Следующий шаг — снять состояние занятости сетки при отказе и исправить построение достижимого маршрута; ускорение MPC пока не подтверждено как полезное.

- Гипотеза: локальные исправления стартовой занятой ячейки недостаточны для устойчивой дуэли. Добавлен `hsl-referee`: по точным позам Gazebo он фиксирует **первое** событие поимки, касания площадки или истечения 120/360 с активного симуляционного времени, сохраняет `results/latest_outcome.json` и закрывает движение обоих роботов. Исход `timeout` засчитывается исследователю по регламенту, но `explorer_reached_goal=false` сохраняется отдельно. Дымовая проверка с `DUEL_MAX_ACTIVE_S=30.0`: остановка ровно на 30,0 с; отчёты обоих роботов помечены `active=false` и одним `run_id`. Скорости 0,134/0,152 м/с, ниже нового требования.
- Добавлен `benchmarks/run_duel_series.py`: на каждом seed пересоздаёт мир, ждёт обе позы, меняет назначение ролей местами, запускает матч и сводит исход с обоими отчётами. Базовая команда: `python3 benchmarks/run_duel_series.py --runs 4 --active-s 120 --build`, образ соответствовал `78be21e+dirty.1e025b9838cf`. Семена 0, 1, 2 завершились тайм-аутом **без поимки и без цели**. Средняя скорость исследователя по трём матчам 0,032 м/с, стража 0,042 м/с; доступность планировщика 0,195 и 0,998; средняя доля времени в движении около 0,16 и 0,224. У исследователя были 0/1/2 столкновения, у стража 0/0/0. Исходные данные — `results/series-20260927T105324Z/index.json` и `summary.json` (локальные, игнорируются Git). Четвёртый seed **не оценён**: CLI ожидания топика завис и завершился по тайм-ауту до старта. Повторное ожидание после такого сбоя добавлено в скрипт, ещё не проверено серией.
- Наблюдение: у исследователя при близком стражe повторяется `NO_GLOBAL_PATH`; защитная зона вокруг противника накрывает самого исследователя и прежние A* и локальный перебор не разрешали выйти наружу. У стража при seed 1 позы `(0,023;3,149)` и `(0,461;3,338)` давали расстояние 0,477 м: он получал `OK`, но команда была нулевой, поскольку точка поимки считалась достигнутой при допуске 0,08 м, раньше порога судьи 0,45 м.
- Текущая гипотеза и изменение **ещё не проверены в Gazebo**: разрешён монотонный выход из зоны соперника в A* и локальном переборе; точка поимки стража приближена к 0,39 м, допуск уменьшен до 0,015 м. В запуске MPC заданы `v_ref=0.55`, `v_cmd_max=0.65`, более мягкий предел бокового ускорения `a_lat_max=0.12`; ограничения скоростей decision manager подняты до 0,55–0,65 м/с на маршевых участках. Порог брака средней скорости в скрипте изменён с 0,05 на **0,3 м/с** по уточнению пользователя; речь о среднем пути за всё активное время до первого исхода, а не о скорости только в движении. Локальные тесты до изменения скоростей: 74 прошли. Следующий шаг — сборка, повторная серия и разбор остаточных пустых путей, скорости и исходов.

## Итерация: два автономных робота и выход из тупика планировщика (27.09)

- Текущий `duel` запускает два отдельных контура `адаптер → decision manager → планировщик → MPC → шлюз` и два наблюдателя метрик. Первый контур использует `/navigation/*` и `/cmd_vel`, второй — `/opponent/navigation/*` и `/opponent/cmd_vel`; у каждого топика команды проверен ровно один издатель (`hsl_mpc_gate`). Сценарный патруль исключён из профиля `duel`. `helm start_match`/`stop_match` переключают оба контура, отчёты записываются в `results/` и `results/opponent/`. Это проверено запущенными ROS-узлами и короткими заездами, но пока не подтверждает выполнение задач ролей.
- Базовый заезд после первого запуска: исследователь за 74,7 с симуляционного времени прошёл 1,613 м (средняя 0,022 м/с, планировщик `OK` 29,7% времени); страж за 74,9 с — 2,33 м (0,031 м/с, `OK` 99,6%). Цели и поимки не было, RTF около 0,51. Второй заезд был запущен **без сброса мира** из конечных поз первого: за 73,1 с каждый прошёл около 0,1 м, средняя 0,001 м/с. Он диагностический и не является независимым повтором. Движение обоих после `stop_match` запрещено.
- Во втором заезде исследователь постоянно выдавал `NO_LOCAL_PATH`, пустой путь и нулевой `/cmd_vel`; страж выдавал `OK`, но чередовал угловые команды `+1/-1` при нулевой линейной. Отдельная диагностика показала, что реальная поза исследователя `(0,665; 0,833)` попала в занятую ячейку `(4, 6)` грубой сетки планировщика; один сосед свободен, хотя физического столкновения не было. Это объясняет отказ всех локальных траекторий, начинающихся в этой ячейке.
- Исправлен локальный перебор: он допускает выход из занятой стартовой ячейки, сохраняет безопасный короткий префикс вместо отбрасывания всего манёвра, берёт более близкую точку глобального пути и выдаёт стабильную целевую ориентацию для поворота на месте. `python3 -m pytest -q tests/test_navigation.py helm_launch/tests/tests.py` — 70 тестов прошли, `git diff --check` — без ошибок; образ `duel` пересобран.
- Свежий заезд после пересоздания мира (те же стартовые позы): у стража `capture_at_s=42,0` по геометрии Gazebo без телепорта. До ручной остановки на 145-й секунде исследователь прошёл 7,042 м (0,049 м/с, `planner_ok_fraction=0,279`), страж 8,678 м (0,060 м/с, `planner_ok_fraction=0,289`), RTF 0,53, столкновений нет. Значительная часть низкой средней скорости приходится на время **после** зарегистрированной поимки: дуэль ещё не завершалась автоматически, поэтому эти средние не являются оценкой до исхода. Этот результат подтверждает одну фактическую поимку, но не повторяемость и не успех исследователя.
- Следующий шаг: единый судейский узел должен фиксировать первое событие и останавливать обоих роботов, затем добавить воспроизводимый seed и автоматическую серию полных заездов по 6 минут активного времени. До серии из 20 независимых заездов и фактических достижений цели исследователем критерий задачи не подтверждён.

## Ветка и интеграция

- Ветка `feature/decision-manager` находится поверх `origin/main` на коммите `48760c3`; первоначальный контур decision manager был локально закоммичен до объединения. Интеграция с новым `main` оформлена в этой же локальной ветке.
- Из `main` получены Gazebo Classic 11, мир `maze.world`, аналитическая карта занятости `/map`, точная симуляционная поза `/localization/pose`, демонстрационные планировщик и MPC-контроллер из подмодуля `mpc_motion_control`, команды `helm start/stop` для штатного профиля `simulation`. Экспериментально установлено, что `/odom` в Classic уже задан в мировых координатах; штатная карта теперь тоже использует их, а `duel` переводит мир в `map` относительно старта.
- Ничего не отправлялось в `origin` по прямому указанию пользователя.

## Действующие профили и модули

- `gazebo`: один Kobuki в Gazebo Classic.
- `simulation`: одиночный Kobuki, карта из SDF, заранее заданная траектория и MPC из `main`. Этот профиль сохранён.
- `duel`: отдельный сценарий с двумя Kobuki, раздельными темами и TF. Оба управляются собственными автономными стеками; прежний сценарный патруль вынесен в профиль `scripted_duel`. `hsl-map` публикует карту относительно старта первого робота. `hsl_sim_adapter` переводит точную симуляционную позу в `map`, публикует скан и точки статической карты, передаёт симуляционную `/map` как `/navigation/known_grid`. Поза соперника выдаётся в `/navigation/opponent` только при подтверждении текущим LiDAR и отсутствии стены на прямой видимости; `/navigation/opponent_visible` показывает результат. MPC следует локальному пути через защитный шлюз; у `/cmd_vel` и `/opponent/cmd_vel` по одному издателю.
- Разрешение движения по умолчанию закрыто и включается сервисом `/match/allow_motion` после `freeze time`. Шлюз останавливает робота при `WAIT`/`STOP`, пустом пути и устаревшей позе, скане, задании, пути или команде MPC. Простой `debug_follower` оставлен как явно запускаемая запасная реализация контроллера.
- `helm start_match` и `helm stop_match` вызывают сервис разрешения движения без ручного экранирования аргументов. Состояние также публикуется на `/match/allowed` (transient local). Узел `hsl-metrics` измеряет фактический путь, среднюю скорость за весь матч и во время движения, использование ориентира 0,7 м/с, RMS линейного и углового ускорения, повторные старты после остановок, контакты корпуса, время видимости и доступности планировщика, моменты поимки/достижения цели. Текущий снимок — `/match/metrics` (JSON в `String`), итоги каждого заезда — `results/duel-*.json` и `results/latest.json`. Это исследовательские показатели, не официальные баллы. Отладочные телепорты исключаются из пройденного пути.
- Для оценки быстродействия тот же отчёт содержит отношение симуляционного времени к реальному (`real_time_factor`), длительности вычисления итераций decision/planning/control (`/navigation/*_cycle_ms`), реальные интервалы между публикациями решения, статуса планировщика, скана и команды, а также возраст скана на момент команды. По каждому ряду сохраняются p95, максимум и число событий от двух секунд; для вычислений также считается число превышений бюджета 200/200/50 мс. В `duel` оба CPU LiDAR теперь имеют 360 × 16 лучей при 10 Гц вместо 900 × 40; значения настраиваются через `.env`, одиночный профиль использует прежнее разрешение.
- Страж движется к последней видимой позиции (либо к старту соперника), затем выбирает доступные точки карты для поиска. При наблюдении соперника он строит путь к позиции поимки, не использует отталкивание от самой цели и останавливается после выполнения геометрических условий поимки в пределах видимости. В оценочном профиле оба робота автономны; исход определяет отдельный судейский узел.
- По плану команды статическая карта реального лабиринта будет снята заранее; картографирование во время заезда не планируется. Источник этой карты, локализация и обнаружение соперника на реальном роботе ещё не подключены. Карта из SDF известна только в симуляции.

## Проверки

- После добавления метрик быстродействия выполнены `python3 -m pytest -q tests/test_navigation.py helm_launch/tests/tests.py` (67 тестов), `compileall` и `git diff --check`. Образ `duel` пересобран, Xacro проверен с параметрами 360 × 16, контейнеры пересозданы. При двух роботах и GUI до уменьшения плотности LiDAR `gz stats` показывал real time factor около 0,15; после изменения — 0,56–0,57 в короткой выборке. В заезде 15,8 с симуляционного / 30,4 с реального времени отчёт показал RTF 0,52, p95/максимум вычисления планировщика 77,39/114,82 мс (0 превышений 200 мс), decision 0,70/0,82 мс, control 0,37/2,03 мс. Максимальные реальные интервалы между публикациями решения/планировщика были 485/487 мс. За этот короткий прогон двухсекундных итераций не обнаружено. Временная проба шага физики 0,002 с и 80 итераций не ускорила Gazebo, исходные настройки физики возвращены. Последняя доработка счётчика `over_2s` и учёта пауз свыше 10 с проверена локальными тестами, но после неё образ не пересобирался. После проверки контейнеры `duel` остановлены и удалены; для запуска актуального кода нужны `helm build duel` и `helm up duel`.
- Для обновления матча выполнены `python3 -m pytest -q tests/test_navigation.py helm_launch/tests/tests.py` (65 тестов), `compileall` и `git diff --check`; образ `duel` пересобран. В первом коротком прогоне `helm start_match`/`stop_match` с ролью `explorer` вернули успешные ответы сервиса, узел сохранил JSON: 4,4 с симуляционного времени, путь 1,134 м, средняя скорость 0,258 м/с, столкновения 0. В прогоне `guardian` до сближения было состояние `searching opponent start area`, планировщик выдавал `OK` и ненулевой `/cmd_vel`; после отладочного переноса в открытый коридор `/navigation/opponent_visible=true`, решение перешло в `opponent captured`, наблюдатель зарегистрировал `capture_at_s=11.4`. Дистанция этого теста была искажена переносом; после теста добавлена фильтрация скачков. При проверке столкновения обнаружено, что Gazebo переименовывает коллизию корпуса после объединения фиксированных суставов: имя в датчике исправлено, образ повторно собран. Затем два отдельных прогона подтвердили `robot_collisions=1` при физическом контакте двух Kobuki и `wall_collisions=1` при управляемом наезде на статическую стену. Контакты с полом исключены.
- Для нового сценария выполнены `python3 -m pytest -q tests/test_navigation.py helm_launch/tests/tests.py` (60 тестов прошли), `compileall`, `git diff --check` и `docker compose --profile duel config --services`. Геометрия `maze.world` проверена по той же функции растеризации, что использует `jr_map`: `(2.5, 2.5)` свободна и доступна из старта при радиусе 0,25 м, участок патруля до `(1.0, 2.5)` свободен, прямая из старта перекрыта стеной.
- Образ `jr_image:latest` пересобран, `duel` пересоздан и проверен в Gazebo Classic без GUI. Соперник появился в `(2.5, 2.5)`. Из старта `/navigation/opponent_visible=false`; после временного переноса своего робота в открытый коридор `(1.5, 2.5)` флаг стал `true`, `/navigation/opponent` выдал позицию около `(2.84, 2.68)` в кадре `map`; после возврата на старт флаг снова стал `false`. После разрешения движения соперник прошёл от `x=2.5` до `x≈2.03` вдоль коридора с командой `0.23 м/с`; после запрета и его `/opponent/cmd_vel`, и свой `/cmd_vel` стали нулевыми. Проверка полного прохода до разворота и прохода на реальном роботе остаётся открытой.
- Упрощён README: оставлены команды запуска `duel`, выбор роли и разрешение движения; подробный контракт по-прежнему описан в AGENTS.md. Проверены diff и ссылки; ROS/Gazebo для этой правки документации не запускались.
- До обновления `main` предыдущий контур был проверен в Gazebo Harmonic. Эти результаты относятся к старой среде; ниже приведены новые проверки Gazebo Classic.
- После ребейза: `python3 -m pytest -q tests/test_navigation.py helm_launch/tests/tests.py` — 56 тестов прошли; `compileall`, `git diff --check` и `docker compose --profile duel config --services` — успешно. Образ с пакетами ROS и подмодулем MPC собран локально.
- В `duel` проверены `/navigation/self`, `/navigation/opponent`, `/navigation/scan`, `/navigation/map_points`, `/navigation/known_grid`, `/navigation/intent` и оба `Path`. TF `map → base_footprint` и `map → opponent/base_footprint` совпали с ground truth. У Gazebo ray-сенсора обнаружены около 900 собственных отражений внутри 0,2 м; симуляционный адаптер удаляет точки корпуса в радиусе 0,25 м, сохраняя более дальние препятствия.
- До разрешения старта — `WAIT` и нулевой `/cmd_vel`. После разрешения исследователь публиковал оба непустых пути и двигался: в повторной восьмисекундной выборке 6/6 путей обоих уровней были непустыми, 13/13 команд — ненулевыми, положение изменилось с `(1.032, −0.031)` до `(1.256, −0.131)` при движущемся сопернике. При близком сопернике планировщик сначала выдавал `NO_GLOBAL_PATH`, затем перестроил безопасный путь.
- Для стража подтверждены `role=guardian`, начальные `WAIT` и нулевая команда; после разрешения движения — поведение преследования, 7/7 непустых глобальных и локальных путей, 13/13 ненулевых команд, перемещение с `(2.611, −0.874)` до `(2.848, −0.791)` за 8 секунд при движущемся сопернике. После запрета движения снова `WAIT` и нулевой `/cmd_vel`.
- В профиле `simulation` из `main` отдельно создан одиночный Kobuki. `/odom` и `/localization/pose` совпали около `(-0.34, -0.18)` в мировых координатах, `/livox/lidar` публиковал облако, карта из SDF использовала мировое начало, демонстрационный планировщик выдавал `/planning/trajectory` в кадре `odom`, MPC поднялся в режиме ожидания. После smoke-проверки `simulation` остановлен, `duel` возвращён к роли `explorer` с закрытым движением.

## Последние итерации локального планирования и скорости (29.09)

**Итерация 1 — скорость по кривизне и запасу до препятствий.** Планировщик теперь вычисляет верхнюю скорость по фактически выбранному локальному пути: прямизна ограничивается оценкой кривизны и боковым ускорением, а доступный за вычетом радиуса корпуса/запаса clearance задаёт дополнительный cap. Значение публикуется в `navigation/speed_limit`; MPC использует его как динамическую целевую скорость, шлюз применяет как финальный предел. Оба контура допускают разгон до 1,0 м/с, но только в пределах cap и поведенческой скорости; изменение задания сглажено ускорением 0,45 м/с² и торможением 0,45 м/с². При устаревшем cap шлюз останавливается. Диагностика трассы теперь сохраняет среднее, минимум, p90 cap и долю времени выше 0,3 м/с.

Проверки перед Gazebo: **121 тест passed**, `compileall`, `git diff --check`, `helm build duel`. Серия сценария 1, seeds 12–14, лимит 90 с: `results/series-20260929T133657Z/`. Все три исхода — поимка, цели нет; контактов нет. Средняя скорость за весь матч: исследователь **0,149 м/с**, страж **0,205 м/с**; ниже 0,2 остались 3/3 исследователей и 2/3 стражей. Средний cap в трассах был 0,265–0,388 м/с, но `gate_clipped` составлял 9–52%, а локальные кривые занимали 0–10% обновлений в отдельных трассах. Значит расчёт cap сам по себе не снимает основной простой: контроллер часто ждёт разворота из-за геометрии пути.

**Итерация 2 — больше безопасных дуг.** По логам первой серии частые причины отказа дуги: `route_heading_large`, `short_distance`, `route_handle_curve_clearance` и `route_handle_curvature`. Предел разницы курса расширен с 1,15 до 1,45 рад, минимальная длина кривой снижена с 0,65 до 0,45 м, проверяются более длинные кубические кривые/ручки до 1,2 м, дистанция поиска маршрута увеличена до 2,5 м. Swept-clearance и предел кривизны 2,6 1/м сохранены; непрошедшие путь проверки кривые отклоняются. Добавлен тест 1,3 рад; **122 теста прошли**, `helm build duel` успешен. Повтор scenario 1, seeds 12–14, 90 с: `results/series-20260929T134723Z/`. Результат — 2 поимки / 1 достижение цели, **0 контактов**; средние скорости исследователя/стража 0,157/0,189 м/с, ниже 0,2 во всех матчах исследователя и в одном из трёх стража. Доля кривых выше, чем в итерации 1, но `gate_clipped` остаётся до 41%, повороты на месте — 28,1%/16,3%. Кривые часто отвергались по clearance и кривизне, поэтому цель скорости не достигнута.

**Итерация 3 — согласовать допустимую кривизну с динамическим замедлением.** Геометрический предел дуги временно поднят до 3,2 1/м, запас уменьшен до `robot_radius + 0,12 м`, а порог курса оставлен 1,55 рад. После **122 локальных тестов** и сборки проведён повтор scenario 1, seeds 12–14, 90 с: `results/series-20260929T135921Z/`. Итог — 2 поимки / 1 цель, **0 контактов**; скорость 0,153/0,174 м/с, исследователь поворачивал на месте 35,1% матча, страж — 17,1%. Проба ухудшила скорости относительно итерации 2, поэтому геометрический предел/запас возвращены к 2,6 1/м и `robot_radius + 0,18 м`, курс — до 1,45 рад.

**Итерация 4 — двигаться при умеренной ошибке курса.** Прямой путь теперь допускает поступательное движение до 1,0 рад ошибки (ранее 0,65), с гистерезисом 0,75 рад; curved path использует тот же порог. В динамический speed cap добавлен множитель `cos(error)` с нижней границей 0,15. Добавлены отдельные clearance-, curvature- и alignment-caps в трассу. **123 теста**, `compileall`, `helm build duel` прошли. Серия scenario 1, seeds 12–14, лимит 90 с: `results/series-20260929T141158Z/`. Итог — 1 поимка / 2 тайм-аута, 0 контактов; средние скорости 0,158/0,181 м/с, повороты 31,4%/20,4%. Скорости cap-компонентов в трассе показали основной лимит clearance (среднее 0,256–0,341 м/с); медианная ошибка курса при обнулении шлюзом всё ещё 1,4–1,5 рад, то есть умеренный порог 1,0 почти не меняет частые большие доворачивания. Цель скорости и успешность не улучшены.

**Итерация 5 — выбирать более свободный путь, чтобы повысить допустимую скорость.** A* временно получил мягкую цену за недостаток места до предпочтительного clearance `robot_radius + 0,22 м`; модульный тест подтвердил сдвиг пути от стены 0,35 м к 0,40 м. Сборка и **124 теста** прошли. Gazebo серия scenario 1, seeds 12–14, 90 с: `results/series-20260929T142756Z/`. Результат — **1 достижение / 2 поимки**, контактов 0; средняя скорость исследователя/стража 0,151/0,215 м/с против 0,158/0,181 м/с предыдущего прогона. Исследователь остался ниже 0,2 во всех трёх матчах; страж — в одном из трёх. Средний `speed_limit` исследователя по трассам составлял 0,212–0,264 м/с, но доля движения около 0,7 и `turning_fraction` в серии — 0,253/0,042. Наблюдаемый выигрыш в балансе неустойчив, цель скорости не достигнута, поэтому штраф A* **откачен**.

**Итерация 6 — длиннее дуга только при достаточном свободном месте.** Трассы показывали: предел MPC по кривизне давал cap 0,46–0,99 м/с, clearance — 0,25–0,30 м/с; локальная дуга появлялась в 0–24% обновлений, а шлюз обнулял до 55% команд при ошибках курса 0,97–1,57 рад. После отката итерации 5 допустимый угол дуги стал расти с фактическим запасом до стены/границы; при достаточном месте планировщик ищет маршрут дальше и перебирает более длинные ручки кубической кривой. Открытая карта приняла дугу при ошибке курса 1,65 рад, тесная карта сохранила меньший предел. **125 тестов** и полная сборка прошли. Серия scenario 1, seeds 12–14, 90 с: `results/series-20260929T144101Z/`; 1 достижение / 2 поимки, контактов 0, средняя скорость 0,159/0,187 м/с. Исследователь ниже 0,2 во всех трёх матчах, страж — в двух. Частота длинных дуг по трассам осталась низкой и переменной; существенного выигрыша нет.

**Итерация 7 — локальный кинематический rollout вместо ожидания выравнивания.** Подключены безопасные примитивы постоянной поступательной/угловой скорости только там, где исходный путь требует ошибки курса >0,55 рад и rollout даёт измеримое продвижение. Учтены карта, динамические препятствия, соперник, прежняя угловая скорость и tracking clearance `robot_radius + 0,18 м`. **126 тестов** и сборка прошли. Серия scenario 1, seeds 12–14, 90 с: `results/series-20260929T145431Z/`. Баланс — **2 цели / 1 поимка**, контактов 0; средние скорости исследователя/стража **0,169/0,198 м/с** против 0,159/0,187 в итерации 6. Доля времени с поступательным движением выросла до 0,759/0,887 (была 0,686/0,848), доля дуговых траекторий по трассам — в среднем 0,183 (была 0,103), обрезанные шлюзом команды снизились примерно с 0,21 до 0,17. Улучшение повторилось по moving fraction, но целевые 0,2 м/с ещё не достигнуты: все матчи исследователя ниже, один из трёх стражей выше. Rollout оставлен как улучшение.

**Итерация 8 — стоимость A* с учётом времени движения (эксперимент, позднее откачен).** Для избежания маршрутов, которые проходят почти у минимального clearance, цена ячейки дополнительно оценивала предполагаемую потерю скорости; collision/inflation ограничения не ослаблялись. Проверка: `helm build duel` прошла; **127 тестов** прошли. Headless серия на сценарии 1, seeds 12–14, 90 с, `results/series-20260929T151133Z/`, выполнялась с исходным кодом обоих MPC-узлов и speed cap только на выходном шлюзе. Получены 3 достижения цели исследователем, 0 поимок стражем; средняя скорость исследователя/стража **0,177/0,200 м/с**; контакты в среднем **0,333/0** на матч. Цели по минимальным 0,2 м/с исследователь не достиг. Доля поступательного движения — 0,762/0,871; медианная ошибка курса при ограниченной команде шлюзом — 1,19–1,50 рад. Средний speed cap по трассам — 0,195–0,265 м/с, в частности исследователь в одном заезде ограничивался clearance cap 0,218 м/с. Эксперимент не устранил простой; пользователь подтвердил, что A* следует оставить без этой добавочной оценки скорости.

**Обзор исходников Nav2 Humble.** [RPP README](https://github.com/ros-navigation/navigation2/blob/humble/nav2_regulated_pure_pursuit_controller/README.md) и [реализация контроллера](https://github.com/ros-navigation/navigation2/blob/humble/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp): Nav2 обрезает глобальный маршрут с ближайшей к роботу точки, масштабирует lookahead по текущей скорости и ограничивает его min/max, интерполирует пересечение пути с окружностью точного радиуса; затем считает `curvature = 2*y/(x²+y²)` и `angular = linear*curvature`. Линейная скорость ограничивается кривизной, близостью препятствий и расстоянием до цели. Отдельно Nav2 прогнозирует командную дугу с шагом costmap до заданного времени/дистанции и проверяет footprint. Большую ошибку курса RPP всё ещё может отрабатывать поворотом на месте. [Nav2 MPPI](https://github.com/ros-navigation/navigation2/blob/humble/nav2_mppi_controller/README.md) рассматривает много sampled motion trajectories и оценивает их obstacle/footprint и path-following critics; это более масштабная замена контроллера и не требуется для текущей проверки локальной геометрии.

**Итерация 9 — локальная дуга по Regulated Pure Pursuit.** Взяты геометрические части RPP: ближайшая проекция маршрута, lookahead `clamp(|v|*1,5 с, 0,5 м, 1,3 м)`, точное пересечение сегмента с окружностью и дуга с кривизной `2*y/(x²+y²)`. Планировщик публикует эту дугу как локальную Path-ссылку для существующего Lat-MPC; он не заменён генератором команд RPP. Дуга проверяется по LiDAR/карте, корпусу, границе арены и clearance соперника с дополнительным запасом на tracking; слишком тесная дуга отклоняется, затем пробуются меньшие lookahead и прежние проверенные кривые/rollout. Дуга пересчитывается по текущим позе, скорости и скану на каждом цикле планировщика. Исходники обоих MPC в подмодуле `src/mpc_motion_control` не менялись. Добавлены тесты точного пересечения, зависимости lookahead от скорости, касательной гладкости и отказа при пересечении нового препятствия. **131 тест прошёл**, `compileall` и `git diff --check` прошли, `helm build duel` прошёл. Контрольная серия на диапазоне 1,0 м/с (seed 12–13 из трёх), `results/series-20260929T152520Z/`: два исследователя достигли цели, контактов нет; средняя скорость исследователь/страж по двум матчам **0,202/0,198 м/с**. Доля поступательного движения 0,864/0,888, дуговые пути занимали 0,148–0,305 трассы; медианная ошибка курса 0,98–1,35 рад против 1,19–1,55 рад в контрольной серии. Третий seed пользователь остановил вместе с контейнерами для проверки; полный итог серии не рассчитан. Исходный диапазон CC-MPC `v_ref=0,3`, `v_cmd_max=0,3` восстановлен в исходнике запуска; более ранняя серия на RPP с этим диапазоном не проводилась.

**Текущее решение по ограничению скорости.** По уточнению пользователя удалены все добавленные в planner/gate пределы `navigation/speed_limit` по кривизне, clearance и углу курса, а также `speed_limit_max/min` и сглаживание изменения cap. Из DecisionPolicy убрано снижение max_speed при приближении стража к точке захвата; активное поведение запрашивает 1,0 м/с, а фактический диапазон задаёт штатный контроллер (`v_ref=0,3`, `v_cmd_max=0,3`, включая его исходное замедление по `v_curve`). Выходной шлюз возвращён к прежней команде ограничения около конечной точки пути. Оценка скорости в A* удалена, базовая стоимость глобального маршрута восстановлена. RPP остаётся геометрическим построителем проверенной локальной дуги для исходного MPC. Срез до этого изменения сохранён в коммите `a5ad815` и теге `checkpoint/navigation-rpp-20260929`. Прогон, начатый до этого уточнения, отменён до появления отчёта и не считается результатом. После удаления cap: **127 тестов прошли**, `compileall`, `git diff --check` и `helm build duel` прошли.

**Headless-проверка восстановленного диапазона контроллера.** `results/series-20260929T155855Z/`, scenario 1, seeds 12–14, 90 с: 2 достижения цели / 1 поимка, контактов 0. Средняя скорость исследователя/стража — **0,176/0,218 м/с**; исследователь ниже 0,2 м/с в двух матчах из трёх, страж — ни в одном. Средний `planner_ok_fraction` 0,967/0,974, `moving_fraction` 0,728/0,859, повороты на месте 0,251/0,125. Трассы показывают медианную ошибку курса 1,22–1,37 рад и сильно переменную долю кривых путей 0–0,269; у исследователя seed 14 кривых не было, а шлюз ограничил поступательную команду в 37,7% отсчётов. Значит, после удаления сторонних скоростных caps стабильность без контактов сохранилась, но исследователь всё ещё медленно проходит матчи из-за частых разворотов/неплавной локальной геометрии. Контейнеры после серии остановлены и удалены `helm clean duel`.

**Проба повторного использования безопасной дуги RPP (не оставлена).** Вариант держал ранее построенную дугу, пока цель не менялась и скан/карта подтверждали её безопасность. Headless серия `results/series-20260929T161055Z/`, те же scenario/seeds: 2 цели / 1 поимка, 0 контактов; средние скорости 0,191/0,204 м/с против 0,176/0,218 до пробы, но среднее по двум ролям осталось **0,197 м/с**. Доля смен локальной геометрии уменьшилась 0,625→0,568, однако доля поворотов на месте выросла 0,188→0,201, angular-accel RMS 1,024→1,076 рад/с², а медианная ошибка курса осталась около 1,3 рад; линейное ускорение стало мягче, но общей победы нет. Изменение откачено, текущий вариант снова пересчитывает RPP на каждом цикле. До прогона **128 тестов** и `helm build duel` прошли. Duel-контейнеры остановлены и удалены.

**RPP minimum lookahead 0,5→0,8 м (текущий кандидат).** При штатном диапазоне контроллера 0,3 м/с геометрический lookahead часто упирался в 0,5 м и отклонял безопасную дугу на повороте 1,2 рад из-за предела кривизны. Минимум поднят до 0,8 м; каждый участок более длинной дуги по-прежнему проходит проверку clearance, столкновений нет. Регрессионный тест показывает, что такой поворот теперь укладывается в предел кривизны и даёт касательную к курсу робота дугу. Headless серия `results/series-20260929T162119Z/`, scenario 1, seeds 12–14: 2 цели / 1 поимка, 0 контактов; скорости исследователь/страж **0,194/0,210 м/с** против 0,176/0,218 на lookahead 0,5 м. Исследователь улучшился на 0,018 м/с, страж снизился на 0,008; скорость ниже 0,2 осталась в одном матче у каждой роли. Средние `planner_ok_fraction` 0,976/0,976, `moving_fraction` 0,771/0,806, повороты на месте 0,213/0,186. Средняя доля кривых путей по trace — 0,174, доля смен геометрии — 0,546; медианная ошибка курса всё ещё около 1,21–1,50 рад, а отсечение команд шлюзом — 0,128–0,385. Изменение немного выравнивает скорости и оставлено для следующей итерации, но условие 0,2 м/с для обеих ролей пока не доказано. **128 тестов**, `compileall`, `git diff --check` и `helm build duel` прошли. После серии duel-контейнеры остановлены и удалены.

## Открытые вопросы

- Проверить в Gazebo видимость соперника с разреженным LiDAR 360 × 16 в открытом коридоре и за стеной, провести несколько длительных заездов и сравнить RTF с другими плотностями лучей. Значение 240 × 12 пока не измерено. RTF около 0,52 остаётся ниже реального времени; дальнейшее ускорение симуляции открыто.
- Проверить полный проход патруля до разворота, переход стража от последней видимой позиции к обходу карты и длительный естественный заезд до поимки. Метрики теперь считаются по просьбе пользователя; пороги и агрегирование предстоит откалибровать на серии повторяемых прогонов. Потенциальные поля пока не добавлялись.
- Длительные заезды, судейское событие поимки и остановку при потере входных данных проверить отдельно. Остановка на устаревшем скане покрыта модульным тестом шлюза.
- Подставить фактические контуры стартовых площадок и настроить веса поведения под условия испытания. Текущие координаты в YAML — пример для `maze.world`.
- Подключить реальные источники локализации, карты и трека соперника; проверить на роботе и реализовать проверку судейских условий поимки/достижения площадки. Два полных стека друг против друга пока не запускались.

## 02.10.2026 — AMCL внутри start_real и выравнивание карты (текущий цикл)

Запрос: статическая карта реального лабиринта + положение по сопоставлению
карты/скана для decision manager, планировщика, MPPI; выровнять поворот карты.

Изменено:
- Официальные Humble nav2_amcl и pointcloud_to_laserscan добавлены в real образ.
  Драйвер продолжает /odom и odom→base_footprint; AMCL /amcl_pose и динамический
  map→odom. real_observations передаёт исправленную pose в navigation/self и
  облако в navigation/scan для всего автономного контура. Статический anchor
  отключён в режиме amcl, второго map→odom нет.
- config/real.yaml включает maps/maze_bag_v1.yaml, amcl, localization.yaml;
  real_match.yaml отделён от sim match.yaml. Старт0,0/yaw0.031416 — приближение
  начала записи, opponent.start остаётся примером, задавать фактическую площадку.
- AMCL likelihood_field,500–2000particles,120beams; срез base_footprint
  z0.10–0.40м, range0.25–8м,0.5°bin. Внешний YAML без пересборки.
- localization_monitor проверяет fresh sensors/pose/TF и большую ось XY
  covariance≤0.20м std, yaw≤0.35рад; номotion update каждые0.5с.
  /localization/ready10Гц, /localization/status JSON. Enable требует ready;
  observations перестаёт выдавать pose/scan при потере ready; real_match
  закрывает active и отзывает allow_motion. Восстановление без нового enable
  не возобновляет движение. Ковариация не исключает ложного совпадения стен.
- RViz показывает карту, 2D Pose Estimate→initialpose; rviz:true в real.yaml.
- Карта перерастеризована из зарегистрированных сканов с yaw+1.8° (по
  концентрации ортогональных проекций стен, поиск±5°, без знания лабиринта).
  Поворот тот же для PCD/траектории/rotation; origin[-0.50,-0.65],74×96,
  bounds[-0.50,-0.65,3.20,4.15],482occupied/4837free/1785unknown.
  Артефакты results/real-bag-check-maze-v1/map-aligned/. Стены не дорисованы.

Гипотеза: одометрия даёт непрерывное движение, AMCL исправляет её дрейф
через текущие стены; это устраняет зависимость всей навигации от static anchor.
Проверка использует сырой Livox bag, а одометрию восстанавливает из LIO и
искусственно добавляет линейный дрейф(+0.30,-0.20м). Причина: исходный bag
не содержит wheel odom/TF. Карта/reference из одной записи, поэтому ошибки
не являются независимой физической точностью. Монтаж и base offset пока
приблизительные, перенос на другую карту не доказан.

Промежуточная фактическая проверка:
- real образ собирается; pytest tests + helm_launch/tests:133passed;
  compileall, git diff --check и Compose real config проходят.
- localization-check2.json/log:1408 исходных облаков,1398navigation samples,
  ready98.93% (включая старт), median ошибки3.93см,p956.35см относительно LIO;
  внесённый конечный drift36.06см. Официальный AMCL и native MPPI активированы.
  enable через tools/real_control.py --require-localization успешен внутри
  изолированного драйвер-free контейнера domain92/networknone, один final
  cmd_vel publisher hsl_motion_gate. После прекращения потока ready/activefalse.
- Первый smoke выявил numpy.bool_ в сериализации covariance predicate —
  исправлено преобразование к bool; аварийный выход monitor закрывает launch.
- После успешного replay исправляется прежний SIGINT double-shutdown Python
  decision/planning/gate (контекст уже остановлен); промежуточный автоматический
  edit сломал отступы вложенных finally, исправлен до финальной проверки.
  Это не принятое рабочее дерево/образ. Compileall всех изменённых пакетов
  после исправления проходит. Финальная проверка последнего образа ниже.

Команда воспроизведения полного аппаратно изолированного smoke — раздел
«Проверка локализации без аппаратных драйверов» docs/OFFLINE_MAPPING.md:
tools/check_real_localization.py --exercise-permission запускает весь
robot.launch drivers_enabled=false. USB, сеть робота и другие ROS domains
не затрагиваются. Настоящий start_real по-прежнему запускается закрытым.

Финальная проверка завершена на последнем jr_real_image:
sha256:94b3f3253ee37e6918394a39695a6bd44b58c95ea391361c02362261af4a7449.
Команда выше, domain94/networknone, исходный raw bag, rate2,
robot.launch drivers_enabled=false, --exercise-permission:

| Проверка | Фактический результат |
| --- | --- |
| LiDAR / navigation/self | 1408 исходных облаков / 1397 samples |
| Ready по replay samples, включая инициализацию | 96.95% |
| Медиана / p95 XY-расхождения с зависимым LIO reference | 3.97см / 5.94см |
| Добавленный drift одометрии к концу | 36.06см |
| Разрешение движения в изолированном тесте | active_seen=true |
| После прекращения sensors | ready=false, active=false, allowed=false |
| Последний cmd_vel после sensor loss | linear.x=0, angular.z=0 |
| Издатели finalcmd | только hsl_motion_gate |
| Map server / AMCL | оба получили карту74×96,0.05м/cell |
| Завершение SIGINT | все14 процессов clean, без Traceback/process died |

Отчёт results/real-bag-check-maze-v1/localization-final4.json, полный launch
log рядом. Сводка продублирована в config/maps/maze_bag_v1_quality.json.
Собранный образ соответствует изменённым runtime исходникам; pytest133passed,
real tests18passed, compileall всех изменённых Python-пакетов, Compose config
и diff check проходят. Это не полный физический навигационный заезд:
команды не управляли записанной траекторией, аппаратные драйверы не запускались.

Вывод: статическая карта и официальный AMCL подключены ко всему real стеку;
наблюдаемое сопоставление сканов исправляет добавленный odom drift на этой
записи, motion permission/stale stop проверены фактически. Карту выровняли
единым преобразованием, сохранив исходные наблюдения/неизвестные участки.

Следующий шаг на настоящем Kobuki: подтвердить монтаж LiDAR/base offset,
задать реальные стартовые площадки в real_match.yaml, start_real с rviz:true,
уточнить initialpose, проверить наложение сканов, затем enable_real и записать
bag с wheelodom/TF/AMCL для независимых повторных проверок, скорости/контактов
и готовности. При симметричных стенах AMCL может дать уверенное неверное
совпадение, covariance не является независимой гарантией. Физическая точность,
поведение обоих ролей на этой карте и переносимость остаются открытыми.

## 02.10.2026 — ноль карты в левом нижнем углу лабиринта

Запрос: перенести ноль недалеко от прежнего, в нижний левый угол; yaw0 вправо.
Выбран внутренний угол основных наблюдаемых стен: прежние[-0.15,-0.15].
Доминирующие occupied колонка/строка имеют центры−0.175м, внутренние грани
−0.15м. Это привязка к наблюдаемой сетке, не геодезическая калибровка.
Новые map XY=прежние+[0.15,0.15], оси/углы не вращались.

Изменены YAML origin[-0.35,-0.50,0], bounds[-0.35,-0.50,3.35,4.30], обе
стартовые позиции real_match, raw_xy_shift в provenance, PCD/trajectory и
preview с отметкой(0,0)/стрелкой yaw0. Robot.start теперь[0.15,0.15,0.031416]:
физическое место/направление прежние. Opponent.start[2.99,2.25,π] остаётся
шаблоном площадки, его нужно заменить фактическим. Отрицательный origin YAML
относится к фоновому краю изображения; ноль расположен внутри него у стен.
Exporter воспроизводит эту привязку через --map-zero -0.15 -0.15 после
--align-walls. Данные в results/real-bag-check-maze-v1/map-corner/.

Проверено:
- PGM побайтно тот же (SHA256 a8d1488ad28acae0e2a2994573463841eb48a9302321b748906af3bab2b20e84).
  Геометрия, разрешение, размеры и cell counts не менялись.
- Формула raw→map и траектория сохраняют общий XY-перенос; ориентации прежние.
- Фактический Humble map_server configure/activate в изолированном container
  domain97/networknone:map frame,74×96,0.05м,origin[-0.35,-0.50],
  482occupied/4837free/1785unknown; отчёт map-corner/map-server-check.json.
- pytest tests/test_real_robot.py:18passed; py_compile exporter/diff check проходят.
- Полный AMCL replay этой работы не повторялся: прежние результаты записаны
  как проверки до общего переноса координат, не выданы за новый заезд.

Конфигурация/карта внешние, runtime не менялся: для применения пересоздать
start_real, пересборка real образа не нужна. Заданные вручную старые координаты
перенести на[+0.15,+0.15] либо выбрать новые в RViz. Физическую привязку и
наложение сканов проверить перед enable. Изменения не коммитились автоматически.

Уточнение по габаритам: текущие planner и MPPI используют круг0.23м (корпус
Kobuki0.178м плюс clearance0.052м), не измеренный контур всего оснащённого
робота. Единого поля footprint в real.yaml нет; mounting LiDAR задаётся отдельно.
Размеры фактических выступающих частей не получены, геометрию не меняли.

## 02.10.2026 — разбор пользовательского hardware startup log

Источник: logs_lidar_error.txt, переданный пользователем с реального робота;
это анализ сохранённого лога, не подключение к текущему hardware runtime.
Карта74×96 с новым origin[-0.35,-0.50] загружена, AMCL активирован с заданной
позой[0.5,0.5,0]. Kobuki подключился /dev/kobuki и сообщил версии, но несколько
раз сообщает Malformed sub-payload (парсер получил меньше байтов неизвестного
подпакета, чем заявленная length); стабильность odom этим логом не подтверждена.

Критический блокер: Livox SDK bind failed → Failed to init livox lidar sdk →
Init lds lidar fail. LiDAR не инициализирован. AMCL activation/initialpose и
request_nomotion_update не доказывают получение сканов; map→odom отсутствует,
native MPPI ждёт map←base_footprint в течение всего присланного фрагмента.
Это следствие отсутствия localization TF, не ошибка чтения самой карты.

Следующая диагностика на компьютере робота: ip -br -4 addr, sudo ss -lunp,
docker ps; сверить host192.168.88.205 и lidar192.168.88.105 с livox_mid360.json,
UDP host56101/56201/56301/56401 и наличие второго драйвера/занятых портов.
Неприсвоенный host IP и занятый порт — гипотезы, точная причина bind не
установлена по этому логу. Не назначать адрес другого компьютера вслепую.
После исправления сеть/JSON пересоздать start_real (JSON без пересборки),
проверить /livox/lidar, /localization/scan, /odom и localization/status.
Разрешение движения до восстановления сенсоров/готовности не выдавать.
Runtime/config по этому разбору не изменены; аппаратный успех не подтверждён.

## 02.10.2026 — разбор первого физического автономного заезда и правки

Пользователь сообщил: recovery на видимом свободном участке, движение к стене
без контакта, прибытие к конечной площадке с последующим движением назад.
Подключение SSH к tb2 выполнено с разрешения пользователя; репозиторий найден
в /home/tb2/hackathon2026, HEADa5e617c. Старт/цель там[0.5,0.5,0]/[0.5,3.5,0].
Изменённый пользователем real_match сохранён и не заменён локальным шаблоном.
Фотографией/сообщением подтверждена площадка0.5×0.5м в центре ячейки1×1м:
исправлена наша half_size0.5→0.25, у соперника0.25 уже было правильно.

Сохранены присланный/полный container logs; локально
results/real-bringup-hardware-20261002/remote-container.log. В старом автономном
запуске не писался bag: существующие recordings относятся к прежнему teleop,
не к этой поездке. Нет синхронных поз/path/costmap/intent/commands, поэтому
точные причины всех трёх эпизодов из console log восстановить нельзя.
В новом логе Livox Init success, карта/AMCL/nativeMPPI активировались;
MPPI несколько раз Optimizer fail to compute path, Kobuki по-прежнему иногда
Malformed sub-payload. В полном console log нет сообщения global watchdog
No translation: изменение этой ветки ниже — устранение найденного риска,
не доказательство причины именно пользовательского recovery.

Изменено:
1. RealMatch проверяет каждую свежую navigation/self map-позу исследователя
   против центра миссионной площадки0.08м, сразу закрывает active, завершение
   запоминается и permission отзывается. Не ждёт следующего decision tick
   или единственного текстового reason. В DecisionPolicy также finish latch:
   шум позы после достижения не запускает новую поездку назад. Цель всё ещё
   центр площадки, не касание её границы; допуск не увеличен.
2. Watchdog планировщика после4с задержки не объявляет свободную точку или
   actualgoal виртуальным препятствием. recovery_avoid и принудительный escape
   применяются к подтверждённо blocked candidate. Нет маршрута по-прежнему
   может вызвать проверенный escape. Границы footprint/скорости не менялись.
3. Start_real автоматически создаёт recordings/<UTC>-autonomous, сохраняет
   hardware/mission/localization/map, dirty code patch, запускает MCAP с
   lidar/IMU/odom/TF/map/AMCL/localization, pose/opponent/intent/path,
   planning/MPPI diagnostics, costmap, match и cmd_vel. Stop_real закрывает
   движение, завершает recorder и проверяет metadata для autonomous тоже.
   Внутренний session_dir только для запуска/проверок, отдельного пользовательского
   сценария карты не добавлено. Запись прекращает стек при аварийном выходе
   recorder, как и другие обязательные процессы.

Сборка/проверки:
- Локально pytest tests:134passed (новый regression — explorer finish после
  смещения позы остаётся STOP); compileall/diffcheck/real Composeconfig проходят.
- ROS high-rate goal check без hardware на локальном компьютере и на самом tb2:
  активный этап, pose0.5/3.3 вне допуска ещёactive, один sample0.5/3.45 завершает,
  следующая pose0.5/3.3 не возобновляет. allowed отозван, повторный allow не
  открывает completedstage. Нет терминального intent вfixture: независимый
  geometricguard проверен отдельно отdecision. Итог active=false,finished=true.
- Короткий изолированный replay180 исходных LiDAR:pose167, AMCLready до потери
  sensors, после прекращения ready/active/allowed=false,cmd_vel[0,0], один
  finalpublisher. MCAP финализирован с4214messages, включая self167, intent90,
  planningdiag86, MPPI199, cmd361. Это ROSintegration на прежнем bag, не
  физический заезд; recordedtrajectory не управляется командамиfixture.
- Исходники скопированы на tb2 с резервными копиями исходных runtime/config,
  образ там пересобран; measuredstarts сохранены. Оборудование выключено:
  физическая проверка нового поведения не проводилась. Отдельные driver-free
  container tests не управляли USB/LiDAR и не разрешали hardware движение.

Осталось: получить новый autonomous bag фактического заезда, проверить
локализацию/оппонент detector/markedcostmap/optimizer rejection в эпизодах
стен/recovery; подтвердить попадание оценённой и физической позы в центр.
8см нельзя интерпретировать как точность реального AMCL. Параметры MPPI и
радиус защиты вслепую не ослаблялись. Изменения проверены локально и установлены на роботе; по запросу пользователя
02.10.2026 подготовлен Git checkpoint для отправки в origin. Предыдущий
checkpoint a5e617c сохранён в истории. Повторная проверка перед коммитом:
pytest tests — 134 passed, git diff --check — без ошибок.

Финальное состояние deployment:
- В исправленной audit-проверке costmap topics /native_mppi/costmap и
  /native_mppi/costmap_raw действительно записаны (19 сообщений каждого),
  MCAP финализирован. /costmap_updates может быть пустым при full-map публикации.
- Окончательный образ tb2 sha256:67f85a2d6c1bb12abee1ce7e738258cc4b86d6a89f93baf4ac6b034a67631759.
  Хеши пяти изменённых runtime/host файлов сверены с локальным деревом.
  Резервные копии/previous-drive.log/ROS logs на самом роботе сохранены в
  results/real-bringup-hardware-20261002 (ignored), новые measured poses не менялись.
- Во время работы появился пользовательский запуск промежуточного образа;
  он штатно остановлен с pause+SIGINT, metadata записанной сессии подтверждена,
  контейнер пересоздан на финальном образе через start_real. Новый каталог
  recordings/20261002T122723.604552Z-autonomous, движение изначально закрыто.
  База/лидар для фактической новой поездки не включались агентом.
- Высокочастотный ROS finish regression сохранён в tests/ros_goal_stop_check.py;
  запускать в изолированном real image без доступа к устройствам, как временный
  /tmp/hsl-goal-stop-check.py, использованный на локальном компьютере и tb2.
- Read-only ROS проверка фактически работающего финального контейнера tb2:
  /match/allowed=false,/match/active=false,/cmd_vel=[0,0]. Новая сессия работает
  с закрытым движением; включение разрешения агентом на реальной базе не делалось.

## 02.10.2026 — замена симуляционного мира на polygon_rosbag

Запрос: перенести реконструированный реальный лабиринт из feature/rosbag_map,
заменить старый мир и построить согласованную статическую карту.
Источник мира: origin/feature/rosbag_map, commit
3744e75e732d57678473c633fa71b5bb177940b7; перенесён только polygon_rosbag.world,
геометрия 10 box-панелей сохранена. Старый maze.world удалён, доступен из Git.

По уточнению пользователя map[0,0] — внутренний нижний левый угол стен:
world[-0.468,-0.582], +X вправо, +Y вверх, yaw0 вправо. Старты map[0.5,0.5,0]
и[0.5,3.5,0], обе площадки0.5×0.5м. Фактический Gazebo spawn[0.032,-0.082,0]
и[0.032,2.918,0]. Bounds[-0.025,-0.025,3.065,4.05]. Конфигурация реального
робота и bag-карта maze_bag_v1 не изменены; их frame не подменяется.

Изменения: match.yaml, .env, Compose и launch/defaults синхронизированы;
jr_map/sdf_geometry.py выделяет прежнюю геометрию в модуль без ROS.
И живой /map, и tools/export_sdf_map.py используют этот же растеризатор.
Сохранён config/maps/polygon_rosbag.yaml + pgm:102×122,0.05м/ячейка,
444 occupied, срез0.25м,padding1м,origin[-1.0244244094,-1.0205000367,0].
Origin изображения включает поля и не совпадает с внутренним нулём map.

Команды: python3 tools/export_sdf_map.py; helm build duel; helm start_match.
Проверка pytest tests:135passed; новая проверка сравнивает YAML/PGM с
растеризацией и проверяет доступность стартов/маршрут A* с радиусом0.23м.
Fixture runtime-audit исправлен: referee получает origin map, независимо
от собственной стартовой позы; production audit уже проверял это правильно.
Исторические серии старого maze не являются доказательством нового полигона.

Фактические проверки после сборки:
- helm build duel завершён, окончательный jr_image
  sha256:d32fccf33aef16fe4c469136a4e827d95dd41c17a0542cf1739bcb4cd057746d.
  Compose config, compileall, git diff --check проходят.
- Изолированный ROS-запуск jr_map.launch.xml подтвердил map→odom translation
  [0.468,0.582,0] и карту444 occupied без ошибок запуска.
- Реальный Gazebo прогон: python3 benchmarks/run_duel_series.py
  --isolated-project hsl-polygon-check --ros-domain-id 74 --gazebo-port 11419
  --runs 1 --start-seed 0 --active-s 30 --trace --audit-start
  Полные артефакты results/isolated/hsl-polygon-check/series-20261002T130232Z.
  Runtime audit сверил исходники/параметры; gate before/partial-start passed.
  Начальные navigation/self map-позы[0.499635,0.500200] и[0.499749,3.500202],
  yaw≈0. Все ячейки живого /map побитово совпали с экспортом PGM;
  снимок results/isolated/hsl-polygon-check/map-audit.json.
- Первый исход guardian_capture после10.8с активного sim-времени (seed0),
  оба остановлены, затем оценочный мир очищен. Explorer:2.989м,средняя0.277м/с,
  moving0.926;guardian:3.866м,средняя0.358м/с,moving0.898. Коллизии0 у обоих,
  planner_ok0.889 у обоих,RTF0.606. Есть короткие RECOVERY/NO_GLOBAL_PATH у
  explorer и NO_LOCAL_PATH у guardian; это проверка подключения нового мира,
  не доказательство устойчивой навигации/баланса ролей. Цель explorer в этом
  заезде не достигнута (его поймали). Следующий шаг при развитии алгоритмов —
  повторная серия уже на этом полигоне, без ссылок на успехи прежнего мира.

## 02.10.2026 — неизвестные коробки, слой препятствий и подготовка испытаний

Запрос пользователя:2–3 коробки15×15×40см и1 коробка40×60×20см, произвольный
yaw, вне статической карты; отладить в Gazebo и убрать отказ в диагональном
проходе. Позже пользователь сузил текущий этап: НЕ менять detector, добавить
коробки и подготовить тест; новый detector он подключит отдельно.

Цикл1. Гипотеза: только текущий скан не сохраняет неизвестные препятствия,
а дополнительный safety_margin0.12–0.14 поверх protected robot_radius0.23
отвергает проходимые проходы. Изменено: bounded ObstacleMemory8с с очищением
по новым лучам; отдельный obstacle_grid объединяет static grid и наблюдения,
публикуется5Гц; общий слой для глобального A* и native MPPI. Исходный /map
не меняется, это не SLAM. Номинальный дополнительный margin снят; footprint
остался0.23. Inflation A* учитывает реальное положение точки относительно
центра клетки; swept-edge проверяет реальные расстояния вместо правила
двух боковых inflated клеток, кэширует геометрию. Unit-тест: диагональный
коридор0.60м проходится защищённым диском0.46м, прежний margin требовал0.74м.

Цикл2. Проверка привязки LiDAR на stamp сначала обнаружила TF extrapolation:
сканы приходили раньше соответствующего TF и все отбрасывались. Исправление:
очередь8 сканов, process_cloud ждёт доступный TF, не подменяет время latest TF.
Проверка: scan появился, native_ready обоих true; image sources audit прошёл
после промежуточного копирования observations.py в private контейнеры.
Это экспериментальное обновление, окончательный образ пересобирается отдельно.

Контролируемый Gazebo заезд до уточнения пользователя: hsl-boxes-check/domain75,
Gazebo port11420; seed0,3 узких +1 широкая, yaw0/45/90/135deg. Decision и referee
private проекта отключены; фиксированные GOAL intents из test harness, второй
робот удалён с goal площадки. Это НЕ оценочная дуэль. Коробки выбирались по
статическому маршруту с существующим альтернативным путём и SpawnEntity;
ground truth коробок используется лишь для измерения зазора, не навигации.
Результат: goal radius0.08 достигнут после21.97с;путь5.916м,средняя0.269м/с,
min body-to-box clearance0.223м (disk0.178). Planner samples:199OK,3WAIT_OR_STOP,
без recovery. Observed occupancy621–742 против static444. Отчёт
results/obstacles-20261002/seed0.json. При этом202 samples с ложным rival track
возле коробок: дефект detector подтверждён; применять этот результат к
автономной дуэли нельзя. Попытка изменить detector полностью отменена по
указанию пользователя; src/hsl_perception идентичен HEAD.

Подготовка к следующему тесту: config/simulation_obstacles.yaml содержит те
же4 коробки и проверенные позиции, допускает изменение pose/size без сборки.
Штатный spawn_obstacles добавляет их в Gazebo отдельно от world, publishes
simulation/obstacles_ready/status; start_match ждёт ready до permission,
runtime snapshot сохраняет cfg/hash. enabled:false отключает fixtures.
RViz показывает отдельный obstacle_grid. README/AGENTS обновлены.

Проверки текущей сцены:141pytest passed,compileall/diffcheck; start_match
--prepare-only в hsl-boxes-check завершён за53с. Audit
results/obstacles-20261002/prepared-audit.json:4 коробки ready; все ячейки
/map совпадают с polygon_rosbag.pgm; allowed/peer_allowed/activefalse;
обе cmd_vel[0,0]. Source/parameter runtime audit подготовленного мира прошёл.
Реальные драйверы/оборудование не запускались. Дальнейшие заезды по последнему
указанию пользователя отложены до нового detector; статические/перемещённые
коробки при разных seed, очистку слоя и физический диагональный проход ещё
нужно подтвердить повторными заездами. Текущая доказанная езда — один
контролируемый объезд, не серия и не соревнование двух автономных ролей.

Финальная сборка: helm build duel прошёл, jr_image sha256:77144401105ae01374b3d322a38d597c14d02fa5847548a8301d10506bb05a06.
Хеши8 ключевых source/config файлов внутри образа совпали с рабочим деревом;
включая исходный detector. Подготовленный private мир и его init очищены,
чтобы не снижать RTF последующего пользовательского запуска. Для визуального
осмотра без езды: python3 benchmarks/start_match.py --prepare-only.
После подключения нового detector — пересборка и совместный start_match.

## 02.10.2026 — реальные bags, политика узких коробок и разнесённая сцена

Пользователь предоставил25 сессий в
/home/eddyswens/ROS/hsl2026Extra/recordings_2oct_18_36_time/recordings.
Условия коробки/соперник не размечены. Роли прочитаны из сохранённых миссий;
source revision у late sessions9fa5edc, у некоторых dirty только real_match.
Это записи прежнего CPP detector, НЕ развернутой8f176c7 shape-версии.
Исходные bags/configs не менялись, оборудование/SSH не использовались.

Цикл1 — диагностика. audit_real_bags.py запускается в jr_image --network none,
без ROS publishers.25 сессий прочитаны,14 имеют восстановимые LiDAR облака
через записанный map<-sensor TF на stamp;0.2с subsampling. У одной сессии
145851 нет metadata.yaml, отдельный MCAP прочитан без изменения входа.
Ранние сессии с нулём navigation/self — только частичная диагностика.
Артефакты results/real-bags-20261002: audit-all.log, summary.json,
индивидуальные JSON/NPZ, obstacle_summary.json, cloud_overview.png.
Отсутствие разметки не позволяет измерять semantic precision/recall.
Записанный opponent также НЕ ground truth. Перепроигрывание source/simulation
shape-профилей не равно работе трекера на полной частоте.

В late guardian151820/152104/152152 measured average0.083/0.084/0.058м/с.
Скорость интегрирована по own wheel twist в recorded active window с
покрытием в speed_time_coverage_s; это не симуляционная оценочная серия.
MPPI swept_collision557/81/1013 samples; median rejected index13/не сводился/19.
Есть как ближайшие отказы, так и отказ по дальней части3с прогноза.
Проверку swept footprint не снимали. У explorer152423 speed0.206,
recovery42.3с из93.55с active. В recorded_tracks_vs_static.json видно
110/430 старых opponent центров на known wall в150325,69/209 в152104,
144/770 в152152; собственных центров на wall0. Это кандидаты на дефект
старого detector/привязки, не доказательство семантической разметки всего bag.
Часть recovery сопровождается EVADE от таких треков; не лечить её снятием
коллизионных проверок или скоростными ограничениями.

Цикл2 — политика по явному запросу:15×15×40см игнорировать и разрешать
толкание,40×60×20см учитывать. SmallBoxFilter в hsl_planning берёт только
raw scan+known_grid. Проверяет yaw-independent грани15см, высоту/заполнение
середины; known wall возвращает без удаления. Низкие/широкие/неоднозначные
кластеры остаются. Одни filtered points кормят memory/A* и obstacle_scan
для MPPI; raw scan остаётся detector. Старые отметки распознанной коробки
явно стираются из memory. Real MCAP теперь также включает raw/filtered
navigation scan, obstacle_grid, filter diagnostics после build_real.

Первый строгий вариант (.015 edge fit,6 vertical bins) не распознавал
коробки с шумом Gazebo. Серия3 seeds0/1/2 с прежней расстановкой,
series-20261002T161055Z: capture21.7/29.4/20.8с, explorer speed
0.215/0.241/0.247,guardian0.263/0.194/0.278,0 контактов,RTF0.485–0.514.
Это НЕ подтверждение игнорирования коробок: ignored scans0 в доступных
traces. Дальнейшая адаптация: percentile5/95 bounds, edge tolerance.04,
3 vertical bins плюс middle_share>=.20, ограниченные размеры,partial cache8с.
442 полных сохранённых симуляционных clouds start/gap/baseline: исключены
161/124/67 кластеров возле узких fixtures,0 возле peer truth и0 других.
Ground truth используется только offline label; это не полная recall-проверка.

Цикл3 — физика/расстановка.3 small fixtures movable:true mass0.10kg
friction0.30, collision сохранена; broad box static. Масса/трение не измерены.
По следующему запросу разнесены: small1[1.40,3.40,0],
small2[2.50,3.65,pi/4],small3[2.40,0.55,pi/2],large[1.05,1.20,pi/4].
На подготовленном headless мире29 полных clouds, guardian одновременно
видит и распознаёт обе верхние коробки (обычно68–74 исключённых points).
Артефакты spread-scene-clouds.json/spread-scene-filter.json.

Контролируемый push harness в private hsl-small-box-check/domain77/port11422,
решающие контейнеры/referee отключены, другой робот остаётся стоять на своём
старте. Начало[.5,.5],goal[2.4,.5],box[1.2,.5],yaw0. Это НЕ дуэль;
нет teleport robot или box truth в navigation. Два предварительных fixture
сбоя до движения: DeleteEntity opponent подвесил Gazebo; затем ожидание
неподключённого /gazebo/model_states. Исправлено: robot не удаляется,
final box pose измеряется Gazebo transport gz model -i; active heartbeat
публикуется постоянно. Эти сбои не выдавать за результаты навигации.

Первый полноценный push: коробка сдвинута1.181м, но goal false за45.07с,
путь3.539м/средняя0.079;153 NO_LOCAL_PATH samples. Только small-box контакты
в contact_pairs. Артефакт push0-initial.json. Гипотеза подтверждаемого следующего
цикла: второй marking слой Nav2 сохраняет точки распознанной/удалённой коробки.
Обновлён shared occupancy: obstacle_grid также на каждом скане; ObstacleLayer
marking:false,clearing:true, занятость принадлежит bounded shared grid.
Подробные итоги повторного push и новой серии записываются ниже.

Промежуточные проверки160 pytest passed,compileall; native image пересобирается
после изменения marking. Metrics теперь small_box_contacts отдельно;
collisions продолжает включать все контакты, wall_collisions не включает
small fixture pushes. Hardware image пока не обновлён, не развернут.

Повторный push с marking:false всё ещё goal false45.06с,путь4.067м,
speed0.090,box displacement1.173м; push0-grid-only.json. Гипотеза о двойном
слое сама по себе недостаточна. Диагностика конца заезда содержит только
1–4 sparse returns; первоначальный partial cache их не обслуживал. Исправлено:
known box sparse continuation (не initialization),cache8с как occupancy TTL,
expire/reset сохранены. Unit sparse/init/expire добавлен,161tests passed.
Для чистого controlled fixture warmup10 sim s после удаления прежних boxes,
чтобы исключить residual memory. Одна новая подготовка упала с gzserver -11
при спавне второго робота до permission; повторяется тот же старт без движения.
Не включать infrastructure failure в навигационные успехи/провалы.
Последний offline replay442 clouds:172/137/67 исключённых cluster рядом с
узкими boxes,0 рядом с peer truth,0 other;filter-offline-sim.json.

Цикл4 — sparse continuation проверен на физическом controlled push yaw0
с clean warmup10sim s. push0-contact.json: goal true за11.81с,
путь2.082м, mean0.176м/с, final box displacement1.424м, коробка стоит
вертикально. Contact pairs только own/small box. NO_LOCAL_PATH16 из109
samples: улучшение относительно двух предыдущих неудачных45с push, но
минимальная средняя0.2 ещё не подтверждена. Это НЕ полноценная дуэль.
Образ jr_image sha256:5cf30043aa012f4efaeb885539118a9d03cac9026b393ce4c701725e9a4cd59c.
Повторная подготовка перед yaw pi/4 и финальная серия выполняются ниже.

Controlled yaw pi/4 push45-contact.json: box displaced1.506м, остаётся
вертикальной, contacts только small box. Центр goal за45.06с НЕ достигнут:
ближайшее расстояние0.099м против допуска0.08м, последние samples OK с почти
нулевым движением. Путь2.979м/mean0.066; NO_LOCAL_PATH4,NO_GLOBAL_PATH35
из417 samples. Исчезновение box returns в self-mask больше не вызывает
длительного NO_LOCAL_PATH, но остаточная ошибка/физический контакт у цели
остаётся отдельной незакрытой проблемой. Не выдавать успешное перемещение
коробки за успешное достижение центра и приемлемую среднюю скорость.
Текущий реальный offline report:14 sessions,4174 sampled clouds,2933 frames
с small-box гипотезами (не semantic labels). В активных окнах старых bags
151820/152104/152152 swept failures556/79/1011; значения чуть меньше
инвентаризации всего файла выше, поскольку отфильтрованы по active window.

### Итог текущего цикла: разнесённые коробки, 02.10.2026

Финальный образ и исходники проверены свежими заездами:
`series-20261002T170855Z`, revision `8f176c7+dirty.e11369b3793b`.
Команда:

```bash
helm build duel
python3 benchmarks/run_duel_series.py --isolated-project hsl-small-box-check \
  --ros-domain-id 77 --gazebo-port 11422 --runs 3 --start-seed 0 \
  --active-s 90 --trace --audit-start
python3 benchmarks/report_detector.py results/isolated/hsl-small-box-check/series-20261002T170855Z \
  --output results/isolated/hsl-small-box-check/series-20261002T170855Z/detector-report.json
python3 benchmarks/report_straight_motion.py results/isolated/hsl-small-box-check/series-20261002T170855Z
```

Начала: исследователь `[0.5, 0.5, 0]`, страж `[0.5, 3.5, 0]`;
расстановка коробок — текущий `config/simulation_obstacles.yaml`.
Все три матча — независимые миры, оба стека автономные. Подтверждены
runtime hashes и параметры, запрет команд до общего старта и после финиша,
один final command publisher. Окна обеих метрик совпадают с referee.
Это отладочная серия с лимитом 90 с, НЕ финальная 20-серия на 360 с.

| Seed | Исход / время, с | Исследователь, м/с | Страж, м/с | Контакты со стенами | Контакты с маленькими коробками | RTF |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | Поимка / 29.9 | 0.210 | 0.175 | 0 | 1 (исследователь) | 0.492 |
| 1 | Поимка / 20.3 | 0.217 | 0.254 | 0 | 0 | 0.487 |
| 2 | Поимка / 20.3 | 0.233 | 0.257 | 0 | 1 (исследователь) | 0.481 |

Агрегированные средние по заездам: исследователь 0.220, страж 0.229 м/с;
доли обычного движения 0.919 / 0.802, planner OK 0.777 / 0.907.
Не выдавать среднее серии за выполнение порога в каждом матче:
страж в seed 0 ниже 0.2 м/с. Целей исследователя 0, поимок стража 3.

В активных referee-окнах scans с исключёнными узкими коробками:
исследователь 246/280, 152/193, 164/198;
страж 279/291, 189/196, 192/201. Видимая большая коробка сохраняется
как широкий низкий кластер. Политика проходит через общий obstacle_grid
и obstacle_scan, без изменения raw detector cloud.

Детектор: 858 согласованных с pose соперника оценок, ни одной ошибки
более 0.3 м; p90 по роли/заезду 0.026–0.055 м. Это проверка ошибки
доступных треков, не полная recall/semantic precision. Box-near evaluator
использует НАЧАЛЬНЫЕ позы fixtures, не положение после толкания; эта
граница теперь явно указана в `report_detector.py`.

`straight-report.json`: RMS бокового отклонения от наблюдаемого прямого
reference 0.038–0.039 м у исследователя, 0.075–0.110 м у стража.
Время наблюдаемого прямого движения всего 4.8–9.5 с на роль/матч;
результат не доказывает отсутствие коррекций во всех коридорах.
У стража 4–5 смен знака omega за эти интервалы, плавность требует улучшения.

Финальные проверки: 161 pytest passed; compileall и git diff --check;
Compose simulation/real config валидны. Только симуляционный образ собран.
Private runtime и его init/daemon удалены после проверки; другие проекты
не затрагивались. Изменения этого цикла пока не закоммичены и не отправлены;
отдельный detector checkpoint `8f176c7` уже отправлен по запросу пользователя.

**Следующие шаги:**

1. Воспроизвести остаточную ошибку 0.099 м у цели после толкания yaw 45°.
   `push_box_trial.py` дополнен записью measured twist и mppi diagnostics
   для следующего повтора; имеющиеся push JSON созданы до этого дополнения.
   Разобрать reference/MPPI costs и физический контакт, не ослаблять swept safety.
2. Разобрать seed 0 стража и реальный bag `152152`: различить отказы
   полной прогнозной траектории, ложные старые треки и recovery; карта/TF
   в реальных bags местами имеют заметное рассогласование, нужна проверка
   привязки перед выводами о семантике облаков.
3. Поднять уверенность исследователя при уклонении: эта серия не дала
   ни одного достижения цели. Не подгонять исходы seed или ухудшением стража.
4. Повторить настоящие заезды после `helm build_real` с записью новых
   raw/filtered cloud, grid, filter diagnostics. Реальная эффективность
   фильтра по неразмеченным старым bags НЕ доказана; масса/трение fixtures
   пока приближённые. Для финальной оценки нужны 20 независимых матчей
   с лимитом 360 с после устранения оставшихся дефектов.

## 02Oct evening — real standstill: own LiDAR returns and detector TF

Пользователь прервал общую отладку коробок: три свежие записи в
`/home/eddyswens/ROS/hsl2026Extra/вечерняя_отладка/2031` показывают почти
неподвижного робота. Условия без точных меток: сначала коробка/возможно
соперник примерно в метре, затем убраны; широких креплений нет. Четыре
вертикальные штанги находятся на границе штатной базы Kobuki.
Исходники всех трёх записей: checkpoint `8f176c7`, локальная правка роли.
Предыдущие дневные записи относятся к другой версии `9fa5edc`.

### Диагноз исходных записей

| Запись | Роль | Средняя активная wheel speed, м/с | Detector diagnostics | Swept rejection на первом шаге |
| --- | --- | ---: | ---: | ---: |
| 172454.810732 | исследователь | 0.0228 | 0 | 579 |
| 172638.815974 | исследователь | 0.0345 | 0 | 691 |
| корень 2031 / 172941 | страж | 0.0147 | 0 | 712 |

Разрешение движения и готовность локализации выдавались. Детектор ожидал
TF `livox_frame`, тогда как реальное оборудование публикует `livox`:
обработки облаков и opponent output нет во всех трёх записях. Второй
недостаток: устойчивые ближние возвраты в четырёх направлениях движутся
вместе с корпусом, проходят прежнюю XY self mask 0.25 м и растеризуются
в lethal costmap около контура. Например, при own=[0.5,0.5] клетка
[0.575,0.275] имеет cost254 на расстоянии 0.237 м. MPPI отвергает движение
почти сразу. Это объясняет проблему даже после удаления внешней коробки.

### Исправление

- `sensor_frame` параметризован: real=`livox`, simulation=`livox_frame`.
- Независимый от TF `real_lidar_filter` перед AMCL и real_observations:
  четыре угловых сектора в LiDAR frame, центры ±15°/±165°, полуширина5°,
  3D дальность≤0.45м; low/reserved confidence группы tag удаляются,
  medium и верхние reserved bits сохраняются. Параметры внешние:
  `config/lidar_filter.yaml`, путь в real.yaml. Это измеренная дальность
  помех, не физический размер штанг/базы. Общая radial mask не увеличена.
- Raw topic сохраняется; filtered cloud с неизменными header/point fields
  и filter diagnostics добавлены в автономную запись. YAML копируется
  в session. Ручная запись остаётся сырой.
- `enable_real` требует свежие detector processing diagnostics; обнаруживать
  соперника для разрешения не требуется. Ошибка TF больше не скрывается
  за готовым MPPI. Без static map детектор не готов, автономный enable
  закрыт; cloud marking без карты включён, чтобы не получить пустой costmap.
- При static map grid владеет obstacle marking, cloud выполняет clearing.
  Collision footprint/swept checks и лимиты скорости не ослаблены.

### Проверки на данных и ROS

Полный replay сырого LiDAR, scan-time TF, raster0.05м, клетки в пределах0.27м
от собственного положения (это входные клетки, не измерение столкновений):

| Запись | Обработано облаков | Облака с ближними клетками до / после | Число клеток до / после |
| --- | ---: | ---: | ---: |
| корень 2031 | 802 | 528 / 2 | 825 / 2 |
| 172454 | 814 | 391 / 4 | 509 / 4 |
| 172638 | 1165 | 422 / 1 | 513 / 1 |

20 облаков пропущены из-за отсутствующего TF. Данные:
`results/real-bags-evening-debug-20261002/*-lidar-filter.json`;
`near-self-body-frame.png` показывает возвраты в системе корпуса.
Фильтр сохраняет примерно97.45% всех исходных точек. Сектора могут также
удалить часть реального близкого препятствия: при смене крепления требуется
повторная калибровка. За штангой нет достоверного измерения скрытого предмета.

Изолированный stationary ROS replay первых20 облаков записи172454:
новый real образ, network none, own pose неподвижна, без AMCL/драйверов/gate
и финального cmd_vel. 15 секунд, одинаковая запись и исходная pose:

| Режим | Detector diagnostics | MPPI ok | optimizer failure | swept collision |
| --- | ---: | ---: | ---: | ---: |
| исходный frame/нефильтрованный вход/cloud marking | 0 | 245 | 22 | 21 |
| исправленный frame/фильтр/shared grid | 25 | 282 | 0 | 6 |

Файлы `start-before-real.json`, `start-after.json`. Этот snapshot НЕ
воспроизвёл полную исходную неподвижность: baseline тоже выдавал команды.
Он подтверждает устранение TF-сбоя и доступность расчёта на стационарном
входе, но не достижение цели/скорость/отсутствие столкновений. MPPI стохастичен,
один snapshot не является статистическим сравнением эффективности.
Ранее `start-before.json` запускался в simulation image; для сравнения выше
использован повтор в одном real image. Завершающие shutdown traceback в
snapshot ROS log возникают при остановке дочерних процессов после отчёта.

Команды повторения (в изолированном контейнере):

```bash
helm build_real
# source /solution/install/setup.bash внутри jr_real_image
python3 /work/benchmarks/replay_lidar_filter.py --help
python3 /work/benchmarks/replay_real_start.py /bag-session --baseline --output /work/results/real-bags-evening-debug-20261002/start-before-real.json --seconds 15
python3 /work/benchmarks/replay_real_start.py /bag-session --output /work/results/real-bags-evening-debug-20261002/start-after.json --seconds 15
```

Real image успешно собран: `051989790b7f88339b05676b4d3374060c5b26ab864bee007912df79d518432d`.
167 pytest passed,1 skipped (ROS serializer); внутри real image serializer
проверен отдельно: обе endian-разметки, organized cloud с row padding,
сохранение timestamp/tag/полей/header. Compileall и оба Compose config
валидны; у simulation Compose без окружения предупреждение VEHICLE_ID.
По просьбе пользователя замеры производительности не продолжать.
На оборудование не подключались, не запускали драйверы и не двигали базу.

### Сохранённая предшествующая отладка реальных коробок

Real detector profile в `profiles.py`: max_gap_share0.25,line_ratio0.50,
strong extent0.25,arc75°,inlier0.80,merged strong=false. Strong extent
применяется только к новым трекам; partial weak может продлевать трек.
Fit center проверяется по known-free static map; unknown не foreground.
Source segmentation/tracker остаются byte-identical. Box filter защищает
кластер возле свежего измеренного соперника≤0.3с, до shape/cache exclusion.

14 дневных записей,4174 sampled clouds: overlap свежего трека с отдельной
узкокоробочной гипотезой547→43, fit center в wall/unknown115→0. Три поздних
записи full-rate2630 clouds: overlaps149→9, invalid centers16→0,
fresh outputs1207→1464. Это диагностические прокси на неразмеченных данных,
не semantic precision/recall и не доказательство успешной реальной езды.
Артефакты `results/real-bags-debug-20261002/full-rate-comparison`,
`tracks-before-after.png`. Эти изменения ещё не были в вечерних заездах.

**Следующий шаг:** на реальном оборудовании пересобрать/перезапустить,
проверить filter и detector diagnostics, повторить автономный заезд с
автоматическим bag. Проверить оставшиеся ближние клетки, AMCL привязку,
скорость, recovery и стоп у центра цели. Только затем возвращаться к общей
проверке коробок/детекции и симуляционной серии. Изменения не закоммичены.

### Уточнение после реальной проверки фильтра

Фильтр штанг оформлен отдельным коммитом `197fdb4` и отправлен в origin по
просьбе пользователя. Остальные изменения по коробкам/профилям остаются
в рабочем дереве. Пользователь сообщил: фильтр помог лишь немного.
Запрошен новый bag после фильтра; в локальном hsl2026Extra пока видны
прежние записи. Не объявлять проблему движения устранённой.

История показывает важную регрессию схемы: `7c71cee` переключил native
static_layer.map_topic с /map на /navigation/obstacle_grid и добавил память
8с. Ранее9fa5edc локальный Nav2 obstacle layer уже marking/clearing LiDAR,
статическая карта не менялась, planner использовал текущие scan_points.
Новые помехи могут переиздаваться из нашей памяти через static layer;
очистка облачного obstacle layer не удаляет их из upstream памяти.
Это объяснение по коду, не доказательство полного причинного эффекта.

A/B stationary15с /first20 clouds172454, без фильтра, одинаковый real image:
обычная observed_grid против передачи исходной static map в native grid
(remap только publisher planner + replay publisher static grid). В memory
режиме2 из27 costmap публикаций содержали1 lethal клетку<0.27м; в legacy
режиме0 из27. В обоих запусках преобладал STALE_INPUT (255/240 из277/280
диагностик), поэтому сравнение moving commands/скоростей непригодно.
Файлы costmap-memory-ab.json / costmap-legacy-ab.json в вечернем debug dir.
Следующий шаг: вернуть native static/map + штатный облачный obstacle layer,
память оставить для глобального перестроения; проверить отдельно и повторить
проезд коробок. Нельзя маскировать проблему отключением swept collision.

## 02Oct late evening — boxes and costmap ownership correction

### Box/detector cycle preserved after filter checkpoint

Серия `series-20261002T181501Z`, revision197fdb4+dirty.8b02309c5525,
private domain77/port11422, seeds0/1/2, active cap90с:

| Seed | Исход / сим-с | Скорость E/G, м/с | Стены E/G | Small-box contacts E/G |
| --- | --- | --- | --- | --- |
| 0 | capture /23.3 | 0.222/0.217 | 0/0 | 1/0 |
| 1 | capture /21.0 | 0.221/0.249 | 0/0 | 1/0 |
| 2 | capture /15.8 | 0.267/0.316 | 0/0 | 0/0 |

Большие низкие кластеры сохраняются; например first trace seed0:134 wide-low
retain records,0 discarded. Но explorer seed1 имеет ошибку rival max1.889м:
трек у угла большой коробки. Это критический недостаток, а не подтверждённая
успешность детектора. Финальная20-серия не выполнена, целей E0.

В вечерних filtered raw replay317+455 облаков: узкокоробочные overlaps
треков100→0 при real profile,6 wide-low clusters retained/0 ignored.
Fresh outputs218→154 и97→0; реальные семантические labels отсутствуют.
Нельзя утверждать, что все исчезнувшие tracks ложные. Файлы:
results/real-bags-evening-filtered-20261002 и
results/real-bags-evening-box-comparison-20261002.
`audit_real_bags.py --filter-lidar` использует production filter до map TF.

### Попытка различить угол и дугу

Добавлен shape_confidence.rectangle_edge_mse: rotated rectangle robust
5/95 bounds, без размеров/координат fixtures. strong_rectangle_ratio0.70
в simulation понижает кандидата до weak, если rectangle edge MSE меньше
70% circle MSE. Real default0: на оборудовании ещё не включён.
Source segmentation/tracker byte-identical. Синтетический L-corner тест
воспроизводит старое false birth и его предотвращение, круговой корпус
сохраняется. Saved replay344 clouds:136 peer-near estimates до/после,
208 no_detection до/после; не полная recall/precision.

Следующая live серия `series-20261002T183245Z`:

| Seed | Исход / сим-с | Скорость E/G, м/с | Стены E/G |
| --- | --- | --- | --- |
| 0 | capture /15.2 | 0.214/0.238 | 0/0 |
| 1 | capture /20.2 | 0.222/0.244 | 0/0 |
| 2 | capture /18.6 | 0.247/0.184 | 0/0 |

Проверка угла НЕ решила все ложные треки: explorer seed0 max error1.606м.
По диагностике один strong candidate на29.236с открыл трек, два следующих
weak подтвердили его. Следующая гипотеза — подтверждать новый трек повторной
strong geometry, а не только суммой strong+weak hits; ещё не реализовано.
Не утверждать, что новая проверка полностью исправила детектор.

Повторы seed1 `series-20261002T182154Z` и182536Z дали иные состояния;
в первом guard mean speed0.121. Full capture сначала завершался после удаления
контейнера и терял stdout (пустой02-clouds.json). Теперь capture --stream
flushes JSONL; replay принимает JSON/JSONL. Offline GT — только labels.
Артефакты corner-clouds-stream-20261002.jsonl/json в private results dir.

### Слои MPPI

После сообщения пользователя, что фильтр штанг помог лишь немного:
удалён forced static_layer.map_topic=observed_grid из native_mppi.cpp.
StaticLayer снова читает исходную /map; штатный ObstacleLayer marking=true,
clearing=true получает filtered obstacle_scan. Memory8с остаётся глобальному
планировщику и diagnostic obstacle_grid. Real launch больше не выключает
cloud marking при наличии карты. Raw /map не модифицируется; footprint0.23м
и swept checks сохранены. Это исправление хранения/очистки, не замена
self-return фильтру. Без нового реального bag причина остаточного затыка
не установлена; пользователь сообщил, что bag не сохранился.

Real image7eb87f0bdde2912ca32c3520b99d6f16284b8c8227a024e781e3c0b6a699c146,
sim image1c9dbb740453160db647c10973075efa07cfa37a7e171a1913f54c5b6ddd17f2.
Изолированный новый live матч `series-20261002T184639Z`, seed0:
capture9.7сим-с, mean E/G0.222/0.286, wall0/0, small contacts0/0.
Detector p90 E/G0.048/0.034м,max0.055/0.037м. Это один короткий матч,
не подтверждение устойчивости/реальной езды и не доказательство эффективности
только смены слоя. Goals E0. Не сравнивать stochastic matches как точное A/B.

Stationary30с real ROS replay /first20 clouds172454:
static source/raw input — near lethal cells0 во всех56 costmap frames,
552 MPPI ok,31 swept rejection; static source/filtered — near cells0 во всех56,
565 ok,18 swept. Не движение робота, команды не outcome/скорость.
В memory/raw15с были4 costmap frames с1 near lethal cell и46 swept rejects,
но107 STALE_INPUT; разные boot windows/длительности не дают сравнивать rate.
Файлы costmap-*-repeat.json в evening-debug. В snapshot static map/TF
публиковались20Гц, что не соответствует real latched map. Исправлено на1Гц
для следующего final replay; прежние цифры не являются доказательством real
частоты detector diagnostics или CPU. Замеры filter performance не продолжать.

169 pytest passed,1 host skip; compileall/diff check. Real и sim images
собраны, private duel удалён runner, затем его init/daemon удалены вручную;
другие проекты не затронуты. Изменения этого этапа локальные, фильтр checkpoint
197fdb4 уже отправлен. Source changes в слое/детекторе отдельно не отправлены.

**Далее:** повторить real заезд с новым образом и сохранить bag; проверить
static/observed/native costmaps, свежесть входов и режим decision. Завершить
подтверждение трека против коробочных углов; повторить push yaw45 goal precision
(прежний closest0.099м всё ещё незакрыт). Проверить native очистку после удаления
коробки в отдельном navigation fixture и новые полноценные role outcomes.

Final replay с static map/TF1Гц: costmap-filtered-final.json —0 near lethal
cells во всех56 frames,568 MPPI ok/12 swept, но всего3 detector diagnostics.
Короткий повтор detector-health.json (15с) —40 detector diagnostics,
279 ok/9 swept,0 near cells во всех27 frames. Поэтому не выводить из одного
snapshot гарантированную частоту/готовность detector на hardware. Replay
не содержит AMCL, движения базы или новой записи после фильтра. Прежние
цифры подтверждают работоспособность цепочки и очистку входных клеток,
а реальные оставшиеся затыки без нового bag всё ещё не диагностированы.

## 2026-10-03 — новые реальные bag, C++ обработка, динамический слой (цикл незавершён)

Приоритет пользователя: рабочая обработка реального LiDAR/объектов в bag,
затем симуляция. Цель остаётся active; устойчивую езду не объявлять доказанной.
Текущее дерево не закоммичено; checkpoint 1c695e3, origin 197fdb4.

Вход read-only: hsl2026Extra/21-31-logs/2031-record. Из 10 записей три
содержательные: 181529 (101с), 182155 (45с), 182315 (120с). Остальные пустые
или короче секунды. Условия объектов не размечены пользователем.
Baseline средние активные скорости соответственно 0,019/0,041/0,037 м/с.
Raw LiDAR регулярный (~0,1с); filtered header age p95 ~0,61–0,64с;
navigation/self имеет паузы 7,51/9,82/5,47с. Нельзя объяснять все остановки
штангами. Близкие динамические lethal клетки в отклонённых траекториях
на 0,240–0,250м; статических совпадений нет. Артефакты audit/pipeline/costmap
в results/real-bags-2131-*.

Изменения:
- C++ hsl_lidar_filter вместо Python ROS filter; одинаковые маски/качество,
  сохранены поля/tag/endian/header. Array('B') исключает лишние проверки
  каждого байта в Python oracle/cloud transport. Python filter — offline.
- hsl_perception_cpp переносит circle/PCA/CV Kalman детектор; основной launch
  переключён, пересборка real/sim ещё идёт. Python node удалён; cloud.py
  оставлен для offline capture. Три strong hits подтверждают трек, weak
  поддерживают уже подтверждённый. MT19937 отличается от NumPy PCG64.
- planning.yaml общий sim/real: resolution0,10, radius0,23; мягкая inflation
  0,45/scaling8, critic repulsion1/critical10/margin0,05. Новые параметры
  не доказаны устойчивой ездой. Пользователь разрешил настройку запаса,
  но body radius≥0,178 обязателен.
- SemanticObstacleLayer наследует Nav2 ObstacleLayer; ignored_obstacles
  очищает ранее отмеченные узкие коробки только в динамическом слое.
- RealObservations сохраняет ограниченные очереди odom/cloud до TF на stamp;
  собственная маска привязана к базе на stamp облака. Freshness не ослаблена.
  observation_diagnostics добавлен в запись. Результаты под нагрузкой хуже,
  нельзя объявлять очереди универсальным исправлением без проверки.

Проверки:
- host pytest tests helm_launch/tests: 179 passed,1 skipped.
- check_lidar_cpp: byte parity 1023 real clouds + 2 synthetic endian/padding.
  Не измеряли время фильтра (пользователь запретил benchmark).
- check_cpp_detector: 5213 clouds, 685 свежих обнаружений у обоих вариантов,
  0 расхождений fresh/strong/foreground/clusters. Max center difference0,072м.
  Это fidelity, не precision/recall. Новые три bag не дают свежих треков;
  старые bag 151308/151617/151820/152152 дают треки. Без labels не утверждать,
  что каждый трек — робот, или что отсутствие трека — отсутствие робота.
- check_cpp_detector_ros:35 frames,33 outputs; visibility expires, no coast
  publications, map/child-frame contract passed.
- check_semantic_costmap: поздно распознанная узкая коробка удалена, широкая
  коробка и статическая стена сохранены (изолированный ROS fixture).
- replay_real_transport: network none, drivers false, motion never enabled,
  recorded map→odom не используется. После отключения законченного ручного
  мира filtered-only: own steady max gap0,101с,p950,080; scan max0,212,p950,117.
  Raw AMCL max own0,144,p950,080; scan0,169/0,114. Выигрыша raw не подтверждено;
  экспериментальный launch argument/CLI удалены, production filtered.
  Под параллельной симуляцией own gap7с: нагрузка важна, не выдавать A/B под
  разной нагрузкой за влияние фильтра.

Симуляционная серия (ещё Python detector, новая strong confirmation/planning):
`python3 benchmarks/run_duel_series.py --isolated-project hsl-boxes-2131
--ros-domain-id 83 --gazebo-port 11424 --runs 3 --start-seed 0 --active-s 90
--trace --audit-start`. results/isolated/hsl-boxes-2131/series-20261002T205623Z.

| Seed | Первый исход | Время sim | Скорость E/G | Contacts E/G |
|---|---|---:|---:|---:|
|0|guardian_capture|12,3с|0,275/0,264|1/0|
|1|timeout, оба застряли|90с|0,021/0,020|0/0|
|2|explorer_goal|17,4с|0,338/0,165|1/0|

Контакты нужно отдельно разобрать по wall/robot/small_box; не трактовать
колонку contacts как исключительно столкновения со стенами. Seed1: A* OK,
но native swept_collision (~94% времени) блокирует обе оптимизированные
траектории. Не завершать цель по двум удачным заездам. RTF первой части
серии искажён параллельным ручным миром, не сравнивать его как производительность.

Следующий шаг: завершить образы C++/актуальный source audit (cloud/node
cleanup произошёл во время сборки, может потребоваться повторная cached
сборка); прогнать реальные raw bags через установленный C++ стек, визуально
разобрать спорные robot/box кластеры и потерю наблюдений. Затем убрать
непрерывный swept rejection без отключения проверки контура и подтвердить
box ignore/goal/capture повторными фактическими заездами. Осталась прежняя
точность push yaw45 goal closest0,099м. Финальная 20×360с серия ещё не выполнена.

### Продолжение — реальная full-rate проверка важнее sampled parity

Full-rate bag 152423 выявил 111 различий свежего обнаружения и выбор другого
трека (до0,66м) при одинаковых strong counts. Причина: разные пары RANSAC
в смешанных кластерах (NumPy PCG64 vs C++ MT19937), влияющие на weak updates
и ассоциации. Унифицировали xorshift32/порядок пар в обоих core. Повтор
full-rate-parity-shared.json:2630 frames,1510 fresh,0 различий свежести,
strong/foreground/clusters, max position difference1,5e-15м. Новый parity
harness теперь падает при расхождении, а не только пишет отчёт. Нужен
последующий повтор ROS-адаптера/установленных образов после пересборки.

Визуальная проверка реальных кластеров (plot_real_clusters.py) выявила
геометрическую неоднозначность: длинная грань могла стать квадратом 15см
при повороте fitting rectangle. Ужесточили близость к граням/размеры;
неоднозначные размытые возвраты сохраняются. Добавлены проверки длинной
грани23см при разных yaw и blurred faces. Текущие tests184passed1skip.
Gallery — гипотезы по измеренным точкам, а не semantic labels; безусловное
правильное распознавание всех объектов не подтверждено. Последний shape
replay надо сверить с окончательным bound0,19/0,18 (часть артефактов снималась
до этого изменения). Сохраняется защита свежего rival от удаления как коробки.

Native MPPI после swept rejection теперь сбрасывает optimizer warm start:
команда не была выполнена, но оптимизатор уже сдвинул свою последовательность.
Контур/проверка всей траектории сохранены, новых ограничений скорости нет.
Это пока гипотеза против зависания seed1, ещё не подтверждённая заездом.

Raw sensor replay 181529,45с с C++ detector: первый запуск не поднял AMCL
из-за lost service response map_server/change_state (FastDDS); повтор
transport-native-181529-repeat:819 self/418scan/422filtered, steady own
maxgap0,142с,scan0,198с,finalcommands0. Первый провал не скрывать.
Production middleware — CycloneDDS; в network-none он без явных localhost
peers не обнаруживает другие процессы (нулевой replay, ожидаемо failed).
Transport harness теперь использует отдельный domain177 и явный unicast
loopback, проверяет ненулевые steady observations. Это настройка replay,
не изменение production DDS. Данные bag не изменяются.

### Native raw replay и окончательное правило подтверждения коробки

Production-образ real build7 готов; sim build6 готов. Python detector ROS node
удалён, ссылки capture/replay переведены на cloud helper/native executable.
Native covariance XY cross terms сохранены; невалидная карта отклоняется.
Host tests185passed1skip, diff --check/Compose sim+real passed.

Последние meaningful raw replay (Cyclone unicast, domain177, no drivers/no
permission/no recorded map→odom) с текущим C++ фильтром/детектором:

| Bag | Окно | self / scan / filtered | Max own gap | Max scan gap | Detector diagnostics |
|---|---:|---:|---:|---:|---:|
|181529|40с|501 /351 /379|0,387с|0,263с|210|
|182155|40с|664 /373 /379|0,149с|0,145с|365|
|182315|45с|882 /433 /442|0,120с|0,142с|433|
|151820 (старый, наблюдаемый робот)|60с|993 /582 /593|0,268с|0,206с|360, из них118 fresh|

В первых трёх записях свежих tracks нет, как и в offline replay. В 151820
всего144 tracked_robot clusters защищены от удаления; новых confirmed small
boxes нет, только3 tentative shape matches. Все final cmd_vel нулевые.
Часть повторов шла рядом со сборкой: это функциональная проверка транспорта,
не сравнение CPU/RTF и не доказательство physical driving. Локализация в
replay оценивается по readiness/свежести, не по независимой точности позы.

SmallBoxFilter теперь требует3 различных spatially consistent stamps≤0,6с
перед открытием восьмисекундной box identity. Одинаковый stamp не добавляет
подтверждения. Partial/cache продолжают только установленную identity.
Распознавание без stamp в offline геометрическом unit fixture мгновенное;
production всегда подаёт stamp. После ужесточения/consensus full-rate:
151820:0 ignored frames;152152:33frames315points;152423:120frames688points.
Это количество исключённых измерений, не recall коробок. Текущий positive
geometric profile ориентирован на вертикальные узкие коробки; лежащие/закрытые
и неоднозначные объекты остаются препятствиями, если форму нельзя подтвердить.

Уточнение simulation contacts старой серии: оба единичных контакта E (seed0/2)
— small_box; wall_collisions=robot_collisions=0 во всех трёх заездах. Seed1
застревание остаётся непроверенным после optimizer reset; ближайший debug run
должен воспроизвести именно его с новым C++ стеком. 152423 raw60с replay и
повтор semantic-layer fixture запущены; проверить результаты, не считать
запущенную команду подтверждением успеха.

152423 raw60с завершён:1027self/578scan/589filtered, max steady own0,230с,
scan0,161с;133detector diagnostics/76fresh;116ignoredframes1532points.
Semantic fixture final повтор прошёл: узкая dynamic mark очищена до стоимости30
(inflation от других объектов), wide/wall остались lethal254. Этот fixture
не подтверждает каждый real cluster как правильный объект.

Обнаружена лишняя проверка ROS detector: abs(latest own stamp–scan stamp)≤0,3
отбрасывала многие валидные сканы, хотя собственная поза не участвует в их
геометрической трансформации. Native adapter теперь требует fresh own≤1,2с
(как остальная навигация), scan≤0,5с и exact sensor TF на scan stamp. Штампы
не переписываются, visible≤0,3с сохраняется. RealObservations берёт newest
transformable cloud, superseded_scans — явный счётчик. Добавлен ROS fixture
с текущей own pose и чуть более старым valid scan. Пересборки8/7 запущены;
повторять реальный bag/fixture после их завершения. Геометрический core не
менялся, поэтому previous core parity не заменяет adapter verification.

### Последний подтверждённый повтор (build real8 / sim7)

Images: real3d13f0c8540f3823a52c45e932661d2e514a84ec2650a1c3987e2b219691cf9a;
sime218a1f7c10eed647dbd222f3228c4b76ede9ec2540e590a5c43b532da766797.
ROS stamp fixture:36diagnostics/34outputs, valid older scan with current own
processed, visibility expires, coast not published, stale own blocks processing.

152423 raw60с, newest cloud + TF-based own freshness:958self/557scan/591filtered;
550detector diagnostics,370fresh (до этого133/76). Max own0,255с,scan0,279с.
11ignoredframes156points,557protected clusters; уменьшение ignored count
связано с тем, что свежий track чаще защищает близкие к нему кластеры. Без
semantic labels не выдавать увеличение detections за доказанный рост recall.
Часть повтора шла рядом со сборкой, timing не CPU benchmark. Cmd_vel0.

Targeted seed1 regression:
```bash
python3 benchmarks/run_duel_series.py --isolated-project hsl-boxes-cpp-check --ros-domain-id 84 --gazebo-port 11425 --runs 1 --start-seed 1 --active-s 60 --trace --audit-start
```
series-20261002T215121Z:guardian_capture7,9simс, скоростиE/G0,304/0,303,
planner_ok0,949/0,924, wall/robot/small_box contacts0. Прежние0,021/0,020 и
94%swept rejection не повторились. Одновременно поменялись detector/queue/
box consensus/optimizer reset, не приписывать весь выигрыш только reset.
Прямого observed rival track в этом заезде нет (visible0 у обоих), guardian
шёл SEEK; поимка геометрическая, не доказательство качества преследования.
Runtime после заезда автоматически очищен. Финальная серия не проведена.

Следующий шаг: C++ sim visibility/profile на captured true peer clouds
(ground truth только для offline labels), реальные спорные robot/box clusters;
проверить другие seeds и push-yaw45 центр цели. Цель остаётся active.

## 03.10.2026 — приоритет raw bag и независимый аудит robot/box

Уточнение пользователя: основной критерий — реальные облака LiDAR и реальные
объекты в bag. Это закреплено в AGENTS. C++/Python parity и Gazebo не заменяют
проверку правильности распознавания на этих данных.

Наблюдаемый дефект оценки: production SmallBoxFilter защищает свежий opponent
track. Поэтому прежний box_hypothesis_overlap после защиты мог скрывать
конфликт классификаторов. Добавлен независимый SmallBoxFilter без защиты
трека только в offline replay_real_shapes; production алгоритмы не изменены.
Используется текущий full-rate core с shared RANSAC, не устаревшая сборка.

```bash
python3 benchmarks/replay_real_shapes.py results/real-bags-debug-20261002/full-rate --output results/real-bags-2131-shapes-independent
```

| Bag | Облака | Fresh tracks | Overlap после защиты | Независимый overlap |
|---|---:|---:|---:|---:|
|151820|761|476|0|2|
|152152|834|499|1|39|
|152423|1035|535|1|79|

Overlap означает расстояние track до подтверждённой геометрической гипотезы
коробки меньше0,25м. Это НЕ число ложных детекций: возможны коробка рядом с
роботом и ложная box hypothesis на частичном корпусе. Результаты/точки/стампы
сохранены в JSON. Следующий шаг — разбор соответствующих последовательностей
реального облака, включая окружение и движение объекта; не выдавать нулевой
старый overlap за отсутствие ложных tracks. Нельзя просто запрещать все
tracks рядом с box hypothesis: так можно потерять настоящего робота.
Проверки:39 detector/box/config tests passed, py_compile и diff --check прошли.

### Дополнительная проверка симуляции с текущими C++ наблюдениями

Серия results/isolated/hsl-cpp-vision/series-20261002T215801Z, seed1,
active limit60с, actual guardian_capture22,7симс. СкоростиE/G0,230/0,220м/с;
visible0,542/0,555; planner_ok0,736/0,890; wall contacts0/0, Esmall_box1.
Captured187облаков; offline replay production sim profile:75peer_near,
112no_detection,0other. Ошибка online track medianE/G0,022/0,018м,
p90 0,048/0,047м. Ground truth используется только для оценки. Это подтверждает
наличие tracks в данном сим-заезде, не реальное semantic качество.
Capture теперь доступен без обязательного добавления unknown obstacle;
завершается после match active→false. Runner tests11passed.

Контролируемый push fixture в свежем hsl-push-cpp/domain86/port11427:
решения/referee/opponent control остановлены, один movable15×15×40см,
yaw45°, box[1.2,0.5], target[2.4,0.5]. Goal reached за6,56симс,
closest0,0712м при допуске0,08; mean0,279м/с; box displaced1,471м.
Единственный contact pair — small box/own body. Параметры controller не меняли.
Это не автономная дуэль и не проверка настоящей механики коробки.
Полный JSON остался внутри adapter: копирование на host завершилось permission
error (results directory root-owned), затем runtime был удалён. Доступна только
сводка stdout tool run, подробные trace НЕ сохранены; не включать этот fixture
в финальную оценочную серию. Перед cleanup следующих fixtures обязательно
подтвердить успешное сохранение артефакта. Изолированный runtime закрыт.

Цель остаётся active: приоритет следующего цикла — спорные реальные объекты,
затем повторяемость движения/финиша и полная серия текущего дерева.

### Уточнение аудита: tentative не равно подтверждённая коробка

Предыдущий independent_box_overlap включал small_box_tentative из-за проверки
startswith. Исправлено: учитываются только small_box и cached/sparse established
identity. Полный independent_filter теперь сохраняется рядом с каждым cloud.
Повтор results/real-bags-2131-shapes-independent-confirmed:
151820:0 overlaps;152152:34;152423:69 (fresh476/499/535 соответственно).
Прежние2/39/79 superseded. Это по-прежнему конфликт гипотез, не false-positive
rate. Context plots в results/real-bags-2131-conflict-context показывают,
что в части спорных последовательностей форма меняется при движении; возможно
ошибочное распознавание box на частичном корпусе. Одной XY-близости недостаточно
для утверждения о ложном robot track.

Дополнительная offline гипотеза: strong_min_extent считать по нижнему rim,
а не всему объекту. Trial /tmp/hsl-rim-extent-trial.py, output
results/real-bags-2131-shapes-rim-trial. Fresh476/492/527 против476/499/535,
independent overlaps0/34/69 не улучшились. В production НЕ перенесено: теряются
измерения без устранения исследуемого конфликта. Следующая проверка — полнота
видимой высоты/граней у small-box кандидатов на реальных облаках.

Проверка высоты small-box положительных кластеров на текущих full-rate clouds
запущена как read-only host process (exec session17551, PID969801), на момент
передачи процесс жив и CPU active; stdout buffered, результата пока нет.
Он сравнивает p5(Z), долю точек ниже0,16м и число точек для shape-positive
кластеров вблизи текущего track (<0,25м) и остальных. Не перезапускать только
из-за отсутствия stdout; сначала проверить именно этот handle/PID. Ни нового
порогового фильтра по высоте, ни изменений production detector в этом цикле
не внесено. Автоматический отбор семантических labels не выполнялся.

## 03.10.2026 — проверка карты и планирования на 21-31-logs

Предыдущий read-only процесс session17551 завершился нормально. Высота не
отделила спорные фрагменты: low p5≤0,12м у1/2,13/14,25/26 shape-positive
кластеров рядом с track в151820/152152/152423. Этот порог НЕ добавляли.
Новый воспроизводимый инструмент audit_small_box_shapes.py сохраняет
кандидаты, stamp/chunk, высотные bins, доли нижнего/верхнего интервалов.
Первый запуск завершился TypeError np.bool при JSON serialization; исправлен
явным bool. Повтор session40633 жив на момент записи, результат не присваивать
до успешного сохранения vertical-support.json.

В SmallBoxFilter вычисление высоты перенесено перед 46-angle rectangle fit:
объекты top<0,25 или>0,46м не могут пройти ни новое распознавание, ни cached
правило, поэтому fit для них пропускается. Диагностические размеры в этом
случае отмечены dimensions_source=axis_bounds. Решение retain не меняется;
добавлен low_p5 для просмотра реальных кластеров.19box tests passed. Отдельных
замеров производительности фильтра не проводили.

replay_real_transport дополнен подпиской /map и проверкой верхних returns
z0,50..0,70м на занятые клетки с margin0,10м. Outside/unknown не засчитываются
совпадением. Это map consistency, НЕ независимая точность локализации.
Raw18152935с:605self/324scan/329filtered, max steady own0,120с,scan0,161с;
59alignment samples, weighted match0,9596, median1,0. Driver/motion off,
finalcmd_vel0. Местами match падает, глобального сдвига карты данным не видно.

Добавлен --planning-target X Y: только network-none; вместо decision/real_match
fixture публикует fixed GOAL и active, остальные узлы/конфиги — production.
Первый fixture ошибочно считал allowed=false достаточным запретом: MotionGate
проверяет active и intent, поэтому634MPPI/633final commands были ненулевые.
Драйверов/оборудования/сети в контейнере не было. Failed evidence сохранён
181529-planning-failed-gate.json, не считать тестом закрытого шлюза.
Исправленный fixture remaps только подписку motion_gate /match/active на
/probe/never_active и постоянно публикует тудаfalse; planner получает active.
Assertion artifacts теперь сохраняются ДО проверки, большие данные не печатать
в assertion. Реальный launch и правила шлюза не менялись.

Повтор181529-planning-locked.json,35с, target[0,5;3,5]:
585self/324scan/329filtered; max own0,122с,scan0,137с.
MPPI:634ok/20STALE_INPUT/1WAIT_OR_STOP/1swept_collision.
Global:162OK/33WAIT_OR_STOP/3STALE_INPUT;162nonempty paths/198.
Средняя абсолютная first MPPI command в ok cycles0,227м/с — НЕ измеренная
скорость езды.634nonzeroMPPI commands; finalcmd_vel0.
Wall weighted match0,9645. Map readiness/start/end входят в агрегат counts.
Recovery из-за неподвижной записанной позы не является физическим исходом.
Current Python planning/geometry loaded via /work source PYTHONPATH; C++ image
real8. Raw/odom/TF header ages сохранены, recorded map→odom не replayed.
Source real build9 запущен после теста (/tmp/hsl-real-build9.log,session81117);
проверить завершение, затем пересобрать sim для актуального source audit.

Следующий шаг — реальные коробки/частичные тела: проверки полноты двух граней
и вертикальной поверхности, без подгонки только под сокращение overlaps;
проба полного планирования на152423 с tracked opponent. Цель остаётся active.

Real build9 завершён успешно: a1fcb861b481348f9b1a587a9f6e25511b2cc14a5b5e206fdd1da4fb69796a3a.
Sim build8 жив, session32831/log /tmp/hsl-sim-build8.log.
Запущен15242360с planning-locked probe на установленном image real9, без
source PYTHONPATH override, target[0,5;3,5]; session подтверждать по текущему
handle/log /tmp/hsl-planning-probe-152423.log. Пока не присваивать исход.

## 03.10.2026 — актуальность входов планировщика на реальном cloud

Установленный real9:15242360с planning-locked probe (параллельно sim build8):
744self/494scan/591filtered; max gaps own0,357,scan0,372с;
MPPI426ok/586STALE_INPUT/156WAIT/2swept/1optimizer_failure.
В успешных global cycles median age own0,736с,intent0,919с, p90 0,999/0,994.
Входные сообщения regular, но planner drains depth10 queues после дорогих
callbacks. Hypothesis — текущие состояния нужно брать последние.

Изменение: hsl_planning subscriptions own/opponent/intent KEEP_LAST1 reliable,
scan KEEP_LAST1 BEST_EFFORT. Частоты/таймауты/штампы не изменены. Map/static
QoS сохраняется. Source override проверен на том же60с152423 replay:
1010self/578scan/591filtered; max gaps own0,167,scan0,184с.
MPPI1139ok/20WAIT/5swept, STALE_INPUT0. Global145RECOVERY_ROUTE/78OK/31WAIT/1STALE;
recovery здесь вызван recorded pose, которая не реагирует на MPPI команду.
Median valid global own/intent age0,089/0,085, p90 0,414/0,370с.
Все finalcommands0. Это functional current-data improvement, не driving speed.

Для устранения build-load confound baseline real9 повторён ПОСЛЕ sim build:
152423-planning-baseline-repeat.json:956ok/201STALE_INPUT/6WAIT/1swept;
median valid own/intent age0,474/0,625,p90 0,606/0,973;
input max gaps0,121/0,154/0,133с. Улучшение newest-state queues подтверждается
при уже завершённой сборке. Host tests185passed1skip, diff --check passed.

SmallBoxFilter дополнительно инвалидирует прежние box/pending identities
этого фрагмента, когда fresh observed robot защищает его. Иначе после потери
track cached small_box identity могла продолжать удалять часть корпуса8с.
Регрессия sparse fragment after track expires сохраняет препятствие;
истинная коробка после конфликта вновь требует3 shape-consistent scans.
20box tests passed. Новых порогов gap/размеров по audit counts не добавлено.

Vertical audit сохранён: vertical-support.json; positives4/30/57 по151820/
152152/152423. Upper gap share z0,25..0,34 median outside/near track:
1518200,449/0,331;1521520,347/0,286;1524230,360/0,214. Это hypotheses,
не object labels; частичные тела и маленькие коробки ещё надо разделить.
Низкий top/p5 порог проблему не устранил, в production не введён.

Real build10 жив session40036/log /tmp/hsl-real-build10.log. Sim build8 завершён,
image d1a2d871536663db8c4c5c4d15c67b77d8c05e3689fe229d864c104f009d472d,
но он ПРЕДШЕСТВУЕТ latest-state/cache invalidation; нужен sim build9.
Запущен full181529 raw probe до120с (bag может закончиться раньше), source
PYTHONPATH currentplanning/perception, log /tmp/hsl-planning-probe-181529-full.log.
Проверять полный meaningful bag, а не только ранние35–60с.
Цель active; hardware/driver не включались, push/commit не выполнялись.

Real build10 завершён:22564977f694204e5d94772dfc434274c924c437c4ae325d093386f34a608ce8
(latest-state queues + box cache invalidation). Sim build9 жив session86008,
log /tmp/hsl-sim-build9.log. Full181529 probe session23095 жив;
cache-invalidation full-rate replay session14842 жив,
log /tmp/hsl-shapes-cache-invalidation.log. Проверять именно эти handles;
отсутствие stdout до завершения не считать ошибкой. Ни реальный робот,
ни hardware drivers не запущены. Не объявлять цель завершённой по раннему
окну replay или ненулевой внутренней команде MPPI.

Full181529 probe завершён: bag metadata duration101,445с, requested window120с
охватывает все выбранные raw/odom/odomTF сообщения.1558self/917scan/993filtered;
max steady gaps0,450/0,582/0,172с, p95 0,131/0,167/0,113.
MPPI1912ok/57WAIT/19STALE/3swept; global467OK/37WAIT/1STALE.
Finalcmd_vel0. Часть прохода рядом с build10/sim9; это функциональный прогон,
не сравнение CPU. Длинные разрывы исходного собственного положения не
воспроизвелись. Report181529-full-latest.json сохранён, handle23095 terminal.

Full-rate cache invalidation replay тоже завершён (session14842 terminal):
fresh476/499/535 unchanged; ignored frames0/33/72 (до0/33/120), points0/315/316
(до0/315/688). Independent confirmed overlaps0/34/69 unchanged, потому что
independent branch не использует protected track. Production-like сохранение
корпуса после measured robot label стало строже; semantic accuracy без labels
по этим counts не объявлять. Результаты — real-bags-2131-shapes-cache-invalidation.
Открыто: подтвердить две видимые грани/полноту поверхности для новых box births,
full182315 transport/planning и симуляция после завершения sim build9.

## 03.10.2026 — полный182315 и форма коробки между сканами

Full182315 metadata119,973с, replay requested150с покрывает выбранные сообщения
полностью. Installedreal10, no source override, fixed probe target[0,5;3,5],
network-none/no drivers, locked finalgate.2256self/1181scan/1193filtered;
max steady own0,175с,scan0,195с,filtered0,123с.
MPPI2365ok/8swept/1WAIT, глобальный595OK/29WAIT; STALE_INPUT0 обеих ступеней.
Finalcmd_vel0. Полные181529 и182315 не воспроизвели прежние разрывы own/TF на
5–10с. Это транспорт/compute replay; actual velocity/collisions так не измерять.

Face audit: inspect_box_faces.py на91 shape-positive реальных кластерах;
две adjacent грани с≥4bins по2см и span≥8см поддержаны1/49 outside-track
и4/42 near-track гипотезами. В production такое жёсткое требование НЕ введено:
оно отсекает почти все sparse single-face views и не отделяет тело достаточно.
Результат face-support.json — evidence/diagnostics, не semantic labels.

Вместо этого SmallBoxFilter подтверждает не только3 distinct stamps/близость
центра, но и combined shape всех подходящих сканов≤0,6с (не больше7 облаков).
Если объединённый фрагмент уже не похож на15×15×40см коробку, новая identity
не открывается. Old inconsistent evidence expires; истинный установившийся
box can reconfirm. Уже опознанную pushable коробку можно продолжать двигать
по прежнему spatial cache. Same stamp не добавляет view. Свежий robot label
инвалидирует box/pending как в предыдущем цикле.22box tests passed.

Full-rate2630clouds: fresh476/499/535 unchanged;
production-like ignored frames0/0/69,points0/0/230
(до0/33/72 и0/315/316); independent confirmed overlap0/0/9 (до0/34/69).
Это не доказательство false-positive rate: отсутствие labels не позволяет
объявить каждый оставшийся или устранённый конфликт реальной ошибкой.
Сохранилось игнорирование согласованной формы в152423; не обнуляли политику
всех коробок. Нужно подтвердить physical push после этого изменения.
9оставшихся nearby conflicts относятся к partial moving-body сценам/соседним
объектам; measured track protection сохраняет тело. Untracked ambiguous
fragments ещё открытый вопрос. Результаты real-bags-2131-shapes-combined.

Sim build9 завершён d1d7737462b2b0d46eb78961c0b9bd485638aa7a8d4fa15155adaa18799bd8a6,
но предшествует combined shape patch. Real build11 жив/log /tmp/hsl-real-build11.log.
После него buildsim10, physical push yaw45 и debugduel текущего дерева;
полная оценочная серия всё ещё не выполнена. Цель active, commit/push не делали.

Real11 build завершён:eab68eeb421ca4cf518880d314da6d443822d3e5d2ecb8dde11c088167fa7079.
Host tests188passed1skip, diff --check passed. Sim10 ещё жив session59300,
log /tmp/hsl-sim-build10.log. Next physical test ждёт именно его завершения.
Full152423 installedreal11 replay завершён (bag104,964с,requested120с):
1644self/1013scan/1040filtered; max steady gaps0,290/0,318/0,136с;
1834nonzeroMPPI commands, finalcmd_vel0. Current combined-shape code выполнен
реальным ROS planner на полном raw bag, не только NumPy fixture. Driver/hardware
off, часть прохода рядом со simbuild10. Full report152423-combined-full.json.

### Окно подтверждения при реальной частоте обработанных сканов

В полномROS152423 processed scan stamp gaps median0,20,p90 0,60,p99 0,899с,
max1,20с. Жёсткое0,6с часто не оставляло3доступных скана. Window теперь1,2с,
внешний global.small_box_confirmation_window в planning.yaml (0,2..scan_timeout),
не больше7view samples; объединённая форма остаётся обязательной. Свежесть
pose/scan не ослаблена, cached identity/lifetime8с не изменены. Регрессия
подтверждает истинную stablebox при timestamps0/0,4/0,8с; moving inconsistent
fragments не устанавливают identity.32box/config tests passed.

Offline full-rate при1,2с: fresh476/499/535, ignored0/0/69 и0/0/230points;
independent overlaps0/0/9 — как при0,6с в этом archive. Наравно много полезных
views не обещать только увеличением window. ROS60с probe sourceoverride:
857self/562scan/584filtered, max0,282/0,267/0,143с;
965ok/202WAIT/4swept, STALE_INPUT0, finalcmd0;
146filter frames,0ignored,4tentative (это первые60с, не весь104с bag).
Нулевое число игнорированных точек не объявлять precision/recall или успехом
распознавания. Production-like offline сохраняет игнорирование в поздних сценах.

Real12 build завершён74181def1002a0112794b8183e4af3f2489c3f544393cff3ecfe97ed297948ce.
Sim11 жив session24115/log /tmp/hsl-sim-build11.log, соответствует1,2сwindow;
сим10 предшествует этому изменению. Нет действующих Gazebo миров/драйверов.
Следующее действие после подтверждённого build11 — physical push yaw45 с
сохранением/проверкой host artifact ДО cleanup, затем debugduel с boxes.
Цель active; untracked partial body/box ambiguity остаётся ограничением.

Sim11 завершён670add91cd86eee0fe5221d17fe493d80088ffd094a4b79867261744655a654e.
Подготовка isolated physical fixture hsl-push-final/domain89/port11429 живёт
session60675, log /tmp/hsl-push-final-prepare.log, motion closed. Не перезапускать
без проверки именно этого handle. После готовности stop только его decision,
opponentdecision/referee/opponentcontrol; push_box_trial.py yaw45 output/tmp,
сначала dockercp host/tmp и проверитьJSON, потом сохранитьuser-ownedresults и
только после этого cleanup. Manual worlds/реальное оборудование не трогать.

### Physical push текущего combined/window1,2 дерева

hsl-push-final prepare passed source/params audit, movement never permitted by
runner. Controlled fixture после остановки только его decision/referee/
opponentcontrol; isolated domain89/port11429, no other worlds. Cube15×15×40см,
yaw45, mass0,1kg/friction0,3, box[1,2;0,5], own initial[0,5;0,5], target[2,4;0,5].
Goal reached6,66simс; closest0,0590м (<0,08); distance1,844м; mean0,2769м/с.
Box displacement1,474м; finalz0,20 (upright).170ignored filter reports.
Contact pairs только cube/own body; стен/другого робота нет. Физическая коллизия
не удалена: объект действительно перемещён. Ни rival/world truth ни boxes pose
не поданы в классификатор/планировщик; Gzmodel pose — только оценка.

Полный JSON скопирован изadapter/tmp вhost/tmp, проверены samples/filter/mppi,
затем сохранён user-owned results/push-combined-yaw45-verified.json ПЕРЕД cleanup.
Report включает traces/geometry/MPPI; artifact loss предыдущего fixture не
повторился. Match allowed/active closed finally. Ownedworld cleanup запущен
(/tmp/hsl-push-final-clean.log), затем только debugduel текущей сборки.
Это controlled fixture, не autonomous duel и не доказательство real mechanics.
Цель active: требуется повторная дуэль и разбор оставшихся untracked ambiguities.

Ownedpush world cleanup confirmed terminal. Запущена debugseries3seeds0..2,
isolatedprojecthsl-combined-duel/domain90/port11430,90simс each, trace/startaudit/
detectorcapture. Log/tmp/hsl-combined-duel.log. Это не финальные20×360с.
Сохранить результаты двух ролей и затем разбирать слабую роль/застревания;
не считать сам запуск серии подтверждённым исходом.

### Результаты debugduel и приоритет реального bag

Серия `results/isolated/hsl-combined-duel/series-20261002T233102Z` завершена:
3 успешных запуска, seed0..2, 90с limit, version1c695e3+dirty.b50e9726245b.
Seed0 timeout90с, seed1/2 explorer_goal21,0/21,2с. Средняя скорость E/G:
0,240/0,027; 0,291/0,111; 0,291/0,111м/с. Стен/robot collision0 во всех,
контакты исследователя с pushable boxes6/3/3. Поимок0. Guardian NO_LOCAL_PATH
59,2/18,6/14,6% времени; seed0 также много OK с нулевой командой.
Это выявленный дефект, а не подтверждение готовности двух ролей. Средняя
скорость стража ниже требуемой; final20×360с серия ещё не выполнена.

Пользователь повторно подтвердил основной приоритет: реальный raw LiDAR и
объекты из bag. Запущен полный152423 (104,964с, replay120с), installedreal12,
network none, drivers false, locked final motion gate, fixed planning probe
target[0,5;3,5]. `/tmp/hsl-real-current-full.log`, output
`results/real-bags-2131-registration/152423-current-full.json`.
Результаты прежних build11/sourceoverride не подменяют эту проверку текущего
установленного стека. Число tracks/ignored points не считать precision/recall.

### Полные real12 replay152423 и182155 — результаты

Оба replay завершены без исключений; контейнеры `--rm`, драйверы выключены,
network none, finalcmd_vel0. Цель probe фиксирована, это не автономная езда.
Installed image74181def1002a0112794b8183e4af3f2489c3f544393cff3ecfe97ed297948ce.

| Bag | self/scan/filtered | max steady gaps, с | MPPI results |
| --- | --- | --- | --- |
| 152423,104,964с | 1900/1026/1040 | 0,153/0,160/0,143 | 2046ok/19WAIT/5swept, STALE0 |
| 182155,45,256с датчиков | 782/426/432 | 0,168/0,147/0,126 | 854ok/7WAIT/3STALE/3swept |

152423 detector1026diagnostics/503detected, filter469frames/0ignored.
182155 detector426diagnostics/0detected, filter354frames/0ignored.
Отсутствие detected без object labels не доказывает отсутствие соперника или
правильность детектора. Отсутствие ignored не доказывает готовность коробок.

Разобранное расхождение152423: в районе[2,36;0,22] индивидуальные сканы проходят
15cm shape test, но объединённые81/104/131points дают major extent0,194/0,198/
0,197м при лимите0,190м. Поэтому `small_box_inconsistent_views`, box identity
не создаётся. Возможны шум привязки AMCL/реальная форма или смешанный объект;
семантика этого фрагмента не установлена. Не расширяли порог ради count:
это могло вернуть ошибочное исключение фрагментов корпуса. Offline replay с
записанной привязкой и новый AMCL не являются одинаковым геометрическим входом.
Следующий шаг: проверить этот фрагмент raw cloud в локальной системе и
устойчивость межскановой привязки; отделить геометрический шум от другого
объекта перед изменением допуска объединённой формы.

Replay harness дополнен `sensor_replay`: spans, counts per topic, full-window
coverage.182155: все959TF/959odom/432LiDAR поданы, окно45,256с полное.
Первый152423 загрузил скрипт до добавления полей; покрытие для него опирается
на request120с>bag104,964с и завершение прохода, не на новые поля.
py_compile и diff --check passed. Документация диагностики обновлена.
Reports `results/real-bags-2131-registration/*-current-full.json` и ROS logs.
Commit/push не делали; цель остаётся активной, коробки/guardian stalls открыты.

## 03.10.2026 — полный обзор опровергает узкий фрагмент

Предыдущий goal turn — progress: два завершённых full raw replay, новое
coverage harness и установленная причина shape rejection. Продолжаем на
реальных облаках, цель не завершена.

ROI152423 около[2,36;0,22] на recordedTF при0/58/60/62/64/87с: полноценные
foreground clusters major0,238/0,232/0,291/0,263/0,246/0,272м; нельзя считать
все узкие фрагменты здесь истинной15cm коробкой. Последний3D обзор196points
с широким контуром. `results/real-box-registration/152423-static-region.*`.
Это geometry evidence, не human semantic labels. Подгонку maxwidth0,19→0,21
не вводили. Исходная гипотеза «только AMCL шум мешает коробке» недостаточна.

Обнаружен дефект кэша: полный broad/low/tall view сохранял старую box identity,
следующий sparse view мог снова удаляться8с. Теперь retain с height<0,25 или
>0,46 либо major>0,25 отменяет box/pending, если старый fragment center близок
к текущему center или лежит в XY bounds полного кластера с запасом0,025м.
Инвалидирование только отменяет ignore; не удаляет точки/стены. Diagnostics
`invalidated_box_hypotheses`. Новые регрессии для трёх типов полного объекта
и отмены pending;36box/config tests passed.

Full-rate2630clouds с текущим исправлением: fresh476/499/535 unchanged,
production-like ignoredframes0/0/1,points0/0/25 (до0/0/69,0/0/230),
independent overlaps0/0/2 (до0/0/9). Снижениеcounts само по себе не semantic
успех; требуется доказать узнавание настоящей маленькой коробки на raw bag.
Reports `real-bags-2131-shapes-fuller-view`. Разметки нет, задача коробок открыта.

Отдельная offline-only проба cell0,10 вместо0,08 дала ignored0/23/3frames и
independent overlaps0/5/5. Не перенесена в runtime: неоднозначность растёт в
152152. Её скрипт загружен до fuller-view patch, это отдельная версия trial.
Reports `real-bags-2131-shapes-cluster10`. Исправлен harness: отсутствующая
директория/пустой наборJSON+NPZ теперь ошибка, а не пустой успешный отчёт.
Первые два вызова с неверным input ничего не оценили; сохранённые пустые
results затем заменены настоящим2630clouds replay, не считать их проверкой.

Запущен real build13, `/tmp/hsl-real-build13.log`; после завершения проверить
новый installed stack полным raw152423, не подменять результат sourceoverride.
Sim11 пока предшествует fuller-view patch. Commit/push не делали.

### Installed real13 и потеря граней при прореживании

Real13 build завершён705d89de85158788e47237d438d5a466cc44764c1da308a29bf8d54b9eecb243.
Полный152423 на installedreal13: все2196odom/2196TF/1040raw LiDAR за104,925с
датчиков поданы; 1916self/1027scan/1040filtered, max gaps0,264/0,289/0,138с;
2059ok/6swept/4STALE/1WAIT MPPI.432filter diagnostics/0ignored, finalcmd0.
Fuller-view invalidation не объявляет этот объект маленькой коробкой.

Добавлен offline audit `--cloud-points 0` (все измеренныеXYZ) для сравнения с
прежним6000 target на одинаковых stamps, без замера времени фильтра.
181529 из21-31-logs,period0,3с, current self-filter/recordedTF,276frames:
полныйcloud дал28ignoredframes/888points; sparse6000 дал0ignored/0points.
Устойчивая гипотеза около[2,82;0,88]: major0,12–0,14м, top0,38–0,40м,
вертикальная грань; полноценная box identity после3distinctstamps.
Это геометрическое свидетельство, не human label.3Dgallery и JSONcomparison
вresults/real-bags-2131-dense-gallery и real-box-registration/181529-density-comparison.json.
Две реконструкции results/real-bags-2131-dense и real-bags-2131-same-sparse.

Исправление runtime: RealObservations map_xyz векторно преобразует всеXYZ,
сохранён прежний self mask0,25м на exact scan-timebase TF; планировщик
cloud_xyz читает полныйscan вместоread_xyz5000. Фильтр raw/AMCL не менялся.
Детектор остаётся C++. Новые тесты полного20003points transport/scalar
quaternion parity и сохранения ближних допустимых точек.195passed1skip tests.

Первая sourceoverride ROS попытка остановилась с NameError transform:
при изменении imports забыли скалярное преобразование odom. Собственный
network-none контейнер остановлен, отсутствие подтверждено; import восстановлен,
failed ROS log сохранён, затем запущен новый replay. Это не успешная проверка.
Harness теперь прерывает replay при завершении bringup, не ждёт окно на мёртвых узлах.

Повторный полный181529 sourceoverride завершён: все2106odom/2105TF/993LiDAR
за101,426с поданы;1957self/988scan/993filtered, max gaps0,169/0,238/0,131с;
MPPI1970ok/12WAIT/7STALE/4swept; finalcmd0.602filter frames,54ignored/1888points,
тот же узкий объект[2,82;0,88] подтверждается объединённой формой86points.
988detector diagnostics, detected0; без labels это не утверждение об отсутствии
соперника. В ROS установлена работа фильтра коробок на реальных measured data,
не только NumPy fixture. Не навигационный closed-loop успех.
Report181529-dense-source-full.json, image13+C++ /workPython override.

Запущен real14 build для установки full-cloud transport, `/tmp/hsl-real-build14.log`.
Следующий шаг после сборки: full151820/152423 на installedimage для проверки
детектора с плотным входом и ложного игнорирования тела; затем sim rebuild,
push regression и guardianstall. Commit/push не делали, цель active.

### Installed real14: плотный вход на двух полных сценах

Real14 b57a44083866b5642530c94e8c0facd87b40be05206657295af31363b335c873.
Без Python override, driversfalse/networknone, lockedcmd0:

| Bag | Полное окно датчиков | self/scan/filtered | max gaps, с | detector fresh | filter ignored |
| --- | --- | --- | --- | --- | --- |
| 151820 | 77,546с;1618TF/1618odom/768LiDAR | 1523/755/768 | 0,101/0,140/0,127 | 504/755 | 0/367frames |
| 152423 | 104,925с;2196TF/2196odom/1040LiDAR | 2039/1027/1040 | 0,114/0,192/0,142 | 582/1027 | 0/311frames |

MPPI1518201497ok/27swept/2WAIT;1524231833ok/10swept/220WAIT, STALE0 обоих.
Track protection257/184clusters. В этих сценах плотный вход не исключал
подтверждённые коробки вообще; это не доказательство semantic precision/recall.
Поведение объекта около[2,36;0,22] осталось retain/tentative/inconsistent;
полный вид не подогнан под коробку. Actualcmd0, reports*-dense-installed-full.json.

Трасса стража seed0:1058swept rejects,952(90,0%) после30-го шага из60,
median rejected index52. То есть большинство отказов далеко по горизонту,
а не непосредственно в начале команды. Предыдущую формулировку «почти
в начале» считать неверной для этой серии. Длина прогноза теперь внешний
integer local.MPPI.time_steps вplanning.yaml, default60/3с, validation20..120.
13configuration tests passed. Пока default не сокращён, safety swept-check
по-прежнему проверяет весь горизонт. Планируется A/B60vs30 на одинаковом
новом sim image/семантике, не обход hard collision check.

Чтобы оба образа понимали новый внешний integer, запущены real15 иsim12.
Первый simbuild вызов ошибся service duel-adapter и ничего не собирал;
исправленный VEHICLE_ID=simulator/profileduel/buildhsl-adapter жив,
log/tmp/hsl-sim-build12-retry.log. Real15 log/tmp/hsl-real-build15.log.
Не запускать новый start_real по образу14 с time_steps YAML до пересборки:
его установленный loader предшествует этому ключу. Commit/push не делали.

Real15 завершён9943da06dd4124849a4fc5b2675ec5b4cb3c3a62e22cef2c10d20c86ac610445.
Sim12 завершёнf7bce2171aa89fa4ca1d66854248589d8cb682a17000b318a2b7e376b651eb1d.
Оба установленно понимаютMPPI.time_steps60. Full181529 на installedreal15
(без override) подал все2106odom/2105TF/993LiDAR за101,426с.1779self/978scan/
993filtered,max gaps0,263/0,355/0,222с;MPPI1639ok/343WAIT/4STALE, swept0;
276filter diagnostics,14ignoredframes/540points, fresh detector0.
Цельprobe фиксирована, finalcmd0, no drivers/network. Simbuild12 частично
перекрывался по CPU; сравнивать54vs14как semantic degradation нельзя.
Report181529-dense-installed-full.json, clockoffset записан для совмещения
output diagnostics с raw bag stamps. Настоящее installed подтверждение
игнорирования устойчивой узкой формы в real data получено, не только oracle.
Точно какой предмет был в этой точке, без разметки не утверждать.

Теперь оба образа актуальны. Запущена baseline90с seed0 с60steps на новом
sim12: isolatedhsl-dense-horizon60/domain91/port11431, trace/startaudit/
detectorcapture. Log/tmp/hsl-dense-horizon60.log. После terminal cleanup
сравнить30steps на том же image/code/seed и сохранить обоих role traces.
Default60 пока сохранён. Commit/push не делали, цель active.

Baseline60steps sim12 завершён (series-20261003T041958Z): seed0 explorer_goal
18,5с, E/G mean0,317/0,209м/с, plannerOK97,3/90,3%, NO_LOCAL_PATH0,5/7,0%,
wall/robot collisions0, E smallboxcontacts1, G0; RTF0,394. Guardiancapture0.
Cleanup ownedworld подтверждён. Старый seed0 stall90с на предыдущем дереве
не повторился в этом shortfuller-view/fullscan run; не приписывать это горизонту,
потому что он всё ещё60. Кодversion1c695e3+dirty.253c645fd8cb.

Теперь единственное trial parameter изменение local.MPPI.time_steps60→30
во внешнем planning.yaml. Запущен одинаковый seed0/90с на том же sim12,
isolatedhsl-dense-horizon30/domain92/port11432,trace/audit/detectorcapture.
Log/tmp/hsl-dense-horizon30.log. YAML30 пока эксперимент, не объявлен выбором
по результатам. Полный1,5с прогноз проверяется hard swept contour.

### Сравнение горизонта: сокращение не принято

Trial30steps завершён (series-20261003T042319Z): seed0 explorer_goal30,0с,
E/G mean0,261/0,137м/с, NO_LOCAL_PATH4,3/7,0%, guardianRECOVERY_MPPI13,7%,
wall/robot collisions0, Esmallboxcontacts1. Против60steps: goal18,5с,
E/G0,317/0,209м/с. Mean lateral global E0,025→0,006м, G0,070→0,080м;
linear accel RMS E0,336→0,476,G0,402→0,382м/с²; angular E0,693→0,534,
G1,567→1,614rad/s². Лучшее отклонение E не компенсирует скорость/guardian
recovery. Short horizon гипотеза не подтверждена этим сравнением, YAML30
возвращён60. External knob сохранён, full swept safety не менялся.
Одинseed не даёт статистического доказательства,70%+failures старого дерева
не сравнивать с этим trial как смену толькоtime_steps.

Owned30world cleanup terminal. Запущены seed1..2/90с/default60 на том же sim12,
isolatedhsl-dense-repeat60/domain93/port11433, trace/audit/detectorcapture;
log/tmp/hsl-dense-repeat60.log. Это повторная отладка, не final20×360с.
Следующий шаг: результаты обеих ролей, stalled trajectories/costmap источник
если повторится остановка; physicalpush regression новогоfuller-view дерева
остаётся невыполненной. Не объявлять весь goal завершённым. Commit/push не делали.

По запросу статуса проверено текущее состояние repeat60: index содержит
законченный seed1 explorer_goal19,2с, E/G0,303/0,132м/с. Второй заездseed2
ещё выполняется runner session34513; контейнеры живы, процесс не перезапускать
из-за незавершённого stdout/старых файловtrace. СкоростьG снова ниже0,2,
поэтому исправление остановок/уверенной езды обеих ролей не подтверждено.
Цель active, не объявлять рабочий стек полностью готовым.

## 03.10.2026 — актуализация текста текущей цели

Обновление PROJECT_GOAL и DUEL_IMPROVEMENT_PROMPT отменено по последующему
запросу пользователя. Оба документа возвращены в состояние до этого
обновления, более ранние изменения PROJECT_GOAL сохранены.
Полная20×360 серия остаётся невыполненной. Новых заездов в этом цикле нет.

Repeat60 завершён: series-20261003T042924Z, seed1 explorer_goal19,2с,
E/G0,303/0,132м/с; seed2 guardian_capture23,9с,E/G0,266/0,218м/с.
Wall/robot collisions0 в обоих; малые контактыE1/2, G0/0.
Предыдущая запись «seed2 выполняется» относится к моменту её написания.
Runner завершил очистку собственного мира. Следующий технический шаг —
установить источник запрещающих клеток у стражаseed1, а затем повторить
дуэли и контролируемое толкание на текущем дереве. Цель остаётся active.

### Диагностика источника остановки стража

Hypothesis: seed1 contour reject связан с динамическими отметками, а не
непосредственной стеной. По SDF расстояние собственной позиции[2,2913;1,7614]
до ближайшей физической стены0,2971м; до центра занятой клетки статической
карты0,3174м, при защищённом радиусе0,23м. Это ещё не доказывает источник
Nav2 отказа: нужны реальные клетки rolling costmap.
В native_mppi добавлено diagnostic rejected_cells с costs слоёв вдоль
тех же Bresenham edges контура; максимум8клеток, try-lock без ожидания
layer mutex под master lock. Решение swept-check не меняется.
Sim13 build session11730,/tmp/hsl-sim-build13.log: hsl_nav2_control собран,
общая сборка ещё выполняется. После terminal повторить seed1 с trace.
Текст цели/промпта по запросу пользователя не обновлять.

Sim13b2280c21dd9aa6ea5bbba8357200407c68f8701196075a4787d5a0a978d47f5a
собран; full host tests199passed/1skip. Заезд diagnostic seed1 завершён:
results/isolated/hsl-cost-source/series-20261003T071648Z, Egoal17,6с,
E/G0,330/0,138м/с. Из20 trace samples с blocked cells встречаются стены
и dynamic-only клетки: например[2,325;1,275] в0,355м от SDF стены;
[2,375;1,775] в0,369м. Наличие динамического cost не доказывает старый след:
нужно сопоставление с текущими сканами. Safety не менялась.
В конце G track[2,5347;3,7883] при peer own[0,5758;3,4773]: существенная
ошибка detector, рядом с малой коробкой2. Без правильного трека
средняя скорость сама по себе не означает рабочую поимку.
Offline replay181 samples/.2s дал0fresh и сbaseline, и сmin_extent.25:
не воспроизвёл native track, параметр не принят по этому слабому тесту.
Сохранённые исходные clouds прорежены по времени, RANSAC/tracker историю
это меняет. Capture теперь default every distinct cloud (sample-period0),
replay harness получил --min-extent для проверяемого сравнения.
Запущен новый fullrate diagnostic seed1, isolatedhsl-cost-fullrate/domain95/
port11435/session8747,log/tmp/hsl-cost-fullrate.log. Runtime алгоритмы
пока те же; изменён только диагностический capture/trace. Цель не обновлять.

### Ограничение бесконечного сопровождения по weak фрагментам — эксперимент

Fullratecapture завершён: hsl-cost-fullrate/series-20261003T072332Z,
seed1 explorer_goal19,0с,E/G0,306/0,202м/с; сохранены462clouds обеих ролей.
Offline baseline на них:18peer_near/15other/429none. Strong_min_extent.25
дал те же числа: новую настройку размера не принимаем. Ложный track
обновляется partial body weak после надёжных детекций, не рождается
как новый маленький strong объект.

Изменение Python oracle и production C++: хранить last_strong_update;
coast lifetime(identity) отсчитывать от последней strong детекции. Weak
может поддерживать краткий partial view, но не возобновляет identity
навсегда. Для confirmed используется existingmax_coast1,5с, tentative0,35с.
Это эксперимент, пока не подтверждённый автономной серией/semantic real labels.
Риск: более ранняя потеря реального робота при долгой частичной видимости.
На тех же462clouds:14peer_near/0other/448none. Потеря4peer_near — уходящий
частичный вид (39→9точек рядом с peer), её не скрывать успешным removal15other.
Добавлены регрессии: confirmed+weak не живёт бесконечно, stationarystrong
продолжает подтверждаться;13shape tests passed.

Команды:
```bash
OPENBLAS_NUM_THREADS=1 python3 benchmarks/replay_detector_clouds.py results/isolated/hsl-cost-fullrate/series-20261003T072332Z/00-detector-clouds.jsonl --output /tmp/hsl-fullrate-weaklimit.json
g++ -std=c++17 -O2 -I/usr/include/eigen3 -Isrc/hsl_perception_cpp/include src/hsl_perception_cpp/src/replay.cpp -o /tmp/hsl_detector_weaklimit
OPENBLAS_NUM_THREADS=1 python3 benchmarks/check_cpp_detector.py --binary /tmp/hsl_detector_weaklimit --input results/real-bags-debug-20261002/full-rate --output /tmp/hsl-detector-weaklimit-parity.json
```
На2630real clouds Python/C++ совпали: fresh335/228/397, strong/foreground/
clusters/fresh disagreements0, maxpositiondiff1,35e-15м. Counts стали ниже
прежних476/499/535 — это не доказательство повышения semantic accuracy,
необходимо проверить потерю валидных tracks. Запущены sim14build22324
и real16build97109,logs/tmp/hsl-sim-build14.log,/tmp/hsl-real-build16.log.
Далее установленный real replay и сопоставимые дуэли; радиус пока0,23м.

### Остановка работы по запросу пользователя

Пользователь: «Давай заканчивать». Серия hsl-weaklease-duel остановлена
SIGINT runnerPID2488941; handle60273 terminal130, собственный duel очищен
finally. Заезд не завершён и не считается успехом/неудачей алгоритма.
Sim14 image2a586001b32e40defd2a6687e9ffdf54c53ad05a56216199cb94eb8cee723004;
real16cb9c891a043cdeef9f5bd082a31354b98db06c1ab5ae2c2f0fea432e08140ecd.
Обе сборки terminal успешные. Радиус сохранён0,23м; уменьшающий trial не начат.

Полный installedreal16 replay151820 завершён:
results/real-bags-2131-registration/151820-weaklease-installed-full.json.
Все1618TF/1618odom/768LiDAR поданы за77,546с; received1335self/719scan/761filtered,
max steady gaps0,346/0,303/0,210с. Detector282fresh/689diag; малые коробки
0ignoredframes/235diag. MPPI1052ok/316WAIT/62swept/62STALE/15NO_GLOBAL/5optimizer.
Finalcmd0,networknone,driversfalse, fixedprobe. Часть replay совпала со сборкой
sim14, поэтому performance/count отличия от прошлой installed151820 не
выдавать за semantic улучшение или регрессию. Точность треков без разметки
не доказана, current autonomous weaklimit дуэль не завершена.

Честный итог: страж не исправлен устойчиво; есть diagnostic evidence динамических
и статических blocking cells, но источник динамических ghost ещё не устранён.
Weak lease на462simclouds убрала15other, потеряла4peer_near; требует runtime
проверки, а не объявления detector готовым. Узкая форма наreal181529
достоверно исключалась installedreal15; все реальные типы/ориентации и
current controlledpush не подтверждены. Дополнительный замеченный риск:
on_map вызывает statefulSmallBoxFilter на static/empty map_points с новым
stamp и может влиять на историю scan гипотез; исправление не сделано.
Следующий шаг при возобновлении: законченная sim14 дуэль, анализ weak loss
на размеченных реальных сценах, затем очистка старых динамических клеток/
согласование A*/MPPI и контролируемое толкание. Коммит/пуш не выполнялись.
Текст цели/промпта оставлен без нового обновления по просьбе пользователя.

### 03.10.2026 — отдельный запрос сравнения производительности detector

Общая цель остаётся paused. По запросу пользователя измерены Python/C++
детекторы на одном real bag151820; LiDAR-фильтр не измерялся.
Из768raw clouds768 прочитаны,7 начальных не имеют recordedmap TF,
761 подготовленное облако по18825–20240точек (среднее19505), без прореживания.
Current filtered XYZ/TF общие, current real detector profile/weaklease одинаковые.
Хост i7-10510U, affinity одно ядро, BLAS1;5последовательных повторов с
чередованием порядка и полным прогревом каждого процесса. C++Release-O3/NDEBUG.

| Метрика Detector.step | Python | C++ |
| --- | --- | --- |
| mean, мс | 4,188 | 0,268 |
| median, мс | 4,060 | 0,259 |
| p95, мс | 5,609 | 0,355 |
| p99, мс | 7,222 | 0,491 |
| max, мс | 10,416 | 0,822 |

Mean speedup15,62× для ядра, не всегоROSстека. Во всех5повторах325fresh
обоих, foreground/clusters/strong/fresh/output presence disagreements0,
maxXYdifference4,97e-16м. Это совпадение алгоритмов, не semantic accuracy.
Decode/фильтр/TF/DDS/I/O/launch вне timer. Физический робот не запускался,
полная цель не возобновлена и не объявлена достигнутой.
Report results/detector-performance-20261003/report.json содержит input/binary/
source hashes, пять отдельных средних, параметры/hardware. Подготовленные
input.json/.npz/.bin сохранены. Команды — в DIAGNOSTICS. Новые benchmark
скрипты не меняют production detector. Compile и5replays успешны, diffcheck
без ошибок. Коммит/пуш не выполнялись.

## 03.10.2026 — удаление Python detector и подготовка checkpoint

По запросу пользователя оставлен только C++ detector/tracker. Удалены Python
core/tracker/robot circle fitting/shape_confidence и comparator oracle.
StaticBackground/cluster_xy/split_clusters перенесены без изменения вычислений
в geometry.py, который нужен фильтру коробок. profiles.py содержит только
ROS configuration constants, cloud.py — декодирование. Python LiDAR filter
oracle сохранён: просьба относится к detector, этот фильтр — другой компонент.
Offline replay/audit вызывают native detector_replay через subprocess;
legacy comparison с Python/Git-baseline удалён. Performance comparator стал
native-only, прежний результат15,62× сохранён как исторический.
C++ replay дополнен overrides/диагностическими полями; production node/kernel
при этой чистке не менялись. Регрессии перенесены на прямые C++ unit fixtures:
strong confirmation, bounded weak identity, stationary robot, curved body,
retained tall box. История прежних проверок не означает готовности поведения.

По ограничению пользователя **без прогонов**: ROS/Gazebo/bag replay не запускались.
Host tests188passed/1skip, CLI43passed; native detector_unit и компиляция
replay.cpp успешны. Compose sim+real валиден, compileall/diffcheck успешны.
В исходных Python больше нет импортов удалённого detector/tracker.
Проверка Compose сначала дала warnings безVEHICLE_ID, повтор сsimulator чистый.
Текст цели не обновлялся; общая цель остаётся paused и не достигнутой.
Build-only обоих образов завершён успешно: jr_image (5de61c8fe0ba),
jr_real_image (453ffc17934d). Команды: VEHICLE_ID=simulator docker compose
--env-file .env -f docker/docker-compose.yaml --profile duel build hsl-adapter;
docker compose -f docker/docker-compose.real.yaml build real.
Оба образа собраны после удаления Python detector, без запуска ROS/Gazebo.
Fetch повторён с --no-recurse-submodules: обычный fetch обновил refs, но
попытался получить удалённый legacy MPC submodule из другой ветки.
Текущая ветка не расходится с origin перед новым checkpoint.
Все изменения подготовлены к commit и push feature/decision-manager
по явному запросу пользователя; фактический SHA фиксируется в Git.

## 03.10.2026 — совместная проверка detector/planner после e98f688

Пользователь разрешил прогоны и попросил проверять изменения вместе.
Ограничение предыдущего checkpoint «без прогонов» отменено этим запросом.
Сначала исправлен static launch audit: planning_config присутствует в context,
слои параметров native_mppi объединяются с правильным приоритетом. Проверка
в jr_image без сети/ROS процессов прошла для обеих ролей и namespaces.
Fallback YAML MPPI согласован с внешним planning.yaml; прежде без внешнего
конфига возвращались inflation0.65/critical20 вместо0.45/10.

Базовая общая дуэль: seed0, explorer/guardian, старты config/match.yaml,
штатные коробки enabled, лимит90sim s, ROS_DOMAIN_ID76, Gazebo11421,
private Compose hsl-joint-check. Команда:
python3 benchmarks/run_duel_series.py --isolated-project hsl-joint-check
--ros-domain-id 76 --gazebo-port 11421 --runs 1 --start-seed 0
--first-role explorer --active-s 90 --trace --record-detector-scans
--wall-timeout-s 750
Результаты: results/isolated/hsl-joint-check/series-20261003T083233Z.
Explorer_goal за18.0sim s; скорость explorer0.325/guardian0.131м/с.
Explorer24fresh, ошибка median0.00760/p900.01204м; guardian0fresh,
только1strong frame и SEARCH. Минимальная парная дистанция≈0.508м.
MPPI swept rejects explorer7/guardian49 trace samples; 11guardian samples
имеют static254, остальные не подтверждают статическое препятствие.
Lateral global p90≈0.073м для обеих ролей. Это не успешная поимка.

Полные облака этого же заезда повторены offline с C++ ядром. Добавлены
детальные weak reasons (rim_sparse, line_fit, arc_span, inlier, rectangle,
merged), без изменения численных вычислений. Старый профиль: explorer24/
guardian0peer_near. Главные weak reasons line_fit247/320; строгая проверка
линии .35 подавляла rim даже при наличии точек соперника. Вариант line_ratio
.50 + arc75° дал85/56peer_near и0other на этом наборе; rectangle .70,
inlier .95, gap .12 и требование3strong остаются. Это proximity labels,
не измерение полной semantic accuracy/visibility recall. Real профиль не
изменён. Новые значения применены к simulation launch и offline defaults.

Совместный кандидат: hard radius .21м (body .178+clearance .032),
collision_margin_distance .03м; repulsion1/critical10/inflation.45/scaling8.
Глобальный и локальный контуры получают один radius, swept-check сохранён.
Static map_points больше не вызывают statefulSmallBoxFilter: sim adapter
публикует только known-wall samples, real adapter — empty heartbeat. Раньше
более новый heartbeat мог удалить pending scan views/box identities.
Подтверждение коробок теперь получает время только из отдельных scan.
189Python tests/1skip и C++ unit успешны. Образ обновлён инкрементально
через /tmp/hsl-joint-incremental.Dockerfile FROM предыдущий jr_image, COPY src,
colcon --packages-select hsl_perception_cpp hsl_planning hsl_sim_adapter
hsl_nav2_control (Release, symlink-install). SHA28918fd90798.
Запущен повтор той же общей дуэли, результаты ещё не оценены.
Цель active; устойчивое обнаружение/поимка и узкие проходы ещё не доказаны.

Повтор series-20261003T083958Z остановлен до разрешения движения:
проверка runtime_snapshot в harness ожидала старые line_ratio.35/arc90.
Исправлены ожидаемые значения на .50/75, сама проверка не отключалась.
Это инфраструктурная ошибка, не оценочный заезд. Остался собственный
private runtime с закрытым движением; следующий запуск того же проекта
штатно удаляет его перед стартом. Updated static launch audit прошёл.

Общий повтор series-20261003T084203Z завершён guardian_capture за8.4sim s,
seed0, тот же старт/коробки. Скорости explorer0.260/guardian0.297м/с;
collisions/wall/robot/small_box_contacts0 у обоих; RTF0.518. Explorer9fresh,
guardian10fresh (9aligned); median error0.00306/0.00531м, p900.01391/
0.01214м, aligned error>0.3 и box_near_false0. SEARCH→CAPTURE у стража,
GOAL→EVADE у исследователя. Swept trace6/2 (раньше7/49).
В активном referee окне ABS lateral global median/p90: explorer0.0081/
0.0465м, guardian0.06145/0.1137м. База explorer0.02705/0.0696, guardian
0.0076/0.089. Для стража отклонение выросло при переходе search→capture;
не выдавать улучшение скорости/поимки за улучшение всех метрик.
P90 соседнего изменения cmd_omega:0.0479/0.0765рад/с (база.0526/.1144),
интервалы команд нерегулярны; это не нормированный jerk.
Численный unit для44см диагонального прохода: radius.21 допускает route
и каждый segment; прежний .23 отклоняет прямой segment. Проверка касается
геометрии point walls, не исполнения MPPI внутри реального44см прохода.
Запущены ещё2повтора seeds1/2, alternate physical roles, trace без полного
повторного capture для сокращения нагрузки. Результат одной поимки не
доказывает устойчивость и не завершает общую цель.

Повторы завершились:

| Seed | Физические роли first/second | Исход | sim s | Explorer mean м/с | Guardian mean м/с | wall/robot collisions |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | explorer/guardian | guardian_capture | 8.4 | 0.260 | 0.297 | 0/0 |
| 1 | explorer/guardian | guardian_capture | 9.6 | 0.230 | 0.256 | 0/0 |
| 2 | guardian/explorer | guardian_capture | 20.2 | 0.236 | 0.232 | 0/0 |

Seeds1/2: results/isolated/hsl-joint-check/series-20261003T084433Z.
RTF.564/.539. Detector fresh explorer/guardian12/10(seed1),122/126(seed2);
p90 aligned errors≤0.021м, error>0.3м0 и initial-box-near false0.
Сохранённые траектории показывают actual minimum center-to-static-occupied-
cell-center distance.285–.317м (база.332–.348). Это точечная дистанция,
не измеренная свободная ширина коридора и не footprint clearance.
Роботы реагируют друг на друга в этих трёх условиях, floor mean.2 соблюдён,
зарегистрированных wall/robot contacts нет. Долгая серия, более узкий44см
проход в физическом Gazebo, повторная реальная валидация и все виды occlusion
не проверены. Поэтому общая цель не объявлена завершённой.
После изменения runtime contract полный pytest сначала обнаружил устаревший
fixture .35/90 в test_duel_runner; он синхронизирован .50/75. Проверка
чужого run_id/role/seed/reverse permission остаётся. Private worlds после
каждого содержательного заезда удалены harness, init/daemon не трогались.
Следующий шаг: целевой совместный runtime-тест тесного прохода и разбор
отклонения стража на обновляемой capture reference, без новых общих серий
до появления конкретной гипотезы. Текст цели/промпта не менялся.

Финальный полный pytest:190passed/1skip. Native unit и updated static
launch audit прошли, git diff --check чистый. Новые изменения этого цикла
ещё не закоммичены и не отправлены; предыдущий checkpoint e98f688 в origin.

## 03.10.2026 — targeted diagonal fixture и проверка real profile

Для отдельных fixture добавлен --obstacles-config в run_duel_series и
HSL_SIM_OBSTACLES_FILE в Compose. Default остаётся прежним. Runtime snapshot
проверяет SHA именно выбранного файла. benchmarks/scenarios содержит
narrow_diagonal_match.yaml и narrow_diagonal_obstacles.yaml; обычные
match.yaml/simulation_obstacles.yaml не менялись.

Первый placement [1.05,1.605,.7332] широкого бокса пересекал стены8/9,
SAT подтвердил ошибку fixture. series-20261003T085429Z timeout90sim s,
speed .070/.026; не использовать его как тест физически допустимого узкого
проезда. Следующий placement [.439,1.046,.526] ставит box вдоль стены,
но округление оставило ~0.7мм пересечение со стеной8. Его результат
series-20261003T085845Z explorer_goal32.9sim s, speed .200/.175,
wall/robot collisions0; оба detector работали (200/112fresh, p90errors
.0172/.0274м, aligned error>.3/initial-box-near false0). Пересечение
траектории с сегментом узкого места не найдено: роботы выбрали обход.
Это не доказательство проезда через тесный проход. После этого box pose
уточнён до [.4415,1.0417,.526], SAT теперь не обнаруживает wall overlaps.

Guardian47swept trace samples;38dynamic-only,9static присутствуют.
Остановка PURSUE при own≈[1.126,1.18] имеет rejection index50–57/60
на2.5–2.85s будущего прогноза, а не текущий contour. Гипотеза: трёхсекундный
прогноз в тесных поворотах чрезмерно задерживает доступное движение.
Кандидат horizon40steps/2.0s, все40steps всё ещё проходят swept-check.
Максимальные скорости не изменены. Конфиг/defaults/YAML синхронизированы.
Изменения Python/YAML доставлены overlay COPYsrc поверх symlink-install
image, SHA876065cef1a4; новые C++ вычисления не менялись.

По вопросу пользователя проверены real и новый simulation profiles на
одних761полных filtered clouds реального bag151820 (768raw,7initial missing
recorded TF). Повтор использует cachedpreparedinput предыдущей проверки:
input.bin SHA совпадает; параметры фильтра совпадают с current config.
Real325fresh — как прежде; simulation50fresh. Это не semantic precision/
recall и не replay всего ROS стека. Production real profile НЕ менялся:
line_ratio.50/arc75/inlier.80/gap.25/strong_extent.25, simulation сохраняет
inlier.95/gap.12/rectangle.70. Не переносить sim thresholds на реальный
стек по одному успешному Gazebo. Report в ignored results/real-bags-profile-
check-20261003/report.json. Исправлен accessor speed в NativeReplay, нужный
replay_real_shapes.py (новый native bridge ранее его не предоставлял).

## 03.10.2026 — прекращение утолщения статических стен динамическими сканами

Horizon40 в корректном fixture (seed3, series-20261003T091108Z) ухудшил
результат: explorer_goal37.4s, E.128/G.078м/с, wall/robot collisions0,
RTF.503. Guardian428swept samples, все dynamic-only; застревал у
[1.094,1.09] на25+sim s, часто rejected cell[1.275,.925]. Detector видел
соперника у обеих ролей; отказ не объясняется отсутствием трека. Горизонт
возвращён60, сокращение прогноза не оставлено в production.

По записи scan40.0:57точек возле этой клетки; все совпадают с occupied
статической стеной при допуске .04м + полудиагональ клетки. Симуляционный
ray noise stddev=.02м; собственный робот/соперник не объясняют эту группу
(peer≈[2.25,2.07]). Гипотеза: noisy wall returns повторно утолщают стены
в динамической costmap; повторная оптимизация начинается в том же запрете.

StaticWallReturns отделяет map-matching возвраты от неизвестных объектов.
Параметр global.static_wall_tolerance=.04м,0 отключает, разрешено0…0.08.
Применяется после SmallBoxFilter, численный детектор не менялся. StaticLayer
и known_grid сохраняют стены. obstacle_scan наносит только неизвестные
объекты; clearing_scan сохраняет все возвраты после фильтра коробок.
MPPI читает отдельные marking/clearing sources, оба проходят исключение
наблюдаемого соперника guardian. ObstacleMemory также получает полный набор
для ray clearing и отдельный набор для marking. При наличии known_grid
A* не добавляет noisy map_points обратно к статическим occupied centres.
Не расширяли blind radius, не отключали проверку всего rollout/footprint,
не вводили новый лимит скорости. Ограничение: выступы у стены в пределах
допуска могут быть неотличимы от шума; допуск можно отключить в YAML.

Тесты: стеновой шум отделяется,15см выступ/unknown/outside сохраняются,
нулевой допуск отключает сопоставление. Wall ray удаляет старую box occupancy
без нового нанесения стены.192passed/1skip; C++ сборка 4изменённых пакетов
прошла. Overlay содержит NumPy реализацию без новой SciPy зависимости.

series-20261003T092918Z не стартовал (parameter snapshot timeout), движение
не разрешалось. Во время запуска сменился адрес wlo1 с192.168.1.6 на
172.20.10.3; старые DDS участники пытались отправлять на прежний адрес/VPN.
series-20261003T095947Z прерван до старта: ROS TF между контейнерами также
не появился. Только runtime нашего private Compose project удалён.
Добавлен optional HSL_CYCLONEDDS_URI в sim common.yaml (default прежний),
для private испытания выбран lo+unicast127.0.0.1. Hardware Compose не менялся.
Runner сохраняет override в effective_environment. Это инфраструктурные
нестарты, не метрики поведения. Следующий запуск: тот же seed3/fixture,
проверка выхода стража из прежде блокировавшей позиции.

Перед этими изменениями fetched origin и fast-forward HEAD e98f688→fb98475;
4чужих коммита меняют RViz/real Docker GUI, не пересекаются с правками.
Все новые изменения сохранены незакоммиченными; goal/prompt не редактировались.

Повтор с wall dedup/horizon60: series-20261003T100334Z, seed3,
explorer_goal, E.224/G.155м/с, collisions0. Guardian прошёл прежнюю позицию
[1.094,1.09], но остановился после смены направления около[1.099,.745].
Большинство команд в паузе result=ok и vx≈0, yaw≈.5 при path bearing≈1.57:
гипотеза преждевременного отключения PathAngleCritic при пороге1rad.

Следующий эксперимент: shared MPPI.PathAngleCritic.max_angle_to_furthest
1.0→.35rad, вынесен в planning.yaml/default/native fallback.
series-20261003T100754Z, seed3, корректный тот же fixture: guardian_capture,
G.254м/с, E.055м/с, collisions0. Страж улучшился, исследователь существенно
замедлился: этот общий порог НЕ считать окончательным решением. Следующий
шаг — разобрать explorer rotation/route смены и отделить настройки
направления стража от допуска forward/reverse исследователя; затем совместный
повтор стандартных стартов. Текущий diff содержит эксперимент .35 для обеих
ролей. Цель пока не завершена: минимальная средняя .2 у обоих не подтверждена.

Нестарт series-20261003T100122Z вызван DDS warning перед JSON snapshot.
Runner теперь принимает ровно один однострочный JSON object среди строк
лога; неоднозначный вывод отклоняется. Тест прошёл. Loopback DDS задан
современным Interfaces/NetworkInterface XML, все subsequent private runtimes
автоматически очищены, живых Gazebo миров не осталось.

## 03.10.2026 — static-only costmap по указанию пользователя

Пользователь заморозил детекторы робота/коробок и указал: сейчас передавать
статическую карту как костмапу; в будущем источник будет от детектора.
В этом цикле алгоритмы/профили/фильтр классификации не менялись. Предыдущие
незакоммиченные изменения детектора сохранены, дополнительных правок нет.

MPPI plugins теперь только StaticLayer(`/map`) и InflationLayer. Убраны
cloud marking/clearing subscriptions/publishers wrapper, связанные с
ObstacleLayer; свежесть scan всё ещё проверяется. A* world.update получает
только grid_points из known_grid и пустой scan. Нет переноса map_points,
ObstacleMemory, трека робота или классов коробок в occupancy. Без static_grid
планировщик публикует NO_STATIC_MAP. obstacle_grid сохраняет диагностический
интерфейс, но содержит точную копию data статической карты.

SmallBoxFilter и его diagnostic clouds продолжают работать; они не влияют
на карту. Wall-return dedup больше не включён в transport; новый helper
сохранён без изменений как ранее написанная заготовка. static_wall_tolerance
удалён из активного planning.yaml/defaults. ObstacleMemory module и параметр
lifetime оставлены как заготовка, сейчас в активном planner memory нет.
Трек соперника остаётся у decision/тактики перехвата и убегания: это выбор
маршрута, не запись occupancy. Широкие и маленькие коробки вне статической
карты теперь не препятствия для A*/MPPI; физические коллизии Gazebo сохраняются.
Будущий detector costmap interface не реализовывался и не заявляется готовым.

Проверки:193passed/1skip, git diff --check, Compose config прошёл (warning
VEHICLE_ID пустой в прямом вызове без helm). Incremental sim image пересобран,
SHA75be343101bf. Без Gazebo/hardware выполнен ROS transport test в network-none
контейнере: известная стена сохраняется; LiDAR/map_points с препятствием на
свободном пути не изменяют A* occupancy; obstacle_grid.data==known_grid.data.
/tmp/hsl-static-wiring.py и /tmp/hsl-static-wiring.log — команда/результат.
Свежих дуэлей после static-only ещё не было; результаты предыдущих заездов
относятся к прежней динамической costmap. На hardware нужно пересобрать real
image перед использованием новых источников планирования. README/AGENTS/
DIAGNOSTICS/NAV2_MPPI_ADAPTATION актуализированы. Не коммитили/не пушили.

## 03.10.2026 — большая коробка также подвижна

Причина неподвижности: unknown_box_large не имела movable:true, генератор
создавал static=true. В основном simulation_obstacles.yaml и отдельном
narrow fixture добавлены movable:true, mass=.30кг, friction=.30 (приблизительные,
не измеренные параметры реальной коробки). Размер/поза/коллизия сохранены.
Детекторы не менялись, костмапа остаётся статической. YAML монтируется с
хоста; нужно пересоздать симуляционный мир, сборка не требуется.
Существующий SDF-тест обновлён для всех четырёх movable моделей:5passed.
Gazebo push trial не проводился в этом цикле, static=false проверен в SDF.
README/AGENTS обновлены, git diff --check чистый. Не коммитили/не пушили.

## 03.10.2026 — немного свободнее costmap, без прогона

По запросу пользователя уменьшена мягкая зона инфляции .45→.40м в
ObstaclesCritic и InflationLayer, collision_margin_distance .03→.02м.
Защитный радиус .21м, весь swept-check, веса, скорости и источники карты
сохранены. Детекторы не менялись. planning.yaml/defaults/native YAML
синхронизированы. load_planning и проверка соответствия native fallback
прошли, git diff --check чистый. Симуляция/заезды/сборка не запускались.
YAML подключается с хоста: параметры обычного запуска вступают в силу после
перезапуска стека без пересборки. Результат езды с этими числами не проверен.

## 03.10.2026 — исходный feature/detector вместо прежнего detector/box stack

Гипотеза/задача: вернуть исходный jr_perception из origin/feature/detector
b985515, напрямую на raw LiDAR, без нашего прежнего C++ detector и
классификатора коробок. Пользователь разрешил поправить ROS-интерфейс в
самом детекторе и отменил прогоны после добавления; последний приоритет —
реальная сборка, симуляцию пока отложить.

Изменено: старые hsl_perception(_cpp), SmallBoxFilter, obstacle memory,
semantic layer и их специальные replay/benchmark удалены. Общий декодер
PointCloud2, нужный real_observations, перенесён в hsl_sim_adapter.pointcloud.
Новый detector запускается напрямую из jr_perception; отдельного adapter
node/пакета нет. ROS-интеграция добавляет background_topic=/map, ожидание
карты и health/visible. Raw /livox/lidar и собственная navigation/self на
stamp → исходная сегментация/фит/tracker → opponent/odom. Decision, A* и
native MPPI читают этот топик. MarkerArray opponent/markers подключён к RViz
и записи start_real. AMCL продолжает использовать filtered cloud.
Костмапа static-only, numeric MPPI unchanged. Удалены только два неиспользуемых
параметра старого box/memory из global planning config. Радиус тела0,20,
все local параметры planning.yaml (inflation0,6/scaling2) сохранены.

Сегментация, tracker, background, record_background, detector_eval и оба
recognition YAML побайтно сверены с b985515. AST исходного process после
исключения health-публикаций совпадает; функции формы, преобразования
Odometry/MarkerArray, интерполяции позы и sensor TF не менялись. Перед
алгоритмическими правками или изменением recognition params нужно спросить
пользователя. Фон статической 2D-карты задаёт occupied>=50, walls до0,70м,
полz=0, нулевой поворот origin; его корректность на новых bag пока не оценена.

Реальная сборка:
`docker build -t jr_real_image:latest -f docker/Dockerfile.real .`
Успешно10пакетов, image sha256:bba509fae9ad2d6d2c56dea549c49fd40b8e3e8ab674f1a1bb1328dbc16498e8.
Лог /tmp/hsl-branch-real-build2.log. Аудит установленного real launch:
`docker run --rm --network none -v "$PWD/config:/config:ro" -v "$PWD/benchmarks/audit_real_detector_launch.py:/tmp/audit.py:ro" --entrypoint bash jr_real_image:latest -c 'source /solution/install/setup.bash && python3 /tmp/audit.py'`
passed: raw input, timestamped own pose, map background, detector executable,
StaticLayer+InflationLayer, старые пакеты отсутствуют; ROS nodes не запускались.
Core tests162passed/1skip; helm tests43passed. git diff --check чистый.

Новые bag найдены в hsl2026Extra/recordings_last8. Первый093554-autonomous
259с содержит raw LiDAR, navigation/self, TF и /map; пользователь описал
езду вокруг другого робота. Replay не запускался по последнему указанию.
Метрик распознавания до/после нет; сборка не доказательство распознавания.
Исходный tracker может выдавать coasting-прогноз, visible означает свежесть
выхода, а не новую измеренную детекцию. Коробки — кластеры в MarkerArray,
семантического small/large classifier у этой ветки нет.

Симуляционные исходники топиков согласованы: wheel odom второго робота
/opponent/wheel/odom, чтобы не конкурировать с robot detect /opponent/odom.
Финальный sim-образ/заезд после всех изменений не проверялись и не являются
подтверждённым результатом этого цикла. Реальный робот/драйверы не запускались.
Следующий шаг по разрешению пользователя: проверить новый detector на первом
bag с записанными map pose/TF, затем аппаратный запуск. Не менять recognition
по результатам без согласования. Не коммитили/не пушили. Предыдущее дерево
сохранено в /tmp/hsl-before-detector.patch и архиве
/tmp/hsl-replaced-detectors-20261003.tar.gz; goal text не изменялся.
