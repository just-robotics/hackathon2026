# Статическая карта по записанному bag

## Подготовлено сейчас

Офлайн-аудит `tools/inspect_mapping_bag.py` работает в существующем
`jr_real_image`: читает запись целиком и выводит JSON с топиками, количеством
сообщений, частотами, header/record timestamps, полями PointCloud2, TF-связями,
размахом/длиной одометрического трека и нормой ускорения IMU.
Фиксирует отсутствующие входы, неправильные типы, невалидные данные и метки
времени. Не публикует ROS-топики и не управляет роботом.

Из maze_bag_v1 уже получена статическая карта (02.10.2026), см.
[описание результата](../config/maps/README.md). Физическая локализация по ней
не проверена. Реальные/симуляционные launch не изменены: backend работает
только в изолированном офлайн-контейнере.

## Что передать

Лучше передать **всю папку сессии** `recordings/<UTC>-bag/`:

- `bag/metadata.yaml` и все `.mcap` (либо все `.db3` для другой записи);
- `real.yaml`, mission YAML, `recording.yaml`, Livox JSON;
- `session.json`, если он есть.

Не переименовывать топики/frames и не исправлять timestamps вручную.
Mounting LiDAR и стартовую позу будем брать из конфигов именно этой сессии.
Для записи важно покрыть доступные коридоры, сохранять наблюдение стен и
вернуться к уже пройденным участкам: это позволит оценить расхождение карты.
Данные с другим движущимся роботом/людьми потребуют очистки динамических объектов.

## Проверка записи

Из корня репозитория подставить абсолютный путь к **каталогу bag**:

```bash
mkdir -p results/real-bag-check-new
docker run --rm --network none --cap-drop ALL \
  -v /absolute/path/session/bag:/bag:ro \
  -v "$PWD/tools/inspect_mapping_bag.py:/audit_mapping_bag.py:ro" \
  --entrypoint bash jr_real_image:latest -lc \
  'source /solution/install/setup.bash && python3 /audit_mapping_bag.py /bag' \
  > results/real-bag-check-new/audit.json
```

Для иных имён добавить `--lidar-topic`, `--odom-topic`, `--imu-topic` после
`/bag`. Вход примонтирован read-only, устройств и сети у процесса нет.
Ошибки контракта возвращают exit2; предупреждения требуют разбора, но не
означают автоматическую непригодность записи для любого SLAM.
Список TF-связей не доказывает непрерывную доступность преобразований на stamp
каждого скана. Калибровка IMU и взаимной привязки сенсоров и отсутствие дрейфа этим аудитом
не подтверждаются. JSON всегда содержит `map_quality_verified: false`.

## Порядок работы после получения bag

1. Проверить полноту записи, timestamps/часы, облака XYZ, IMU и TF, фактическое
   движение и покрытие. Разобрать масштаб IMU, sensor extrinsics и время точек.
2. Восстановить траекторию и собрать облака в единой системе. Накопление
   по wheel odom годится только как исходная оценка: нельзя считать его
   окончательной картой при заметном дрейфе. Проверить совмещение повторных
   проходов; выбрать и проверить офлайн-регистрацию/SLAM по фактическим данным.
3. Очистить пол/потолок/корпус/динамические объекты. Получить 3D-облако для
   проверки и 2D occupancy grid с ray tracing: свободные, занятые и неизвестные
   клетки должны различаться. Не считать всё вне облака свободным.
4. Экспортировать `arena.yaml` + `arena.pgm`, сохранить 3D-облако, траекторию,
   параметры обработки, систему координат и отчёт качества. Не включать
   радиус робота в физические стены карты: inflation выполняет Nav2.
5. Проверить ширину коридоров, отсутствие двойных стен, замыкание повторных
   проходов и ориентацию изображения/origin; загрузить карту в Nav2 map_server.

Начальная кандидатная resolution —0.05м, окончательная выбирается по
геометрии и данным. Высотный срез препятствий выбирается по фактической
плоскости пола/корпусу; заранее фиксированный срез не считается проверенным.

## Подключение готового результата

```text
config/maps/arena.yaml
config/maps/arena.pgm
```

В `arena.yaml` image должен быть относительным путём `arena.pgm`. В
`config/real.yaml` задаётся `map_file: maps/arena.yaml`; `arena_bounds` и
стартовые позы миссии задаются в системе полученной карты. После этого
`start_real` поднимает map_server/static layer и передаёт карту planner/detector.

В текущем start_real карта подключена вместе с `localization: amcl`:
pointcloud_to_laserscan → AMCL → динамический map→odom → navigation/self.
Настройки и запуск описаны в [REAL_ROBOT.md](REAL_ROBOT.md#6-карта-и-локализация).
Режим odometry остаётся доступен, но сохраняет дрейф.

## Фактическая обработка maze_bag_v1

Вход: /home/eddyswens/ROS/hsl2026Extra/maze_bag_v1, ROS Lyrical metadata9,
один MCAP,1408 облаков/28163 IMU. Humble metadata parser не поддерживает
новый список QoS; audit/replay открывают сам MCAP. Исходник не переписывается.
Временные метки точек FLOAT64 абсолютные ns: offset0…100.3мс относительно
header, IMU≈200Гц и acceleration norm≈1g. Использован адаптер точного времени
точек, FAST-LIO сам нормализует ускорение по стационарному началу.

Backend — [FAST-LIO2 ROS2 Ericsii](https://github.com/Ericsii/FAST_LIO_ROS2),
ros2 commit2fffc570a25d0df172720bac034fbdb6a13d2162, ikd-Tree e2e3f4e9.
Использован уже существовавший изолированный snapshot с точным timestamp
адаптером HSL: hsl_offline_mapping:maze-v1,
sha256:6eab6677f42acc1c0f82466b08674a03003dc500b08d1a4fca6fb68639b28664.
Его исходники/лицензии сохранены в
results/real-bag-check-maze-v1/fastlio-source-used.tar.gz. Он не входит в
jr_real_image, не является основным симуляционным runtime и не нужен для
загрузки готовой карты. Метка snapshot сохраняет backend при будущей
пересборке jr_image. На другом компьютере snapshot нужно передать отдельно
через docker save/load либо собрать совместимый backend по архиву исходников.

Параметры в config/maps/maze_bag_v1_processing.yaml. Upside-down mounting
взят из симуляции по пояснению пользователя; vendor translation IMU↔LiDAR
вращается в установленные оси. Это допущение, не измеренная калибровка.
Пол оценён robust plane fit, median residual0.0093м; сетка строится из слоя
0.10…0.40м над полом, range≤8м. Свободные клетки получены ray tracing и
пройденным footprint, стены не расширены/не дорисованы. Full scene сохранена
отдельно; bbox выходной карты ограничен крупнейшим наблюдаемым компонентом
стен с0.25м запасом, без изменения клеток внутри.

Повтор registered replay (на том же компьютере; новый пустой output):

```bash
mkdir -p results/real-bag-check-repeat/run/fastlio-logs
cp config/maps/maze_bag_v1_processing.yaml results/real-bag-check-repeat/run/parameters.yaml
docker run --rm --network none --cap-drop ALL --user "$(id -u):$(id -g)" \
  -e HOME=/tmp -e ROS_DOMAIN_ID=98 \
  -v /home/eddyswens/ROS/hsl2026Extra/maze_bag_v1:/bag:ro \
  -v "$PWD/tools/offline_lio_replay.py:/offline_lio_replay.py:ro" \
  -v "$PWD/results/real-bag-check-repeat/run:/work" \
  -v "$PWD/results/real-bag-check-repeat/run/fastlio-logs:/autoware/src/fast_lio/Log" \
  --entrypoint bash hsl_offline_mapping:maze-v1 -lc \
  'source /autoware/install/setup.bash && unset CYCLONEDDS_URI ROS_LOCALHOST_ONLY && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && python3 /offline_lio_replay.py /bag/0_maze_bag_v1_2026_10_02-13_09_53.mcap /work /work/parameters.yaml --rate 1.5'
```

Mount Log нужен для штатного закрытия старого backend: без writable fopen
его destructor вызывает fclose(NULL). В первом проходе это дало SIGSEGV
после завершения replay, данные сохранены. Итоговый проход с Log mount
завершился exit0:1405 registered scans/poses, unpaired0. Первые сканы заняты
инициализацией; последний без достаточной IMU может не обрабатываться.
Это не объявляется потерей или полнотой всех1408 output scans.

Повтор projection:

```bash
docker run --rm --network none --cap-drop ALL --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -v "$PWD/results/real-bag-check-repeat/run:/registered:ro" \
  -v "$PWD/results/real-bag-check-repeat:/work" \
  -v "$PWD/tools/export_static_map.py:/export_static_map.py:ro" \
  --entrypoint bash hsl_offline_mapping:maze-v1 -lc \
  'python3 /export_static_map.py /registered /work/map --crop-to-walls --max-height .4'
```

Результаты maze.yaml/PGM/PNG/preview, maze.pcd, trajectory.csv,
occupancy_evidence.npz и quality.json. Для другого bag менять вход,
параметры, время/привязку сенсоров/границы высоты по аудиту. Эта обработка не
доказывает пригодность этих параметров для любой записи.

## Выравнивание осей

Повторная проекция с `--align-walls --max-height .40 --crop-to-walls`
выбирает yaw в пределах±5° по концентрации проекций наблюдаемых стен.
Для maze_bag_v1 получено+1,8°. Поворот применяется к облакам/позам до
растеризации; сохранены неизвестные клетки, стены не дорисовываются.
Обновлены PGM, origin, bounds, PCD, trajectory и raw_to_map_rotation.

## Проверка локализации без аппаратных драйверов

```bash
docker run --rm --network none \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e ROS_DOMAIN_ID=93 \
  -v "$PWD/config:/config:ro" \
  -v "$PWD/tools/check_real_localization.py:/check.py:ro" \
  -v /home/eddyswens/ROS/hsl2026Extra/maze_bag_v1:/bag:ro \
  -v "$PWD/results/real-bag-check-maze-v1:/data" \
  jr_real_image bash -lc 'source /solution/install/setup.bash && python3 /check.py /bag /data/run2 /config/maps/maze_bag_v1_quality.json /data/localization-final.json --exercise-permission'
```

Нужны сохранённые run2/trajectory.csv и parameters.yaml из офлайн-обработки.
Тест поднимает весь robot.launch с drivers_enabled=false, без /dev и сети
аппаратуры; публикует исходные Livox облака и TF/odom по восстановленной
траектории с искусственным линейно растущим дрейфом +0,30/-0,20м.
Время перебазировано к текущим wall stamps, rate2. Это не настоящий wheel odom.
Сравнивается navigation/self с LIO-derived reference; источники карты и
reference зависимы. Проверяются готовность, прохождение позиции в стек,
единственный cmd_vel publisher и отзыв движения после потери сенсоров.
`--exercise-permission` разрешает команды исключительно внутри этого
изолированного теста без драйверов; не применять его на аппаратном контейнере.
