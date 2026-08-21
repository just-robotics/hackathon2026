# hackathon2026

Стек автопилота (Autoware/ROS 2 Humble) и симулятор NVIDIA Isaac Sim, запускаемые
в docker.

Управление — через `helm`, CLI из [helm_launch/](helm_launch/) в этом же
репозитории. Ставится вручную, см. «Установка».

## Что нужно на хосте

- Docker с плагином `docker compose` (v2) и правами запуска без `sudo`
- Python 3 с `pip`
- NVIDIA GPU + драйвер и [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
  — нужны для Isaac Sim; без них поднимется только автопилот

## Установка

1. Склонировать репозиторий:

   ```bash
   git clone git@github.com:just-robotics/hackathon2026.git
   cd hackathon2026
   ```

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

## Запуск

### Первый запуск

Контейнер симулятора работает от uid 1234 (пользователь `isaac-sim`, его HOME —
`/isaac-sim`), поэтому каталоги кэша должны принадлежать этому uid, иначе Hub не
сможет записать свой конфиг:

```bash
sudo chown -R 1234:1234 docker/.isaac-sim-docker
```

Каталог `docker/.isaac-sim-docker/` хранит кэш шейдеров, логи и данные
между перезапусками — он в `.gitignore`. Достаточно одного раза, дальше сразу к
обычному запуску.

### Обычный запуск

```bash
helm build all          # образы автопилота, долго
helm up simulation      # автопилот + симулятор
helm ps                 # что поднялось
helm flogs isaac        # логи симулятора, ждём "app ready"
helm down simulation    # остановить
```

Первый старт симулятора долгий — прогревается кэш, последующие заметно быстрее.
Готовность отслеживает healthcheck: он ищет `AppReady` в свежем логе Kit, с
запасом `start_period: 180s`.

Дальше — [просмотр картинки](#просмотр) в нативном клиенте.

Настройки — карта, образ, профили, параметры симулятора — в [.env](.env),
пересборка после правок не нужна.

## Структура

| Путь | Что внутри |
| --- | --- |
| [docker/](docker/) | `Dockerfile` автопилота, compose-файлы, кэш симулятора |
| [docker/common.yaml](docker/common.yaml) | базовый сервис `app`, от которого наследуются модули автопилота |
| [docker/docker-compose.yaml](docker/docker-compose.yaml) | сервисы: модули автопилота, `isaac`, `hub-cache` |
| [docker/init-compose.yaml](docker/init-compose.yaml) | инфраструктурные сервисы (`ros-daemon`), поднимаются первыми |
| [docker/launch.yaml](docker/launch.yaml) | описание команд `helm` |
| [helm_launch/](helm_launch/) | исходники CLI `helm` |
| [isaac/](isaac/) | сцена TurtleBot 2 / Kobuki для Isaac Sim: URDF, Kit-расширение, ROS-графы |
| [.env](.env) | все настройки: образ, карта, профили, параметры симулятора |

## helm

Единая точка входа, вызывается из любой директории. Напрямую `docker compose`
дёргать не нужно — `helm` сам переходит в `docker/` и подставляет нужные файлы.

```bash
helm build <профиль>    # собрать образы
helm up <профиль>       # запустить (+ init-сервисы)
helm ps                 # что сейчас работает
helm flogs <профиль>    # логи, follow
helm restart <профиль>  # перезапуск без пересоздания контейнеров
helm down <профиль>     # остановить (kill)
helm clean <профиль>    # остановить и удалить контейнеры
helm enter <сервис>     # bash внутри контейнера
helm exec <сервис> ...  # выполнить команду внутри контейнера
helm commit <сервис> <образ>  # закоммитить контейнер в образ
```

Список профилей и сервисов подскажет сам `helm`:

```bash
helm up -h
```

Основные профили: `all` (весь автопилот), `simulation` (автопилот + симулятор),
`isaac` (только симулятор), `debug`, плюс отдельные модули — `vehicle`,
`sensing`, `localization`, `perception`, `planning`, `control`, `api`, `tools`,
`transforms`.

## Isaac Sim

### Из чего состоит

Профиль `isaac` поднимает:

- **`isaac`** — симулятор, образ `nvcr.io/nvidia/isaac-sim:6.0.1`. Стартует сам:
  у образа `ENTRYPOINT=/isaac-sim/runheadless.sh`, поэтому `command` в compose
  не задаёт запуск, а передаёт флаги в Kit. После `AppReady` Kit-расширение
  `isaacsim.hackathon.turtlebot` грузит Simple Room, импортирует TurtleBot 2
  (база Kobuki) и вешает RTX-лидар в духе Livox Mid-360. Комнату можно
  отключить, см. [Флаг комнаты](#флаг-комнаты).
- **`hub-cache`** — кэш Omniverse Hub, `nvcr.io/nvidia/omniverse/hub_workstation_cache`.
  Обязателен: в образе симулятора прошит `HUB__ARGS__DETECT_ONLY=true`, то есть
  Hub он не запускает, а только ищет уже работающий. Без этого сервиса в логах
  бесконечно повторяется `Hub failed to launch ... without writing file`.

Симулятор говорит с автопилотом по ROS 2 Humble + CycloneDDS (`ROS_DISTRO=humble`,
тот же `cyclonedds.xml`, что у сервисов автопилота). Образ NVIDIA по умолчанию
поднимает Jazzy — без этих переменных топики не будут видны.

`helm up` не пересоздаёт контейнеры. После смены volumes/command у `isaac`
нужно `helm clean isaac && helm up isaac` (или `helm clean simulation && helm up simulation`).

### TurtleBot 2 и Livox Mid-360

В Simple Room спавнится Kobuki с пластинами TurtleBot 2 на шестигранных стойках.
Вход — `geometry_msgs/Twist` на `/cmd_vel`, лидар публикует
`sensor_msgs/PointCloud2` на `/livox/lidar` (`frame_id=livox_frame`).
Дополнительно идут `/clock`, `/odom` и TF `odom` → `base_link` → `livox_frame`.

Лидар подвешен под верхней пластиной вверх ногами: если смотреть на робота
сзади, оси `livox_frame` — X вправо, Y назад, Z вниз. Стойки между пластинами
заданы как visual-геометрия самих пластин, отдельных тел и коллизий не создают,
но RTX-лидар их видит — в облаке остаются шесть узких теней, как на железе.

Высота пола определяется по самой широкой горизонтальной поверхности Simple Room,
робот спавнится на 5 см выше неё и досаживается физикой. Своя плоскость земли
добавляется, только если у комнаты нет коллизий.

Драйвер `livox_ros_driver2` из образа автопилота — для железа, в симуляции его
запускать не нужно: облако публикует сам Isaac.

Проверка, когда симулятор уже в `AppReady` и timeline играет (в логах Kit:
`[turtlebot2] scene ready, timeline playing`). Для `ros2` нужен контейнер
автопилота — профиль `simulation` или отдельно `tools`/`debug`:

```bash
helm enter tools
source /autoware/install/setup.bash

ros2 topic list
# ожидаются /cmd_vel, /livox/lidar, /odom, /clock, /tf, /tf_static

ros2 topic hz /livox/lidar
ros2 topic info /cmd_vel
# у /cmd_vel должен быть subscriber со стороны Isaac

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Isaac штампует сообщения временем симуляции, поэтому RViz2 нужно запускать с
`use_sim_time`, иначе облако считается устаревшим и не рисуется:

```bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
# Fixed Frame: odom или livox_frame, дисплей PointCloud2 на /livox/lidar
```

В WebRTC-клиенте робот должен стоять на полу комнаты (стол Simple Room
отключается), оранжевый цилиндр лидара — под верхней пластиной.

### Флаг комнаты

Simple Room включается флагом `USE_SIMPLE_ROOM` в [.env](.env):

```bash
USE_SIMPLE_ROOM=false  # true | false
```

Флаг управляет только окружением. При `false` комната не грузится, а вместо неё
кладётся серая плоскость 50×50 м на `z = 0` плюс `DomeLight` и `DistantLight`;
робот, лидар и ROS-графы поднимаются как обычно — пустое пространство удобно,
чтобы смотреть облако лидара без стен. Свои источники света тут обязательны: у
Simple Room они есть в самом ассете, а на голой сцене без них картинка чёрная. Значение уходит в контейнер переменной `HACKATHON_ROOM`,
поэтому после правки контейнер нужно пересоздать:
`helm clean isaac && helm up isaac`.

Разово, без правки файла, значение можно передать через окружение — переменные
окружения у compose приоритетнее `.env`:

```bash
helm clean isaac && USE_SIMPLE_ROOM=false helm up isaac
```

Если Kit запускается не через compose и `HACKATHON_ROOM` не задан, работает
флаг Kit (значение по умолчанию `true` лежит в `extension.toml`):

```
--/exts/isaacsim.hackathon.turtlebot/room=false
```

### Просмотр

Картинка идёт по WebRTC в нативный клиент. Ставится один раз, на хост (не в
контейнер):

```bash
curl -fLO https://downloads.isaacsim.nvidia.com/isaacsim-webrtc-streaming-client-2.0.0-linux-x86_64.deb
sudo dpkg -i ./isaacsim-webrtc-streaming-client-*-linux-*.deb && sudo apt -f install
```

Запуск — из меню приложений или командой:

```bash
isaacsim-webrtc-streaming-client
```

В поле адреса указать `127.0.0.1`. Порты клиент подставит сам: 49100 (TCP,
сигналинг) и 47998 (UDP, поток). Контейнер работает в `network_mode: host`,
так что порты доступны напрямую, пробрасывать ничего не нужно.

### Качество картинки

По умолчанию Isaac Sim рендерит с DLSS в режиме Performance — сцена считается
примерно в половине разрешения и апскейлится, из-за чего картинка мылит. В
compose для TurtleBot стоит Balanced: RTX-лидар и так тяжёлый, Quality даёт
слишком низкий FPS.

```yaml
command:
    - "--/rtx/post/dlss/execMode=1"
    - "--ext-folder"
    - "/workspace/isaac/exts"
    - "--enable"
    - "isaacsim.hackathon.turtlebot"
```

Значения `execMode`: `0` Performance, `1` Balanced, `2` Quality, `3` Auto —
других нет. Флаги уходят прямо в Kit, так как `runheadless.sh` пробрасывает
`"$@"`, поэтому сюда же можно добавлять любые другие настройки Kit.

Разрешение отдельно задавать не нужно: в 6.0.1 включён `allowDynamicResize`,
поток подстраивается под окно клиента.

### Просмотр с другой машины

Симулятор ничего дополнительно не требует: он не фиксирует адрес медиапотока, а
перечисляет все свои, и WebRTC выбирает доступный. В клиенте достаточно указать
адрес хоста вместо `127.0.0.1`.

## Диагностика

**Чёрный экран в клиенте, в логах повторяется `Got stop event while waiting for client connection`.**
Сигналинг установлен, а медиаканал — нет: не сходится ICE. Типичная причина —
VPN на хосте в связке с жёстко заданным адресом медиапотока. Если симулятору
передать `ISAACSIM_HOST`, он анонсирует именно этот адрес как `publicIp`, но
ответы уходят через маршрут по умолчанию — то есть с адреса VPN-интерфейса;
клиент считает их чужими и отбрасывает. Поэтому в compose переменная сервису
`isaac` намеренно не передаётся. Проверить, идёт ли поток:

```bash
sudo tcpdump -i lo -nn -c 20 'udp port 47998'
```

Пары «запрос на один адрес — ответ с другого» означают именно эту проблему.

**`Hub failed to launch ... without writing file`.**
Не поднят `hub-cache`, либо каталоги кэша не принадлежат uid 1234 — Hub не может
записать конфиг. Проверить владельца `docker/.isaac-sim-docker` и при
необходимости повторить `chown` из раздела «Первый запуск».

**Симулятор падает с segfault в `librtx.scenedb.plugin.so`.**
Несовместимость версии Isaac Sim с GPU. Наблюдалось на 5.0.0 с RTX 50-й серии;
лечится обновлением тега образа. Встроенная проверка совместимости при этом
рапортует `PASSED` — она смотрит версию драйвера и модель GPU, но не проверяет
работоспособность рендера, так что доверять ей нельзя.

**`failed switching to "aw": operation not permitted` в контейнере `isaac`.**
Значит под тегом образа симулятора оказалась сборка автопилота. Сервис `isaac`
намеренно **не** наследует `common.yaml`: там прописан `build` автопилота, и
`docker compose build` затирал бы им образ NVIDIA. Восстанавливается перекачкой:

```bash
docker pull nvcr.io/nvidia/isaac-sim:6.0.1
```

**`ROS2 Bridge startup failed` / `librcutils.so: cannot open shared object file`.**
Задан `ROS_DISTRO=humble`, но Kit не видит Humble-библиотеки. У сервиса `isaac`
должен быть `LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.core/humble/lib`
(скрипт `setup_ros_env.sh` в образе дописывает этот путь только если
`ROS_DISTRO` ещё не задан). Пересоздать контейнер: `helm clean isaac && helm up isaac`.

**Робот есть, а комнаты нет, в логах `[turtlebot2] Simple Room disabled by flag`.**
Так и задумано при `USE_SIMPLE_ROOM=false` в [.env](.env) — либо значение
осталось в окружении от прошлого запуска. Вернуть `true` и пересоздать
контейнер: `helm clean isaac && helm up isaac`.

**Робот висит над полом или проваливается.**
Пол Simple Room лежит примерно на `z = -0.76`, а не на нуле, поэтому высота
спавна считается по геометрии комнаты. В логах Kit есть контрольные замеры
`[turtlebot2] .../base_link world z=...` — значение должно быть стабильным.
Если комната не загрузилась (нет сети до asset root), сцена откатывается на
плоскость с землёй на `z = 0`.

**`ros2 topic hz /livox/lidar` считает сообщения, но в RViz2 точек нет.**
RViz запущен без `use_sim_time:=true` — время симуляции расходится со
временем хоста, и облако отбрасывается как устаревшее. Второй вариант — не тот
`Fixed Frame`: нужен `odom` или `livox_frame`.

**`[turtlebot2] scene setup failed` в логах Kit.**
Расширение не смогло открыть Simple Room или импортировать URDF. Проверить,
что каталог `isaac/` смонтирован в `/workspace/isaac`, контейнер пересоздан
после смены compose (`helm clean isaac && helm up isaac`) и есть доступ к
asset root Isaac (облако NVIDIA). Без сети Simple Room не скачается — тогда
сцена должна откатиться на плоскость с землёй.

**Предупреждения, на которые не нужно реагировать:** `Failed to open [/var/run/utmp]`,
`Active user not found. Using default user [kiosk]`, `_createExtendCursor: No windowing`,
`Possible version incompatibility ... IStageReaderWriter` — штатный шум headless-режима.

## Ссылки

- [Isaac Sim в контейнере](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/install_container.html) — переменные окружения, volumes, headless-режим.
- [Клиенты стриминга](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/manual_livestream_clients.html) — установка клиента, порты, подключение.
- [Загрузки Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/download.html) — актуальные версии клиента и контрольные суммы.
