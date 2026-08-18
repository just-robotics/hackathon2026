# hackathon2026

Стек автопилота (Autoware/ROS 2 Humble) и симулятор NVIDIA Isaac Sim, запускаемые
в docker через обёртку `helm`.

## Структура

| Путь | Что внутри |
| --- | --- |
| [docker/](docker/) | `Dockerfile` автопилота, compose-файлы, кэш симулятора |
| [docker/common.yaml](docker/common.yaml) | базовый сервис `app`, от которого наследуются модули автопилота |
| [docker/docker-compose.yaml](docker/docker-compose.yaml) | сервисы: модули автопилота, `isaac`, `hub-cache`, `web-viewer` |
| [docker/init-compose.yaml](docker/init-compose.yaml) | инфраструктурные сервисы (`ros-daemon`), поднимаются первыми |
| [docker/launch.yaml](docker/launch.yaml) | описание команд `helm` |
| [docker/web-viewer/](docker/web-viewer/) | сборка браузерного клиента симулятора |
| [helm_launch/](helm_launch/) | исходники CLI `helm` |
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
  не задаёт запуск, а передаёт флаги в Kit.
- **`hub-cache`** — кэш Omniverse Hub, `nvcr.io/nvidia/omniverse/hub_workstation_cache`.
  Обязателен: в образе симулятора прошит `HUB__ARGS__DETECT_ONLY=true`, то есть
  Hub он не запускает, а только ищет уже работающий. Без этого сервиса в логах
  бесконечно повторяется `Hub failed to launch ... without writing file`.
- **`web-viewer`** — браузерный клиент, опционален (см. ниже).

### Первый запуск

Контейнер работает от uid 1234 (пользователь `isaac-sim`, его HOME — `/isaac-sim`),
поэтому каталоги кэша должны принадлежать этому uid, иначе Hub не сможет
записать свой конфиг:

```bash
sudo chown -R 1234:1234 docker/.isaac-sim-docker
```

Каталог `docker/.isaac-sim-docker/` хранит кэш шейдеров, логи, данные и Documents
между перезапусками — он в `.gitignore`. Первый старт долгий (прогрев кэша),
последующие заметно быстрее.

```bash
helm up isaac
helm flogs isaac     # ждём "Isaac Sim Full Streaming App is loaded" и "app ready"
```

Готовность отслеживает healthcheck: он ищет `AppReady` в свежем логе Kit, с
запасом `start_period: 180s`. От него зависит `web-viewer` — браузерный клиент
не поднимется, пока симулятор не отрапортует готовность.

### Просмотр: нативный клиент

Основной способ. Даёт более чёткую картинку, чем браузер, потому что не
масштабирует поток под размер окна.

Ставится один раз, на хост (не в контейнер):

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

### Просмотр: браузер

Альтернатива, когда клиент ставить некуда или нужен доступ с другой машины.
Собирается локально из [docker/web-viewer/](docker/web-viewer/) (Dockerfile от
NVIDIA, само приложение тянется из их npm-реестра).

По умолчанию отдельно от симулятора:

```bash
helm build web-viewer    # один раз, сборка через npm — небыстрая
helm up web-viewer
```

Затем <http://127.0.0.1:8210> в браузере на движке Chromium.

Чтобы вьювер поднимался сразу вместе с симулятором, в [.env](.env):

```ini
USE_WEB=true    # false -- только по явной команде
```

После этого `helm up isaac` поднимает все три сервиса разом. Значение читает
`helm` и подставляет сервису `web-viewer` профиль `isaac` либо `web-viewer` —
так же, как он поступает с `USE_RESOURCES_SPLITTING` и `USE_SIMULATION`.

Адрес и порты **зашиваются в сборку**, поэтому после смены `WEB_VIEWER_HOST` или
портов нужен пересбор:

```bash
helm build web-viewer && helm restart web-viewer
```

Порт вьювера меняется через `WEB_VIEWER_PORT` в `.env`.

### Качество картинки

По умолчанию Isaac Sim рендерит с DLSS в режиме Performance — сцена считается
примерно в половине разрешения и апскейлится, из-за чего картинка мылит. В
compose это переопределено флагом Kit:

```yaml
command:
    - "--/rtx/post/dlss/execMode=2"
```

Значения `execMode`: `0` Performance, `1` Balanced, `2` Quality, `3` Auto —
других нет. Флаги уходят прямо в Kit, так как `runheadless.sh` пробрасывает
`"$@"`, поэтому сюда же можно добавлять любые другие настройки Kit.

Разрешение отдельно задавать не нужно: в 6.0.1 включён `allowDynamicResize`,
поток подстраивается под окно клиента.

### Просмотр с другой машины

Симулятор ничего дополнительно не требует: он не фиксирует адрес медиапотока, а
перечисляет все свои, и WebRTC выбирает доступный. В нативном клиенте достаточно
указать адрес хоста вместо `127.0.0.1`.

Браузерному клиенту адрес зашивается в сборку, поэтому в [.env](.env):

```ini
WEB_VIEWER_HOST=192.168.1.42
```

и пересобрать: `helm build web-viewer`.

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

**Предупреждения, на которые не нужно реагировать:** `Failed to open [/var/run/utmp]`,
`Active user not found. Using default user [kiosk]`, `_createExtendCursor: No windowing`,
`Possible version incompatibility ... IStageReaderWriter` — штатный шум headless-режима.

## Ссылки

- [Isaac Sim в контейнере](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/install_container.html) — переменные окружения, volumes, headless-режим.
- [Клиенты стриминга](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/manual_livestream_clients.html) — установка клиента, порты, подключение.
- [Загрузки Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/download.html) — актуальные версии клиента и контрольные суммы.
