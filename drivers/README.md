# Драйверы реального робота

Пакеты перенесены из предоставленной пользователем папки `hsl25-master`
(комплект HSL25). Сохранены исходные лицензии и авторство.

| Пакет | Назначение |
| --- | --- |
| kobuki_core | Библиотека USB/serial-драйвера базы |
| kobuki_ros_interfaces | Сообщения базы |
| kobuki_node | ROS 2 odom, IMU, сенсоры, команды моторов |
| kobuki_description | URDF Kobuki и TF корпуса |
| livox_ros_driver2 | Предоставленная версия драйвера Mid-360 |

Собираются в `/workspace` через `docker/Dockerfile.real` на базе образа из
предоставленного HSL25 Dockerfile: `nickodema/kobuki:humble-22.04-100625`.
Livox SDK2 уже установлен в нём. Основной `src/` содержит наш автономный стек
и `hsl_real`; они собираются отдельным overlay `/solution` в том же образе.
Реальный launch запускает ровно один драйвер LiDAR. `helm build_real`
собирает только реальный образ и не зависит от симуляционного Autoware. Подробная инструкция: [REAL_ROBOT.md](../docs/REAL_ROBOT.md).

`udev/30-kobuki.rules` создаёт `/dev/kobuki`. IP и mounting из комплекта HSL25
вынесены в `config/livox_mid360.json` и `config/real.yaml`, а не зашиты в launch.
Остальные примеры HSL25, старые Docker-скрипты, teleop/mux, docking и материалы
соревнования не являются частью запуска. Исходной папки в корне проекта нет.
