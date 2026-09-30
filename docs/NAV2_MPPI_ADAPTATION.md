# Адаптация локального MPPI из Nav2 Humble

**Текущий статус (30.09): MPPI остаётся локальным генератором пути.**
Проба поставить перед ним пространственную дугу RPP откатана после дуэлей:
она отклонялась на большинстве циклов и не улучшила скорость.
Повторная сверка исходного кода показала принципиальную разницу интерфейсов:
`MPPIController::computeVelocityCommands` возвращает `TwistStamped`, а
`getOptimizedTrajectory()` вызывается для визуализации. Наша передача этой
временной траектории отдельному Lat-MPC не эквивалентна работе Nav2 MPPI.
Прямое следствие, наблюдаемое в трассах дуэлей: частые почти совпадающие точки
повышают оценку кривизны у Lat-MPC и понижают `v_curve`; прореживание точек
улучшило геометрическую метрику, но вызвало контакты со стеной. Пробная дуга
RPP тоже была адаптацией геометрии, а не полным Nav2 контроллером. Сравнение
серий приведено в [PROJECT_STATUS.md](PROJECT_STATUS.md).

Источник сверки: `ros-navigation/navigation2`, ветка `humble`, commit
[`3c3db59d6969d8ecee8e68468693d006397f4a0c`](https://github.com/ros-navigation/navigation2/tree/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_mppi_controller).
Оригинальный пакет и исходники его авторов опубликованы под Apache-2.0.
Лицензия проекта — [Apache-2.0](../LICENSE). См. также заголовки файлов
[optimizer.cpp](https://github.com/ros-navigation/navigation2/blob/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_mppi_controller/src/optimizer.cpp),
[motion_models.hpp](https://github.com/ros-navigation/navigation2/blob/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_mppi_controller/include/nav2_mppi_controller/motion_models.hpp),
[path_align_critic.cpp](https://github.com/ros-navigation/navigation2/blob/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_mppi_controller/src/critics/path_align_critic.cpp),
[path_follow_critic.cpp](https://github.com/ros-navigation/navigation2/blob/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_mppi_controller/src/critics/path_follow_critic.cpp),
[obstacles_critic.cpp](https://github.com/ros-navigation/navigation2/blob/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_mppi_controller/src/critics/obstacles_critic.cpp) и
[noise_generator.cpp](https://github.com/ros-navigation/navigation2/blob/3c3db59d6969d8ecee8e68468693d006397f4a0c/nav2_mppi_controller/src/noise_generator.cpp).

`src/hsl_planning/hsl_planning/mppi.py` переносит метод: шум вокруг последовательности команд,
модель дифференциального привода, оценка множества траекторий, обновление
взвешенным средним и тёплый старт. В текущем адаптере исходные Nav2 critics
заменены стоимостью по карте и LiDAR, расстоянию/продвижению по A* и плавности.
Дополнительно проверяется весь пройденный контур корпуса, а в `hsl_debug_control`
действует шлюз свежести данных и разрешения движения.

Различия, важные для испытаний:

- Nav2 MPPI непосредственно выдаёт `cmd_vel`; здесь оптимизатор публикует
  короткий `nav_msgs/Path`, который затем исполняет отдельный Lat-MPC.
- Горизонт оптимизации и длина опубликованного пути различаются. Префикс
  ограничен по длине; это адаптация к контроллеру, требующая экспериментальной
  проверки длины пути и непрерывности между обновлениями.
- Штраф начальной касательной, возможность выбрать один безопасный образец
  вместо среднего, короткий повторный горизонт и recovery являются местными
  решениями. Их следует сохранять только при улучшении реальных заездов.
- Параметр `max_speed` сейчас приходит из поведения и может превышать скорость
  штатного CC-MPC. Rollout, ограничения и исходную скорость собственного
  контроллера нужно согласовать без добавления временного лимита для исследования.
- ROS `controller_server`, плагин C++ MPPI, Nav2 costmap и набор critics из
  исходного пакета здесь не запускаются. Поэтому полная эквивалентность Nav2
  не заявляется.

Проверенные серии, метрики и открытые дефекты — в [PROJECT_STATUS.md](PROJECT_STATUS.md).
