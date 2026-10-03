# linefit_ground_segmentation_ros2

Копия https://github.com/baiyeweiguang/linefit_ground_segmentation_ros2,
коммит 5b8a5ae (ROS 2-порт https://github.com/lorenwel/linefit_ground_segmentation,
BSD-3, см. LICENSE). Код не менялся.

В main нет подмодулей, поэтому пакет лежит копией: его собирает обычный
`colcon build` образа.

Поворот в `gravity_aligned_frame` в ноде этого порта не используется:
`Eigen::Affine3d tf;` там не инициализирован. Облако уже приходит в
`base_footprint` от `jr_perception/cloud_prefilter.py`, и `sensor_height: 0`.
