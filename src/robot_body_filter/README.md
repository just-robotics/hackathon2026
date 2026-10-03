# robot_body_filter

Удаляет из лидарного облака точки, попавшие в тело робота, набором кроп-боксов
из конфига. URDF не используется.

Фильтрация разделена на две части:

- **статическая** — корпус и всё, что жёстко закреплено. Бокс задан 9 числами
  в `base_frame`: `origin[3]` + `min[3]` + `max[3]`. Боксов может быть несколько,
  они применяются последовательно.
- **динамическая** — колёса и прочее вращающееся. К тем же 9 числам добавляются
  `axis[3]` (ось вращения в `base_frame`) и `joint_frame` — звено, из TF которого
  берётся угол поворота вокруг этой оси. Бокс поворачивается вокруг `axis`
  в своей точке `origin`.

Последовательность боксов логическая: проход по облаку один, точка отбрасывается
на первом же попадании, промежуточные копии облака не создаются.

Попавшие в боксы точки вырезаются из облака, а не помечаются NaN: выходное
облако неупорядоченное (`height = 1`), `width` равен числу оставшихся точек.

## Статус

Каркас пакета. Объявления интерфейсов на месте, тела методов — заглушки с `TODO`.

## Топики

| Направление | Топик | Тип |
|---|---|---|
| вход | `~/input/pointcloud` | `sensor_msgs/msg/PointCloud2` |
| выход | `~/output/pointcloud` | `sensor_msgs/msg/PointCloud2` |
| выход | `robot_body_filter/body_markers` | `visualization_msgs/msg/MarkerArray` |

Один вход и один выход на ноду; вся цепочка боксов отрабатывает внутри.

## Параметры

| Параметр | Тип | По умолчанию | Описание |
|---|---|---|---|
| `base_frame` | string | `base_link` | Кадр, в котором заданы все боксы |
| `publish_body_markers` | bool | `false` | Публиковать маркеры боксов для RViz |
| `tf_timeout` | double | `0.1` | Ожидание TF, с |

### Описание бокса

| Поле | Тип | Статический | Динамический |
|---|---|---|---|
| `origin` | double[3] | да | да |
| `min` | double[3] | да | да |
| `max` | double[3] | да | да |
| `padding` | double[3] | да | да |
| `axis` | double[3] | — | да |
| `joint_frame` | string | — | да |

```yaml
static_boxes:
  names: [chassis]
  chassis:
    origin: [0.0, 0.0, 0.35]
    min: [-0.80, -0.45, -0.35]
    max: [ 0.80,  0.45,  0.35]
    padding: [0.03, 0.03, 0.03]

dynamic_boxes:
  names: [wheel_front_left]
  wheel_front_left:
    origin: [0.60, 0.40, 0.25]
    min: [-0.28, -0.09, -0.28]
    max: [ 0.28,  0.09,  0.28]
    padding: [0.03, 0.03, 0.03]
    axis: [0.0, 1.0, 0.0]
    joint_frame: wheel_front_left_link
```

## Сборка

```bash
colcon build --packages-select robot_body_filter
```

## Запуск

```bash
ros2 launch robot_body_filter robot_body_filter.launch.xml \
    input/pointcloud:=/sensing/lidar/concatenated/pointcloud
```

## Структура

- `crop_box` — один бокс: габариты, поворот вокруг оси, тест вхождения точки
- `robot_body_filter` — проход по облаку; не зависит от ROS-ноды, позы звеньев получает снаружи
- `robot_body_filter_node` — параметры, TF, подписки/публикации
