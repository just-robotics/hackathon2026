# Оценка непрерывной цели перехвата — 02.10.2026

## Условия

Серия `series-20261001T224501Z`:20 независимых миров, заранее seeds0–19, fixed explorer first, active limit360s до первого события. Native Nav2 MPPI1.1.20, CPP detector height+diameter, own_max_speed0.5, continuous short-goal route. Навигационный source813b515/image44ef99557ed5dfe7cc875222d9fff465489b90ce487362d0c36afe08839e82fa; benchmark revision4165d62. Verified source hash maps одинаковы20/20. Последующие изменения касались offline отчётов и документации.

Старты E[0,0,0],G[2.84,2.1,pi]; max_speed.5/angular1.5; reverse толькоE. Один конфиг maze, без неизвестных fixtures. Seeds13/14 дополнительно наблюдались read-only costmap node; он не публикует управление, добавляет небольшую измерительную нагрузку.

Команда:
```bash
python3 benchmarks/run_duel_series.py --isolated-project hsl-eval --ros-domain-id 73 --gazebo-port 11418 --runs 20 --start-seed 0 --active-s 360 --trace --audit-start
```

## Все матчи

| Seed | Исход | Sim s | Speed E/G m/s | Global lateral RMS E/G m | Angular accel RMS E/G rad/s² | Контакты E/G |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | explorer_goal | 17.7 | 0.387/0.418 | 0.069/0.046 | 0.879/1.444 | 0/0 |
| 1 | explorer_goal | 18.1 | 0.344/0.420 | 0.061/0.043 | 0.721/0.928 | 0/0 |
| 2 | explorer_goal | 18.6 | 0.364/0.411 | 0.076/0.042 | 0.801/1.109 | 0/0 |
| 3 | explorer_goal | 19.4 | 0.377/0.400 | 0.065/0.033 | 0.891/0.859 | 0/0 |
| 4 | explorer_goal | 18.2 | 0.385/0.409 | 0.069/0.042 | 0.762/1.038 | 0/0 |
| 5 | explorer_goal | 20.0 | 0.358/0.388 | 0.090/0.046 | 0.761/1.224 | 0/0 |
| 6 | explorer_goal | 17.8 | 0.366/0.417 | 0.066/0.035 | 0.886/1.124 | 0/0 |
| 7 | explorer_goal | 18.5 | 0.391/0.399 | 0.078/0.039 | 0.783/1.102 | 0/0 |
| 8 | explorer_goal | 18.5 | 0.376/0.413 | 0.062/0.037 | 0.924/0.938 | 0/0 |
| 9 | explorer_goal | 17.1 | 0.372/0.428 | 0.045/0.039 | 0.698/0.759 | 0/0 |
| 10 | explorer_goal | 12.4 | 0.357/0.302 | 0.053/0.044 | 0.825/0.962 | 0/0 |
| 11 | explorer_goal | 16.6 | 0.373/0.438 | 0.051/0.042 | 0.753/0.888 | 0/0 |
| 12 | explorer_goal | 18.7 | 0.375/0.400 | 0.074/0.038 | 0.789/0.965 | 0/0 |
| 13 | explorer_goal | 19.2 | 0.368/0.400 | 0.076/0.038 | 0.889/0.839 | 0/0 |
| 14 | explorer_goal | 18.5 | 0.336/0.409 | 0.070/0.044 | 0.754/1.014 | 0/0 |
| 15 | explorer_goal | 19.5 | 0.366/0.389 | 0.070/0.040 | 0.894/1.206 | 0/0 |
| 16 | explorer_goal | 18.1 | 0.352/0.420 | 0.060/0.046 | 0.743/0.941 | 0/0 |
| 17 | guardian_capture | 14.8 | 0.323/0.433 | 0.057/0.038 | 0.637/1.032 | 0/0 |
| 18 | explorer_goal | 18.4 | 0.384/0.404 | 0.080/0.032 | 0.831/0.860 | 0/0 |
| 19 | explorer_goal | 19.6 | 0.370/0.376 | 0.086/0.035 | 0.856/1.000 | 0/0 |

## Агрегаты

19 целей/1 поимка;0 timeouts/технических ошибок. Это результат наблюдения, seeds не подбирались. Страж — слабая роль, требуется следующий алгоритмический цикл.

| Метрика | Исследователь | Страж |
| --- | --- | --- |
| Средняя индивидуальных средних скоростей | 0.366 | 0.404 |
| Минимальная средняя скорость | 0.323 | 0.302 |
| Ниже0.2/0.3 (соответственно) | 0/0 | 0/0 |
| Global lateral RMS mean | 0.068 | 0.040 |
| Angular accel RMS mean | 0.804 | 1.012 |
| Доля движения mean | 0.964 | 0.974 |
| Planner OK mean | 0.991 | 0.983 |
| Контакты | 0.000 | 0.000 |
| Минимальный static envelope просвет | 0.143 | 0.118 |
| Прямая reference: время s | 111.360 | 148.220 |
| Прямая reference: sign changes/min | 21.013 | 21.455 |

Все40 индивидуальных средних≥0.3m/s, pose_jumps_ignored0. Одинаковые referee windows для обеих ролей,60 парных gate-аудитов пройдены,20 owned runtimes очищены. RTF .545… .623. Каждый cmd_vel имеет одного издателя; до старта/после исхода остановлен.

## Ограничения и следующий шаг

Скорость — за полное активное окно; lateral RMS по дискретным trace observations, не точная непрерывная ошибка. Angular RMS/sign changes — диагностические метрики, не доказательство идеальной плавности. Straight classification/покрытие определены в straight-motion.json. Static clearance: .23-radius envelope против20 SDF wall rectangles на z=.25, по измеренным позам и линейным chords≤.02m; не точный moving footprint и не dynamic obstacle clearance.

Read-only peer costmap audit:54 snapshots seeds13/14,1036 cost254 map-free отметок, но0 дальше.1m от SDF стен. Ghost-peer гипотеза не подтверждена в этих снимках; большее исключение obstacles не добавлять. Геометрический ETA gap в PURSUE есть (например seed1 >=1.08s), но некоторые крайние случаи включают noisy tracks (seed5 error.315m), и причинный выигрыш от нового прогноза пока не доказан.

Ближний lead_time сейчас distance/max_speed, затем резко0 при distance≤.45. Следующий цикл — время до capture radius и непрерывное угасание lead, с прежними скоростями/MPPI/safety; проверить head-on/поперечное движение, границу.45, одинаковые seeds0–2 и затем20.

Проверка относится к этому лабиринту/конфигу. GUI, другие старты/лабиринты, real hardware и неизвестные obstacles текущим20 не проверены. Предыдущие low obstacle tests относятся к более раннему source; потребуется повтор на принятом финальном варианте. Цель целиком не завершена.

Артефакты: `results/isolated/hsl-eval/series-20261001T224501Z/`:index,summary,runtime,gates,traces,evaluation-summary,straight-motion,static-clearance,peer-costmap-summary,final-capture-comparison,progress-motion.png.
