#!/usr/bin/env python3
"""Choose a real bag and replay its clouds with the current Anton detector in RViz.

The isolated container builds jr_perception from this checkout. It starts only
the cached-cloud publisher, detector, and RViz. Source bags are read-only.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
from types import SimpleNamespace
import uuid


REPOSITORY = Path(__file__).resolve().parents[1]
DEFAULT_BAG_ROOT = REPOSITORY.parent / 'hsl2026Extra'
CACHE_ROOTS = ('real-detector-cache-v1', 'real-detector-cache-local-v1',
               'real-detector-replay-cache')
DESCRIPTIONS = {
    '20261003T093554': 'подъезд к статичному роботу без коробок',
    '20261003T094119': 'движение среди коробок, без робота',
    '20261003T094252': 'движение среди коробок со статичным роботом',
    '20261003T095154': 'подъезд к статичному роботу без коробок',
    '20261003T095433': 'вращение среди коробок',
    '20261003T095724': 'наш робот стоит, соперник подъезжает',
    '20261003T095806': 'наш робот стоит, соперник сбивает коробку',
    '20261003T095829': 'наш робот стоит, соперник проезжает мимо коробки',
}


def discover_bags(root: Path) -> list[Path]:
    return sorted({p.parent.resolve() for p in root.rglob('metadata.yaml')
                   if p.parent.name == 'bag' and any(p.parent.glob('*.mcap'))})


def label(bag: Path) -> str:
    description = DESCRIPTIONS.get(bag.parent.name[:15], '')
    return bag.parent.name + (f' — {description}' if description else '')


def select_bag(value: str | None, bags: list[Path], choose=input) -> Path:
    if value:
        candidate = Path(value).expanduser().resolve()
        if (candidate / 'bag' / 'metadata.yaml').is_file():
            candidate /= 'bag'
        if (candidate / 'metadata.yaml').is_file() or (candidate / 'manifest.json').is_file():
            return candidate
        matches = [bag for bag in bags if bag.parent.name == value]
        if len(matches) == 1:
            return matches[0]
        raise ValueError(f'Bag не найден: {value}. Укажите папку с metadata.yaml или имя записи.')
    if not bags:
        raise ValueError('Записи не найдены. Укажите --bag PATH или --bag-root PATH.')
    for index, bag in enumerate(bags, 1):
        print(f'{index:2}. {label(bag)}')
    while True:
        answer = choose('Номер записи (q — выход): ').strip()
        if answer.lower() in ('q', 'quit', 'exit'):
            raise KeyboardInterrupt
        if answer.isdigit() and 1 <= int(answer) <= len(bags):
            return bags[int(answer) - 1]
        print(f'Введите число от 1 до {len(bags)}.')


def current_processing(manifest):
    """Validate every recorded preprocessing source, including the filter."""
    for item in manifest['processing_sources']:
        path = (REPOSITORY / item['path']).resolve()
        if not path.is_relative_to(REPOSITORY) or not path.is_file():
            return False
        if hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
            return False
    return True


def verify_cache(cache: Path, manifest, sha256):
    if manifest.get('truncated') or manifest.get('cloud_mode') not in ('both', 'filtered'):
        raise ValueError(f'Нужен полный cache с filtered clouds: {cache}')
    if not manifest.get('counts', {}).get('clouds', {}).get('filtered', 0):
        raise ValueError(f'Нет очищенных сканов: {cache}')
    if not current_processing(manifest):
        raise ValueError(f'Cache создан другим препроцессором; выберите исходный bag: {cache}')
    for artifact in manifest['artifacts']:
        path = cache / artifact['name']
        if not path.is_file() or path.stat().st_size != artifact['bytes'] \
                or sha256(path) != artifact['sha256']:
            raise ValueError(f'Повреждён cache: {path}')


def prepare_cache(selected: Path) -> Path:
    """Run inside ROS image; never publish messages while extracting."""
    sys.path.insert(0, str(REPOSITORY / 'benchmarks'))
    import real_detector_cache as base
    import real_detector_local_cache as local

    if (selected / 'manifest.json').is_file():
        manifest = json.loads((selected / 'manifest.json').read_text())
        verify_cache(selected, manifest, base.sha256)
        original = Path(manifest['bag_dir'])
        if original.is_dir():
            sources = (local.input_hashes(original) if manifest['schema'] ==
                       'real-detector-cache-local-v1' else base.source_hashes(original))[0]
            if sources != manifest['sources']:
                raise ValueError('Исходный bag изменился; выберите его для обновления cache.')
        return selected

    if selected.name != 'bag' or not (selected / 'metadata.yaml').is_file():
        raise ValueError('Ожидается обычная папка SESSION/bag с MCAP и metadata.yaml.')
    reader, topics = base.open_reader(selected, {'/map', '/livox/lidar',
                                                '/sensing/lidar/points_filtered'})
    del reader
    if '/map' in topics:
        extractor = base
        sources, processing = base.source_hashes(selected)
        signature_data = dict(sources=sources, processing=processing,
                              cloud_mode='both', limit_clouds=None)
    else:
        extractor = local
        sources, processing = local.input_hashes(selected)
        signature_data = dict(sources=sources, processing=processing,
                              filter_parameters=local.filter_parameters(),
                              pose_policy='strict_recorded_odom_local_v1')
        print('В bag нет /map: replay в локальной системе odom, без вычитания карты.', flush=True)
    signature = hashlib.sha256(json.dumps(signature_data, sort_keys=True).encode()).hexdigest()
    for root in CACHE_ROOTS:
        for path in (REPOSITORY / 'results' / root).rglob('manifest.json'):
            if path.parent.name != selected.parent.name:
                continue
            manifest = json.loads(path.read_text())
            if manifest.get('signature') == signature:
                print(f'Проверяю существующий cache: {path.parent}', flush=True)
                verify_cache(path.parent, manifest, base.sha256)
                return path.parent
    output = REPOSITORY / 'results/real-detector-replay-cache' / signature[:16]
    output.mkdir(parents=True, exist_ok=True)
    extractor.extract(selected.parent.name, SimpleNamespace(
        root=selected.parent.parent, output=output, cloud_mode='both', limit_clouds=None))
    cache = output / selected.parent.name
    verify_cache(cache, json.loads((cache / 'manifest.json').read_text()), base.sha256)
    return cache


def container_command(args, selected: Path, run_name: str, output: Path) -> list[str]:
    repo_container = Path('/replay')
    selected_container = (repo_container / selected.relative_to(REPOSITORY)
                          if selected.is_relative_to(REPOSITORY) else selected)
    output_container = repo_container / output.relative_to(REPOSITORY)
    command = ['docker', 'run', '--rm', '--init', '--name', run_name,
               '--stop-signal', 'SIGINT', '--network', 'host',
               '-e', f'ROS_DOMAIN_ID={args.ros_domain_id}', '-e', 'ROS_LOCALHOST_ONLY=1',
               '-e', 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp',
               '--mount', f'type=bind,src={REPOSITORY},dst=/replay,readonly',
               '--mount', f'type=bind,src={REPOSITORY / "results"},dst=/replay/results']
    if not selected.is_relative_to(REPOSITORY):
        # The entire chosen session is read-only; do not expose /dev or hardware.
        source_root = selected.parent if (selected / 'manifest.json').is_file() else selected.parent.parent
        command += ['--mount', f'type=bind,src={source_root},dst={source_root},readonly']
    if not args.headless:
        authority = Path(os.environ.get('XAUTHORITY', '')).expanduser()
        if not os.environ.get('DISPLAY') or not authority.is_file():
            raise ValueError('Для RViz нужны DISPLAY и XAUTHORITY; используйте графическую сессию или --headless.')
        command += ['-e', f'DISPLAY={os.environ["DISPLAY"]}', '-e', 'XAUTHORITY=/tmp/replay.xauthority',
                    '--mount', 'type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly',
                    '--mount', f'type=bind,src={authority.resolve()},dst=/tmp/replay.xauthority,readonly']
    cache_prepare = ['python3', '/replay/tools/replay_real_detector.py', '--prepare-cache',
                     str(selected_container), '--cache-result', '/tmp/replay-cache-path']
    replay = ['python3', '/replay/benchmarks/replay_anton_ros.py']
    replay_tail = ['--profile', '/replay/src/jr_perception/config/anton_real.yaml',
                   '--crop-profile', '/replay/src/jr_perception/config/anton_crop.yaml',
                   '--dbscan-profile', '/replay/src/dbscan_filter/config/dbscan.param.yaml',
                   '--count', '0', '--expect', 'any', '--rate', str(args.rate),
                   '--from-seconds', str(args.from_seconds),
                   '--log', str(output_container / 'detector.log')]
    if (selected / 'metadata.yaml').is_file():
        replay_tail += ['--source-bag', str(selected_container)]
    if not args.headless:
        replay_tail += ['--rviz-config', '/replay/benchmarks/anton_detector.rviz',
                        '--rviz-log', str(output_container / 'rviz.log'), '--hold-seconds', '-1']
    script = '\n'.join([
        'set -e',
        'source /solution/install/setup.bash',
        shlex.join(cache_prepare),
        'CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base /tmp/detector-replay/log build --base-paths /replay/src/jr_perception /replay/src/robot_body_filter /replay/src/dbscan_filter '
        '--packages-select jr_perception robot_body_filter dbscan_filter --parallel-workers 1 --build-base /tmp/detector-replay/build '
        '--install-base /tmp/detector-replay/install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF',
        'source /tmp/detector-replay/install/setup.bash',
        'IFS= read -r replay_cache < /tmp/replay-cache-path',
        'exec ' + shlex.join(replay) + ' "$replay_cache" ' + shlex.join(replay_tail),
    ])
    return command + [args.image, 'bash', '-c', script]


def parser():
    result = argparse.ArgumentParser(description='Выбор real bag, детектор Антона и обычный RViz.')
    result.add_argument('--bag', help='Папка SESSION/bag, папка с manifest.json или имя записи')
    result.add_argument('--bag-root', type=Path, default=DEFAULT_BAG_ROOT,
                        help=f'Где искать записи (по умолчанию {DEFAULT_BAG_ROOT})')
    result.add_argument('--rate', type=float, default=1., help='Скорость воспроизведения, по умолчанию 1')
    result.add_argument('--from-seconds', type=float, default=0., help='Начать с указанной секунды записи')
    result.add_argument('--headless', action='store_true', help='Воспроизвести без RViz')
    result.add_argument('--list', action='store_true', help='Показать записи и выйти')
    result.add_argument('--dry-run', action='store_true', help='Показать Docker-команду без запуска')
    result.add_argument('--image', default='jr_real_image:latest', help='Образ ROS 2 Humble')
    result.add_argument('--ros-domain-id', type=int, default=100 + os.getpid() % 100,
                        help='Изолированный ROS domain (1–232, кроме hardware domain 26)')
    result.add_argument('--prepare-cache', type=Path, help=argparse.SUPPRESS)
    result.add_argument('--cache-result', type=Path, help=argparse.SUPPRESS)
    return result


def main(argv=None):
    options = parser()
    args = options.parse_args(argv)
    if args.prepare_cache:
        args.cache_result.write_text(str(prepare_cache(args.prepare_cache.resolve())) + '\n')
        return 0
    if not math.isfinite(args.rate) or args.rate <= 0:
        options.error('--rate должен быть конечным положительным числом')
    if not math.isfinite(args.from_seconds) or args.from_seconds < 0:
        options.error('--from-seconds должен быть конечным неотрицательным числом')
    if not 1 <= args.ros_domain_id <= 232 or args.ros_domain_id == 26:
        options.error('--ros-domain-id должен быть 1–232 и отличаться от hardware domain 26')
    bags = discover_bags(args.bag_root.expanduser())
    if args.list:
        for index, bag in enumerate(bags, 1):
            print(f'{index:2}. {label(bag)}\n    {bag}')
        return 0
    try:
        if not args.bag and not sys.stdin.isatty():
            raise ValueError('Для выбора нужен терминал; укажите --bag PATH (список: --list).')
        selected = select_bag(args.bag, bags)
        run_name = 'hsl-detector-replay-' + uuid.uuid4().hex[:12]
        timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        output = REPOSITORY / 'results/real-detector-replay' / (timestamp + '-' + run_name[-12:])
        command = container_command(args, selected, run_name, output)
        if args.dry_run:
            print(shlex.join(command))
            return 0
        output.mkdir(parents=True)
        print(f'Запись: {label(selected)}\nЛоги: {output}\nОстановка: Ctrl+C или закрыть RViz.', flush=True)
        process = subprocess.Popen(command)
        try:
            return process.wait()
        except KeyboardInterrupt:
            subprocess.run(['docker', 'stop', '--time', '10', run_name],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
            process.wait()
            return 130
    except KeyboardInterrupt:
        return 130
    except (ValueError, OSError) as error:
        print(f'Ошибка: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
