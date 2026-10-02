#!/usr/bin/env python

"""Вспомогательные функции"""

import os
import shutil
import yaml


def read_yaml(path: str) -> dict:
    """Прочитать yaml файл

    :path путь к файлу
    """
    with open(path) as f:
        try:
            return yaml.safe_load(f)
        except yaml.YAMLError as ex:
            print(ex)

    return {}


def read_env_file(path: str) -> dict:
    """Прочитать .env файл

    :path путь к файлу
    """
    result = {}

    if not os.path.exists(path):
        return result

    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, value = line.partition("=")
            value = value.split("#", 1)[0].strip().strip('"').strip("'")
            result[key.strip()] = value

    return result


def resolve_vehicle_id(use_simulation_str: str) -> str:
    """Определить идентификатор робота

    В симуляции реального робота нет, поэтому идентификатор всегда simulator.
    Иначе берем значение из окружения хоста (определено в ~/.bashrc) -- оно же
    попадает в контейнер через common.yaml.

    :use_simulation_str значение USE_SIMULATION из .env

    :return идентификатор робота
    """
    SIMULATION_VEHICLE_ID = 'simulator'
    
    if use_simulation_str == "true":
        return SIMULATION_VEHICLE_ID

    if use_simulation_str != "false":
        raise ValueError(f"USE_SIMULATION: unexpected value {use_simulation_str}")

    vehicle_id = os.environ.get("VEHICLE_ID", "").strip()

    if not vehicle_id:
        print("Переменная окружения VEHICLE_ID не задана (обычно определена в ~/.bashrc)")
        exit(1)

    return vehicle_id


def prepare_xauthority() -> str:
    """Скопировать cookie X-сервера в стабильный путь

    На Wayland (GNOME) Mutter выдает cookie как /run/user/1000/
    .mutter-Xwaylandauth.XXXXXX и перевыпускает его со новым случайным
    суффиксом при каждом старте сессии. Путь из окружения попадает в конфиг
    контейнера, и после перелогина docker монтирует уже несуществующий файл --
    создает на его месте каталог и падает с "not a directory".

    Поэтому cookie копируется в фиксированный путь, который и монтируется.

    :return путь к копии cookie (или пустая строка, если cookie нет)
    """
    xauthority = os.environ.get("XAUTHORITY", "").strip()

    if not xauthority or not os.path.isfile(xauthority):
        # нет графической сессии (ssh, headless) -- GUI все равно не нужен
        return ""

    stable_dir = os.path.expanduser("~/.hackathon2026")
    stable_path = os.path.join(stable_dir, "xauthority")

    os.makedirs(stable_dir, exist_ok=True)
    # start_match вызывает helm повторно уже с XAUTHORITY, указывающим на копию
    if not (os.path.exists(stable_path) and os.path.samefile(xauthority, stable_path)):
        shutil.copyfile(xauthority, stable_path)
    os.chmod(stable_path, 0o600)

    return stable_path


def execute(command: str, work_dir: str = "") -> int:
    """Выполнить команду

    :command команда
    :work_dir рабочая директория

    :return код ошибки
    """
    cwd = os.getcwd()
    if work_dir:
        os.chdir(work_dir)

    # os.system возвращает wait-статус, а не код возврата процесса
    ret = os.waitstatus_to_exitcode(os.system(command))
    os.chdir(cwd)
    return ret
