#!/usr/bin/env python

"""Вспомогательные функции"""

import os
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


def execute(command: str, work_dir: str = "") -> int:
    """Выполнить команду

    :command команда
    :work_dir рабочая директория

    :return код ошибки
    """
    cwd = os.getcwd()
    if work_dir:
        os.chdir(work_dir)

    ret = os.system(command)
    os.chdir(cwd)
    return ret
