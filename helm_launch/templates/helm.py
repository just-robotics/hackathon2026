#!/usr/bin/env python

"""Основной модуль запуска стека"""

import argcomplete
import os
import sys

# Это для импортов, чтобы либы импортились во всех случаях:
# - локальный запуск скрипта
# - глобальный (установленный через pip) запуск скрипта
if os.path.exists(os.path.abspath("../helm_launch/__init__.py")):
    sys.path.append("../helm_launch")
else:
    try:
        import helm_launch

        sys.path.append(os.path.dirname(helm_launch.__file__))
    except ImportError:
        print("Module helm_launch not found!")
        exit(1)

import arguments
import command
import misc



# PYTHON_ARGCOMPLETE_OK

# Jinja2 параметры, заполняются при установке через setup.py
DOCKER_COMPOSE_DIR = "{{ docker_compose_dir }}"
COMPOSE_FILE = "{{ compose_file }}"
LAUNCH_FILE = "{{ launch_file }}"
ENV_FILE = "{{ env_file }}"

# идентификатор робота, который используется при запуске в симуляции
SIMULATION_VEHICLE_ID = "simulator"


def main():
    os.chdir(DOCKER_COMPOSE_DIR)  # для выполнения compose команд
    compose_config = misc.read_yaml(COMPOSE_FILE)
    run_config = misc.read_yaml(LAUNCH_FILE)
    env_config = misc.read_env_file(ENV_FILE)

    use_splitting_str = env_config["USE_RESOURCES_SPLITTING"]
    use_simulation_str = env_config["USE_SIMULATION"]

    if use_splitting_str == "true":
        # разделение ресурсов задается под конкретное железо, поэтому файл
        # выбирается по реальному идентификатору робота из окружения
        resources_file = f"resources/resources_{os.environ['VEHICLE_ID']}.yaml"
    elif use_splitting_str == "false":
        resources_file = f"resources/resources_no_splitting.yaml"
    else:
        raise ValueError(f"USE_RESOURCES_SPLITTING: unexpected value {use_splitting_str}")

    os.environ["RESOURCES_FILE"] = resources_file
    # значение уходит в контейнер через common.yaml
    os.environ["VEHICLE_ID"] = misc.resolve_vehicle_id(use_simulation_str)

    commands : list= command.parse_commands(compose_config, run_config)

    parser = arguments.register_arguments(commands)
    argcomplete.autocomplete(parser)

    args = parser.parse_args()
    placeholders = arguments.parse_placeholders(args)

    if not args.cmd:
        parser.print_help()
        return 1

    for cmd in commands:
        if args.cmd == cmd.name:
            cmd.execute(*placeholders)

    return 0


if __name__ == "__main__":
    exit(main())
