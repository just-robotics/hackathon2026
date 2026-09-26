#!/usr/bin/env python

"""Подключение git-подмодулей с ROS-пакетами в src/

Подмодули описаны в docker/submodules.yaml. Команда идемпотентна: если
подмодуль уже подключен, он не добавляется повторно, а доподтягивается
(update --init), поэтому одной и той же командой закрываются оба случая --
первое подключение и свежий клон репозитория.
"""

import os
import subprocess
import sys

import misc


SUBMODULES_FILE = "submodules.yaml"


def repo_root() -> str:
    """Найти корень репозитория

    helm выполняет команды из docker/, поэтому корень ищем через git, а не
    относительно текущей директории.

    :return абсолютный путь к корню репозитория
    """
    proc = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )

    root = proc.stdout.decode().strip()

    if proc.returncode != 0 or not root:
        print("Не найден git-репозиторий: запустите команду внутри hackathon2026")
        exit(1)

    return root


def read_config(docker_dir: str) -> dict:
    """Прочитать описание подмодулей

    :docker_dir директория с submodules.yaml

    :return {имя -> {url, branch, path, packages}}
    """
    path = os.path.join(docker_dir, SUBMODULES_FILE)

    if not os.path.exists(path):
        print(f"Не найден файл {path}")
        exit(1)

    config = misc.read_yaml(path)
    submodules = config.get("submodules", {})

    if not submodules:
        print(f"В {path} не описано ни одного подмодуля")
        exit(1)

    return submodules


def is_registered(root: str, path: str) -> bool:
    """Проверить, зарегистрирован ли подмодуль в .gitmodules

    Смотрим именно .gitmodules, а не наличие каталога: после свежего клона
    каталог существует, но пустой, и добавлять подмодуль заново нельзя.

    :root корень репозитория
    :path путь подмодуля относительно корня

    :return True, если подмодуль уже описан
    """
    proc = subprocess.run(
        ["git", "config", "--file", ".gitmodules", "--get-regexp", r"^submodule\..*\.path$"],
        cwd=root,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )

    registered = [line.split(" ", 1)[-1] for line in proc.stdout.decode().splitlines()]
    return path in registered


def run(args: list, cwd: str) -> None:
    """Выполнить команду, прервав работу при ошибке

    :args команда
    :cwd рабочая директория
    """
    print("+", " ".join(args))
    ret = subprocess.run(args, cwd=cwd).returncode

    if ret != 0:
        print(f"Команда завершилась с кодом {ret}")
        exit(1)


def add(root: str, name: str, spec: dict) -> None:
    """Подключить один подмодуль

    :root корень репозитория
    :name имя подмодуля
    :spec описание из submodules.yaml
    """
    path = spec["path"]
    url = spec["url"]
    branch = spec.get("branch")

    if is_registered(root, path):
        print(f"[{name}] уже подключен в {path}, обновляю")
    else:
        print(f"[{name}] подключаю {url} в {path}")
        cmd = ["git", "submodule", "add"]
        if branch:
            cmd += ["-b", branch]
        cmd += [url, path]
        run(cmd, root)

    run(["git", "submodule", "update", "--init", "--recursive", path], root)


def print_packages(root: str, submodules: dict) -> None:
    """Напомнить, какие пакеты собираются из подмодулей

    В корне подмодуля лежит COLCON_IGNORE, поэтому автообход colcon его
    пропускает и пакеты собираются явными путями -- они же прописаны в
    Dockerfile.

    :root корень репозитория
    :submodules описание подмодулей
    """
    packages = []
    for spec in submodules.values():
        packages.extend(spec.get("packages", []))

    if not packages:
        return

    print("\nПакеты, собираемые из подмодулей:")
    for package in packages:
        print(f"  {package}")
    print("\nПересоберите образ, чтобы пакеты попали внутрь: helm build gazebo")


def main() -> int:
    """Точка входа команды helm submodules"""
    docker_dir = os.getcwd()  # helm уже перешел в docker/
    root = repo_root()
    submodules = read_config(docker_dir)

    names = sys.argv[1:]

    for name in names:
        if name not in submodules:
            known = ", ".join(sorted(submodules))
            print(f"Неизвестный подмодуль: {name} (известные: {known})")
            return 1

    if not names:
        names = list(submodules)

    for name in names:
        add(root, name, submodules[name])

    print_packages(root, {name: submodules[name] for name in names})

    return 0


if __name__ == "__main__":
    exit(main())
