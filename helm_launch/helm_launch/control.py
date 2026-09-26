#!/usr/bin/env python

"""Запуск и остановка движения робота

Продольный контур MPC стартует с параметром start=false и не двигает робота,
пока его не выставят -- это страховка от самопроизвольного старта. Имя ноды
зависит от режима (cc или acc), поэтому нода ищется по факту: среди
запущенных берется та, у которой есть параметр start.

Так команда работает и для cc, и для acc, и при lateral:=false, а вместо
тихого "Node not found" (ros2 param set возвращает на нем 0) выдается
внятная ошибка.
"""

import subprocess
import sys


SERVICE = "control"
NAMESPACE = "/control/"
PARAMETER = "start"

# ros2 в контейнере доступен только после source
ROS_PREFIX = "source /autoware/install/setup.bash && "


def ros2(args: str, timeout: int = 60) -> str:
    """Выполнить ros2-команду внутри сервиса control

    :args аргументы ros2
    :timeout таймаут в секундах

    :return stdout команды (пустая строка при ошибке)
    """
    command = [
        "docker", "compose", "exec", "-T", SERVICE,
        "bash", "-c", f"{ROS_PREFIX}ros2 {args}",
    ]

    try:
        proc = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        print(f"ros2 {args}: превышен таймаут {timeout} с")
        exit(1)

    return proc.stdout.decode()


def find_node() -> str:
    """Найти продольную ноду -- ту, у которой есть параметр start

    :return полное имя ноды (при отсутствии -- завершаем с ошибкой)
    """
    nodes = [
        node.strip()
        for node in ros2("node list").splitlines()
        if node.strip().startswith(NAMESPACE)
    ]

    if not nodes:
        print("Ноды контроллера не найдены. Запущен ли сервис control?")
        print("  helm up simulation")
        exit(1)

    for node in nodes:
        parameters = ros2(f"param list {node}").split()
        if PARAMETER in parameters:
            return node

    print(f"Среди нод {', '.join(nodes)} нет параметра {PARAMETER}")
    print("Проверьте, что поднят продольный контур (MPC_LONGITUDINAL в .env)")
    exit(1)


def main() -> int:
    """Точка входа команд helm start и helm stop"""
    if len(sys.argv) < 2 or sys.argv[1] not in ("true", "false"):
        print("Использование: control.py true|false")
        return 1

    value = sys.argv[1]
    node = find_node()

    print(f"{node}: {PARAMETER} -> {value}")
    output = ros2(f"param set {node} {PARAMETER} {value}").strip()
    print(output)

    # ros2 param set возвращает 0 даже когда ничего не сделал,
    # поэтому ориентируемся на текст
    if "Set parameter successful" not in output:
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
