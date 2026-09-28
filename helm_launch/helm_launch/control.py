#!/usr/bin/env python

"""Запуск и остановка движения робота

Продольный контур MPC стартует с параметром start=false и не двигает робота,
пока его не выставят -- это страховка от самопроизвольного старта. Имя ноды
зависит от режима (cc или acc), поэтому нода ищется по факту: среди
запущенных берется та, у которой есть параметр start.

Так команда работает и для cc, и для acc, и при lateral:=false, а вместо
тихого "Node not found" (ros2 param set возвращает на нем 0) выдается
внятная ошибка.

С двумя роботами контроллеров несколько, у каждого свой сервис и свое
пространство имен. Без аргумента команда действует на всех найденных, а имя
робота ограничивает ее одним: `helm start defender`.
"""

import subprocess
import sys


PARAMETER = "start"

# Сервис и пространство имен нод для каждого режима. Одиночный робот держит
# контроллер в сервисе control без префикса, дуэль -- по сервису на роль.
TARGETS = {
    "": ("control", "/control/"),
    "defender": ("control_defender", "/defender/control/"),
    "attacker": ("control_attacker", "/attacker/control/"),
}

ROBOTS = [name for name in TARGETS if name]

# ros2 в контейнере доступен только после source
ROS_PREFIX = "source /autoware/install/setup.bash && "


def ros2(service: str, args: str, timeout: int = 60) -> str:
    """Выполнить ros2-команду внутри сервиса

    :service имя сервиса compose
    :args аргументы ros2
    :timeout таймаут в секундах

    :return stdout команды (пустая строка при ошибке)
    """
    command = [
        "docker", "compose", "exec", "-T", service,
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


def find_node(service: str, namespace: str) -> str:
    """Найти продольную ноду -- ту, у которой есть параметр start

    :service имя сервиса compose
    :namespace пространство имен нод контроллера

    :return полное имя ноды, либо пустая строка если сервис не отвечает
    """
    nodes = [
        node.strip()
        for node in ros2(service, "node list").splitlines()
        if node.strip().startswith(namespace)
    ]

    for node in nodes:
        parameters = ros2(service, f"param list {node}").split()
        if PARAMETER in parameters:
            return node

    return ""


def apply(robot: str, value: str) -> bool:
    """Выставить параметр start у контроллера одного робота

    :robot имя робота или пустая строка для одиночного режима
    :value true или false

    :return True если параметр установлен
    """
    service, namespace = TARGETS[robot]
    node = find_node(service, namespace)

    if not node:
        return False

    print(f"{node}: {PARAMETER} -> {value}")
    output = ros2(service, f"param set {node} {PARAMETER} {value}").strip()
    print(output)

    # ros2 param set возвращает 0 даже когда ничего не сделал,
    # поэтому ориентируемся на текст
    return "Set parameter successful" in output


def main() -> int:
    """Точка входа команд helm start и helm stop"""
    if len(sys.argv) < 2 or sys.argv[1] not in ("true", "false"):
        print("Использование: control.py true|false [робот]")
        return 1

    value = sys.argv[1]
    robot = sys.argv[2] if len(sys.argv) > 2 else None

    if robot is not None and robot not in TARGETS:
        print(f"Неизвестный робот: {robot}")
        print(f"Доступны: {', '.join(ROBOTS)}")
        return 1

    # без имени робота пробуем все цели: одиночный контроллер и обе роли
    targets = [robot] if robot is not None else [""] + ROBOTS
    done = [name for name in targets if apply(name, value)]

    if not done:
        print("Ноды контроллера не найдены. Запущен ли сервис control?")
        print("  helm up simulation   (один робот)")
        print("  helm up duel         (два робота)")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
