#!/usr/bin/env python

"""Модуль парсинга аргументов командной строки"""

import argparse
import argcomplete
import re
import shlex
import sys

import command

from typing import List, Union


class HelmArgumentParser(argparse.ArgumentParser):
    """Allow optional script arguments to begin with an option such as --bag."""

    def parse_args(self, args=None, namespace=None):
        values = list(sys.argv[1:] if args is None else args)
        if values and values[0] in getattr(self, "script_commands", set()):
            if len(values) > 1 and values[1] != "--":
                values.insert(1, "--")
        return super().parse_args(values, namespace)


def register_arguments(commands: List[command.Command]) -> argparse.ArgumentParser:
    """Зарегистрировать параметры

    :commands команды

    :return парсер аргументов
    """
    parser = HelmArgumentParser(argument_default=argparse.SUPPRESS)
    parser.script_commands = {
        name
        for cmd in commands
        if cmd.long_command and cmd.optional_tail and not cmd.parameters
        for name in (cmd.name, *cmd.aliases)
    }
    subparser = parser.add_subparsers(dest="cmd")

    # регистрируем команды вроде up, start и т.п.
    for cmd in commands:
        p = subparser.add_parser(
            cmd.name, aliases=cmd.aliases, help=cmd.description, add_help=False
        )
        p.set_defaults(cmd=cmd.name)  # разрешаем alias относительно cmd
        for i in range(len(cmd.parameters)):
            if cmd.name == "commit" and cmd.parameters_templates[i] == "__image__":
                arg = p.add_argument(f"placeholder{i}")
                arg.completer = argcomplete.completers.ChoicesCompleter(cmd.parameters[i])
            else:
                p.add_argument(f"placeholder{i}", choices=cmd.parameters[i])
        if cmd.long_command:
            p.add_argument(f"placeholder{len(cmd.parameters)}", nargs=argparse.REMAINDER)

    return parser


def parse_placeholders(
    args: argparse.Namespace, optional_tail: bool = False
) -> List[Union[int, float, str]]:
    """Распарсить значения placeholders из полученных аргументов

    :args аргументы командной строки
    :optional_tail разрешить пустой последний placeholder. Нужно командам,
        у которых хвост -- необязательный список (например, команда с optional_tail
        без имен подключает все подмодули)

    :return значения для подстановки в placeholders
    """

    # тут ищем значения для параметров вида placeholder0, placeholder1 и т.д.
    items = list(args.__dict__.items())
    placeholders = []
    for i, (placeholder_name, placeholder_value) in enumerate(items):
        if placeholder_name == 'cmd' and placeholder_value is None:
            continue
        is_tail = i == len(items) - 1
        if len(placeholder_value) == 0 and not (optional_tail and is_tail):
            empty_placeholder_name = f"__placeholder_{i}__"
            cmd = args.__dict__['cmd']
            print(f'helm {cmd}: error: empty placeholder: {empty_placeholder_name}')
            exit(1)
        # мы ищем именно placeholders, т.к. в args.__dict__ хранятся еще другие параметры
        if len(re.findall(r"placeholder\d+", placeholder_name)) == 1:
            placeholders.append((placeholder_name, placeholder_value))

    # чтобы быть уверенными, что placeholder0 на индексе 0 и т.д.
    placeholders = sorted(placeholders, key=lambda x: x[0])

    # а после сортировки имена уже не нужны, оставляем только значения
    def tail_value(value):
        if not isinstance(value, list):
            return value
        if optional_tail:
            # Script arguments retain literal spaces, quotes, and shell symbols.
            return shlex.join(value[1:] if value[:1] == ["--"] else value)
        return " ".join(value)

    placeholders = [tail_value(v) for k, v in placeholders]

    return placeholders
