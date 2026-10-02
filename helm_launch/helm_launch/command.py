#!/usr/bin/env python

"""Модуль парсинга команд"""

import os
import re

import parameters
import misc
import resolvers

from typing import Callable, List, Dict


# Скрипты helm_launch, которые можно вызывать из команд в launch.yaml.
# В .yaml они пишутся как __<имя>_script__ и подменяются на реальный путь --
# он зависит от способа установки (локально или глобально через pip).
SCRIPTS = {}


def resolve_scripts(shell_command: str) -> str:
    """Подставить пути к скриптам helm_launch

    :shell_command команда из launch.yaml

    :return команда с абсолютными путями к скриптам
    """
    package_dir = os.path.dirname(os.path.abspath(__file__))

    for placeholder, script in SCRIPTS.items():
        if placeholder in shell_command:
            path = os.path.join(package_dir, script)
            shell_command = shell_command.replace(placeholder, path)

    return shell_command


class Command:
    """Класс для хранения информации о команде"""

    def __init__(self):
        self.__name: str = ""
        self.__description: str = ""
        self.__aliases: List[str] = []
        self.__parameters: List[List[str]] = []
        self.__parameters_templates: List[str] = []
        self.__commands: Callable = []
        self.__shell_commands: List[str] = []
        self.__long_command: bool = False
        self.__optional_tail: bool = False
        self.__resolvers: List[Dict[str, str]] = []

    def __eq__(self, o):
        if not isinstance(o, Command):
            return False

        return (
            self.__name == o.__name
            and self.__aliases == o.__aliases
            and self.__parameters == o.__parameters
        )

    def __ne__(self, o):
        return not self.__eq__(o)
    
    def __repr__(self):
        out = ''
        out += '=' * 10 + '\n'
        out += f'Command\n'
        out += f'name: {self.name}\n'
        out += f'description: {self.description}\n'
        out += f'aliases: {self.aliases}\n'
        out += f'parameters_templates: {self.parameters_templates}\n'
        out += f'parameters: {self.parameters}\n'
        out += f'long_command: {self.long_command}\n'
        out += f'optional_tail: {self.optional_tail}\n'
        out += f'shell_commands: {self.shell_commands}\n'
        out += f'resolvers: {self.resolvers}\n'
        out += '=' * 10

        return out

    @property
    def name(self):
        return self.__name

    @property
    def description(self):
        return self.__description

    @property
    def aliases(self):
        return self.__aliases

    @property
    def parameters(self):
        return self.__parameters
    
    @property
    def parameters_templates(self):
        return self.__parameters_templates
    
    @property
    def long_command(self):
        return self.__long_command

    @property
    def optional_tail(self):
        return self.__optional_tail

    @property
    def commands(self):
        return self.__commands
    
    @property
    def resolvers(self):
        return self.__resolvers

    @property
    def shell_commands(self):
        return self.__shell_commands

    @name.setter
    def name(self, x: str):
        self.__name = x

    @description.setter
    def description(self, x: str):
        self.__description = x

    @aliases.setter
    def aliases(self, x: list):
        self.__aliases = x

    @parameters_templates.setter
    def parameters_templates(self, x: list):
        self.__parameters_templates = x

    @parameters.setter
    def parameters(self, x: list):
        self.__parameters = x

    @long_command.setter
    def long_command(self, x: list):
        self.__long_command = x

    @optional_tail.setter
    def optional_tail(self, x: bool):
        self.__optional_tail = x

    @commands.setter
    def commands(self, x: list):
        self.__commands = x

    @shell_commands.setter
    def shell_commands(self, x: list):
        self.__shell_commands = x

    @resolvers.setter
    def resolvers(self, x: list):
        self.__resolvers = x

    def execute(self, *x):
        """Выполнить команду

        :return код возврата последней неуспешной подкоманды (0 если все ок)
        """
        codes = [command(*x) for command in self.__commands]

        # берем только настоящие коды возврата: callback может вернуть и не
        # число (например, если misc.execute подменен)
        failed = [c for c in codes if isinstance(c, int) and c != 0]

        return failed[-1] if failed else 0


def create_callback(shell_command: str, resolvers_dict: dict | None = None) -> Callable:
    """Создать callback для выполнения команды

    :shell_command команда

    :resolvers_dict словарь типа {__placeholder_i__ -> resolver}

    :return функция
    """

    def function(*x, shell_command=shell_command):
        shell_command = resolve_scripts(shell_command)

        # resolve placeholders
        for i in range(len(x)):
            placeholder_name = f"__placeholder_{i+1}__"
            placeholder_value = x[i]
            if resolvers_dict and placeholder_name in resolvers_dict.keys():
                resolver_name = resolvers_dict[placeholder_name]
                if not hasattr(resolvers, resolver_name):
                    print(f"Резолвер '{resolver_name}' не найден в файле {resolvers.__file__}")
                    exit(1)
                resolver = getattr(resolvers, resolver_name)
                placeholder_value = resolver(x[i])

            shell_command = shell_command.replace(placeholder_name, str(placeholder_value))

        # resolve placeholders without arguments
        # разрешить placeholders без аргументов (просто убираем)
        shell_command = re.sub(r"__placeholder_\d__", "", shell_command)
        # TODO: выводить warning если есть placholder без параметра

        return misc.execute(shell_command)

    return function


def parse_commands(compose_config: dict, run_config: dict) -> List[Command]:
    """Распарсить команды

    :compose_config конфигурация docker compose
    :run_config конфигурация запуска

    :return список команд
    """
    commands = []

    for command, command_dict in run_config.items():
        c = Command()
        c.name = command
        commands.append(c)

        if "desc" in command_dict:
            c.description = command_dict["desc"]

        # TODO: добавить возможность указывать алиас в одну строку (без списка)
        if "aliases" in command_dict:
            c.aliases = command_dict["aliases"]

        if "parameters" in command_dict:
            # если введен не список параметров, то сделаем его искуственно
            # TODO: вынести в misc
            params = command_dict["parameters"]
            if not isinstance(params, list):
                params = [params]

            c.parameters_templates = params

            for parameter in params:
                choices = parameters.get_parameters(compose_config, parameter)
                c.parameters.append(choices)

        if "long_command" in command_dict:
            c.long_command = command_dict["long_command"]

        if "optional_tail" in command_dict:
            c.optional_tail = command_dict["optional_tail"]

        if "commands" in command_dict:
            # если введен не список команд, то сделаем его искуственно
            # TODO: вынести в misc
            shell_commands = command_dict["commands"]
            if not isinstance(shell_commands, list):
                shell_commands = [shell_commands]

            resolvers_dicts = command_dict.get("resolvers", {})

            for i, shell_command in enumerate(shell_commands):
                resolvers_dict = resolvers_dicts.get(i + 1)
                c.resolvers.append(resolvers_dict)
                c.shell_commands.append(shell_command)
                c.commands.append(create_callback(shell_command, resolvers_dict))

    return commands
