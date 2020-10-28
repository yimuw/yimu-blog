from abc import ABC, abstractmethod
from collections import deque
import json
import visitor_ref
RefObj = visitor_ref.RefObj


class JsonDumper(visitor_ref.VisitorBase):
    def __init__(self):
        self.result = ''
        self.level = 0
        self.indent = 2

    def on_leaf(self, name, obj):
        obj_str = str(obj.val) if not isinstance(
            obj.val, str) else '"{}"'.format(obj.val)
        if name == 'no-name':
            self.result += ' ' * self.level + obj_str + ',\n'
        else:
            self.result += ' ' * self.level + '"' + name + '"' + ':' + obj_str + ',\n'

    def on_enter_level(self, name):
        if name == 'no-name':
            self.result += ' ' * self.level + '{\n'
        else:
            self.result += ' ' * self.level + '"' + name + '"' + ':' + '{\n'
        self.level += self.indent

    def on_leave_level(self):
        self.level -= self.indent
        self.result += ' ' * self.level + '}\n'

    def on_enter_list(self, name, obj):
        if name == 'no-name':
            self.result += ' ' * self.level + '[ \n'
        else:
            self.result += ' ' * self.level + '"' + name + '"' + ':' + '[ \n'
        self.level += self.indent

    def on_leave_list(self):
        # remove the last ",".
        self.result = self.result[:-2] + '\n'
        self.level -= self.indent
        self.result += ' ' * self.level + ']\n'


class JsonLoader(visitor_ref.VisitorBase):
    def __init__(self, dumped):
        self.dumped = deque(dumped.split('\n'))

    def on_leaf(self, name, obj):
        line = self.dumped.popleft()
        line = line.strip().rstrip(',')
        value_str = line
        if ':' in line:
            _, value_str = line.split(':')

        if value_str[0] == '"':
            obj.val = value_str.strip('"')
        else:
            obj.val = float(value_str)

    def on_enter_level(self, name):
        self.dumped.popleft()

    def on_leave_level(self):
        self.dumped.popleft()

    def on_enter_list(self, name, obj):
        cur = self.dumped.popleft()

        def indent(line):
            return len(line) - len(line.lstrip())
        list_start_indent = indent(cur)
        idx = 0
        while indent(self.dumped[idx]) != list_start_indent:
            idx += 1
        list_size = idx

        # need to operate on the list object
        obj.clear()
        for _ in range(list_size):
            obj.append(visitor_ref.RefObj(None))

    def on_leave_list(self):
        self.dumped.popleft()

