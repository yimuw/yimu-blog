from abc import ABC, abstractmethod
import json
from collections import deque


class Vistable(ABC):
    @abstractmethod
    def visit(self):
        pass


class A(Vistable):
    def __init__(self):
        self.a = 1
        self.b = "whatever"
        self.c = [1, 2, 3]

    def visit(self, visitor):
        visitor.visit('a', self.a)
        visitor.visit('b', self.b)
        visitor.visit('c', self.c)


class B(Vistable):
    def __init__(self):
        self.b1 = 100
        self.instance_of_A = A()

    def visit(self, visitor):
        visitor.visit('b1', self.b1)
        visitor.visit('instance_of_A', self.instance_of_A)


class VisitorBase(ABC):
    def __init__(self):
        pass

    @abstractmethod
    def on_leaf(self, name, obj):
        pass

    @abstractmethod
    def on_enter_level(self, name):
        pass

    @abstractmethod
    def on_leave_level(self):
        pass

    @abstractmethod
    def on_enter_list(self, name, obj):
        pass

    @abstractmethod
    def on_leave_list(self):
        pass

    def visit(self, name, obj):
        if isinstance(obj, list):
            self.on_enter_list(name, obj)
            for e in obj:
                self.visit('no-name', e)
            self.on_leave_list()
        elif isinstance(obj, Vistable):
            self.on_enter_level(name)
            obj.visit(self)
            self.on_leave_level()
        else:
            self.on_leaf(name, obj)


class JsonDump(VisitorBase):
    def __init__(self):
        self.result = ''
        self.level = 0
        self.indent = 2

    def on_leaf(self, name, obj):
        obj_str = str(obj) if not isinstance(obj, str) else '"{}"'.format(obj)
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
            self.result += ' ' * self.level + '[\n'
        else:
            self.result += ' ' * self.level + '"' + name + '"' + ':' + '[\n'
        self.level += self.indent

    def on_leave_list(self):
        # remove the last ','
        self.result = self.result[:-2] + '\n'
        self.level -= self.indent
        self.result += ' ' * self.level + ']\n'


class JsonLoad(VisitorBase):
    def __init__(self, dumped):
        self.dumped = deque(dumped.split('\n'))

    def on_leaf(self, name, obj):
        line = self.dumped.popleft()
        line = line.strip().rstrip(',')
        print('line:', line, obj)
        value_str = line
        if ':' in line:
            _, value_str = line.split(':')
            
        if value_str[0] == '"':
            obj = value_str.strip('"')
        else:
            obj = float(value_str)
        print('obj:', obj)


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
        obj = [object] * list_size


    def on_leave_list(self):
        self.dumped.popleft()

if __name__ == "__main__":
    b = B()

    jsonDump = JsonDump()
    jsonDump.visit('no-name', b)
    print(jsonDump.result)

    #json_read = json.loads(jsonDump.result)
    #print(json.dumps(json_read, indent=2))

    loader = JsonLoad(jsonDump.result)

    b2 = B()
    b2.b1 = 1
    b2.instance_of_A.a = "na"
    b2.instance_of_A.b = "na"
    b2.instance_of_A.c = []

    loader.visit('no-name', b2)

    jsonDump = JsonDump()
    jsonDump.visit('no-name', b2)
    print(jsonDump.result)

