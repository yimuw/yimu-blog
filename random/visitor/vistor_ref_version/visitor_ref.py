from abc import ABC, abstractmethod
import json
from collections import deque


class RefObj:
    def __init__(self, val):
        self.val = val


class Vistable(ABC):
    def __init__(self):
        self.version = RefObj(1)

    @abstractmethod
    def visit(self, visitor):
        visitor.visit('version', self.version)


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
        elif isinstance(obj, RefObj):
            self.on_leaf(name, obj)
