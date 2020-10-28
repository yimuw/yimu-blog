from abc import ABC, abstractmethod
import json
from collections import deque
import visitor_ref
RefObj = visitor_ref.RefObj


class BinDumper(visitor_ref.VisitorBase):
    def __init__(self):
        self.result = ''
        self.seperator = '|'

    def on_leaf(self, name, obj):
        obj_str = str(obj.val) if not isinstance(
            obj.val, str) else '"{}"'.format(obj.val)

        self.result += obj_str + self.seperator

    def on_enter_level(self, name):
        pass

    def on_leave_level(self):
        pass

    def on_enter_list(self, name, obj):
        # save the length of the list
        self.result += str(len(obj)) + self.seperator

    def on_leave_list(self):
        pass


class BinLoader(visitor_ref.VisitorBase):
    def __init__(self, dumped):
        self.seperator = '|'
        self.dumped = deque(dumped.split(self.seperator))

    def on_leaf(self, name, obj):
        value_str = self.dumped.popleft()
        if value_str[0] == '"':
            obj.val = value_str.strip('"')
        else:
            obj.val = float(value_str)

    def on_enter_level(self, name):
        pass

    def on_leave_level(self):
        pass

    def on_enter_list(self, name, obj):
        list_size = int(self.dumped.popleft())
        # need to operate on the list object
        obj.clear()
        for _ in range(list_size):
            obj.append(visitor_ref.RefObj(None))

    def on_leave_list(self):
        pass


