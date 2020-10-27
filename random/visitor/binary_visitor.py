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


if __name__ == "__main__":
    # dump the body of b
    b1 = TypeB()
    jsonDump = BinDumper()
    jsonDump.visit('no-name', b1)
    print(jsonDump.result)

    # b2 with no values
    b2 = TypeB()
    b2.b1 = RefObj(None)
    b2.instance_of_A.a = RefObj(None)
    b2.instance_of_A.b = RefObj(None)
    b2.instance_of_A.c = []

    # deserialize b dump into b2
    loader = BinLoader(jsonDump.result)
    loader.visit('no-name', b2)

    # check the dump of b2
    jsonDump = BinDumper()
    jsonDump.visit('no-name', b2)
    print(jsonDump.result)
