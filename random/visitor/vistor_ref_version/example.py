from json_visitor import JsonDumper, JsonLoader
from binary_visitor import BinDumper, BinLoader
from visitor_ref import RefObj, Vistable


class TypeA(Vistable):
    def __init__(self):
        super().__init__()
        self.version = RefObj(2)
        
        self.a = RefObj(1)
        self.b = RefObj("whatever")
        self.c = [RefObj(1), RefObj(-1), RefObj(2.2)]

        self.v2_var = RefObj('version 2!')

    def visit(self, visitor):
        super().visit(visitor)
        visitor.visit('a', self.a)
        visitor.visit('b', self.b)
        visitor.visit('c', self.c)

        if self.version.val >= 2:
            visitor.visit('v2_var', self.v2_var)


class TypeB(Vistable):
    def __init__(self):
        super().__init__()
        self.b1 = RefObj(123)
        self.instance_of_A = TypeA()

    def visit(self, visitor):
        super().visit(visitor)
        visitor.visit('b1', self.b1)
        visitor.visit('instance_of_A', self.instance_of_A)


def json_example():
    print("============= binary_example ================")
    # dump the body of b
    b1 = TypeB()
    b1.instance_of_A.version = RefObj(1)
    json_dumper = JsonDumper()
    json_dumper.visit('no-name', b1)
    print('b1 dump:', json_dumper.result, sep='\n')
    b1_dump = json_dumper.result

    # b2 with no values
    b2 = TypeB()
    b2.b1 = RefObj(None)
    b2.instance_of_A.a = RefObj(None)
    b2.instance_of_A.b = RefObj(None)
    b2.instance_of_A.c = []
    json_dumper = JsonDumper()
    json_dumper.visit('no-name', b2)
    print('b2 dump:', json_dumper.result, sep='\n')

    # deserialize b dump into b2
    loader = JsonLoader(b1_dump)
    loader.visit('no-name', b2)
    
    # check the dump of b2
    json_dumper = JsonDumper()
    json_dumper.visit('no-name', b2)
    print('b2 dump after load b1:', json_dumper.result, sep='\n')


def binary_example():
    print("============= binary_example ================")
    # dump the body of b
    b1 = TypeB()
    bin_dumper = BinDumper()
    bin_dumper.visit('no-name', b1)
    print('b1 dump:', bin_dumper.result, sep='\n')
    b1_dump = bin_dumper.result

    # b2 with no values
    b2 = TypeB()
    b2.b1 = RefObj(None)
    b2.instance_of_A.a = RefObj(None)
    b2.instance_of_A.b = RefObj(None)
    b2.instance_of_A.c = []
    bin_dumper = BinDumper()
    bin_dumper.visit('no-name', b2)
    print('b2 dump:', bin_dumper.result, sep='\n')

    # deserialize b dump into b2
    loader = BinLoader(b1_dump)
    loader.visit('no-name', b2)

    # check the dump of b2
    jsonDump = BinDumper()
    jsonDump.visit('no-name', b2)
    print('b2 dump after load b1:', jsonDump.result, sep='\n')


if __name__ == "__main__":
    # binary_example()
    json_example()
