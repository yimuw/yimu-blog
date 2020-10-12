from number_tree_flow import *
from utils import *

def linear():
    a = Number(1, 'a')
    b = Number(2, 'b')
    c = Number(3, 'c')
    d = Number(4, 'd')
    e = Number(5, 'e')
    f1 = Number(1, 'f1', 'const')
    f2 = Number(1.1, 'f2', 'const')
    f3 = Number(1.2, 'f3', 'const')
    f4 = Number(1.3, 'f4', 'const')


    y = Number(10, 'y')

    t = a * a

    temp = y - (f1*a + f2*b + f3*c + f4*d + e)
    cost = temp * temp
    core = NumberFlowCore(cost)

    # build the topological order. Ignore it is fine. Just want to copy tensorflow
    with core as graph:
        for i in range(1000):
            print("cost.val:", cost.value, " iter:", i)
            core.forward()
            core.clear_grad()
            core.backward()
            core.gradient_desent(rate=0.0001)

            if cost.value < 1e-8:
                break

    for var in core.varible_nodes:
        print(var.id, var.value)

    for const in core.const_nodes:
        print(const.id, const.value)

    res = y.value - (f1.value * a.value + f2.value*b.value +
        f3.value*c.value + f4.value*d.value + e.value)
    print("test res:", res * res)


def logistic():
    a = Number(0, 'a')
    b = Number(0, 'b')
    f1 = Number(1, 'f1', 'const')

    pred = ntf_sigmoid(a * f1 + b)
    cost = - ntf_log(pred)
    core = NumberFlowCore(cost)

    # build the topological order. Ignore it is fine. Just want to copy tensorflow
    with core as graph:
        for i in range(2000):
            print("cost.val:", cost.value, " iter:", i)
            print('pred:', pred)
            # traverse_tree(cost)
            core.forward()
            core.clear_grad()
            core.backward()
            core.gradient_desent(rate=0.01)

            if cost.value < 1e-8:
                break

    for var in core.varible_nodes:
        print(var.id, var.value)

    for const in core.const_nodes:
        print(const.id, const.value)


if __name__ == "__main__":
    # linear()
    logistic()