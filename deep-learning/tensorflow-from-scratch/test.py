from variables_tree_flow import *
from utils import *

def linear():
    a = Variable(1, 'a')
    b = Variable(2, 'b')
    c = Variable(3, 'c')
    d = Variable(4, 'd')
    e = Variable(5, 'e')
    f1 = Variable(1, 'f1', 'const')
    f2 = Variable(1.1, 'f2', 'const')
    f3 = Variable(1.2, 'f3', 'const')
    f4 = Variable(1.3, 'f4', 'const')


    y = Variable(10, 'y')

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
    a = Variable(0, 'a')
    b = Variable(0, 'b')
    f1 = Variable(1, 'f1', 'const')

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


def test_chain():
    theta0 = np.array([Variable(value=1., id='t{}'.format(i))
                        for i in range(2 * 2)]).reshape([2, 2])
    theta1 = np.array([Variable(value=2., id='t{}'.format(i + 10))
                        for i in range(2 * 2)]).reshape([2, 2])
    f1 = np.array([3,4.])
    temp = theta0 @ f1
    print(temp.shape)
    pred =  theta1 @ temp
    cost = pred[0] + pred[1]
    core = NumberFlowCore(cost)

    # build the topological order. Ignore it is fine. Just want to copy tensorflow
    with core as graph:
        for i in range(10):
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
    #logistic()

    test_chain()