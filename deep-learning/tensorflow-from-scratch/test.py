from number_tree_flow import *
from utils import *

a = Number(1, 'a')
b = Number(2, 'b')
c = Number(3, 'c')
d = Number(4, 'd')

temp = (a + b) * (c * d)
cost = temp * temp

core = NumberFlowCore(cost)
for i in range(1000):
    core.forward()
    print("cost.val:", cost.value)
    core.backward()
    core.gradient_desent(rate=0.001)

for var in core.varible_nodes:
    print(var.id, var.value)
