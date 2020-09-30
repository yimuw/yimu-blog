from number_tree_flow import * 
from utils import *

a = Number(1, 'a')
b = Number(2, 'b')
c = Number(3, 'c')
d = Number(4, 'd')
e = Number(5, 'e')
f = Number(-1.5, 'f')

res = (a + b) * c * d + (e * f)
traverse_tree(res)

core = NumberFlowCore()
print("res.val:", res.value)
core.forward(res)
print("res.val:", res.value)