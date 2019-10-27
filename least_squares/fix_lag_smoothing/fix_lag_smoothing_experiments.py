#import linalg package of the SciPy module for the LU decomp 
import scipy.linalg as linalg 
import numpy as np
#import NumPy import numpy as np 
#define A same as before 
A = np.array([[2., 1., 1.], [1., 3., 2.], [1., 0., 0.]]) 
#define B 
B = np.identity(3)
#call the lu_factor function 
LU = linalg.lu_factor(A) 
#solve given LU and B 
x_lu = linalg.lu_solve(LU, B) 
print("Solutions by LU:\n",x_lu) 

x_solve = np.linalg.solve(A, B)
print("Solutions by np.solve:\n",x_solve) 

test = x_lu @ A
print('test: ', test)