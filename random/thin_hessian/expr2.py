import numpy as np
import matplotlib.pyplot as plt
import math

class function3:
    def __init__(self):
        self.A = np.array([
            [0.1, 0.2],
            [0.2, 4.]
        ])

        self.B = np.array([2,2.]).T
        self.C = 0

    def print_gt(self):
        print('x*_gt:', np.linalg.inv(- self.A) @ self.B / 2 )


    def f(self, x):
        A, B, C = self.A, self.B, self.C
        
        return x.T @ A @ x + B.T @ x + C

    def df(self, x):
        A, B, C = self.A, self.B, self.C
        
        grad = 2 * A @ x + B
        return grad

        noise = np.random.normal(0, abs(grad) / 10, grad.shape)
        
        # print("grad, noise:", grad, noise)

        return grad + noise


class Solver:
    def __init__(self,func, x0):
        self.func = func
        self.x = x0

        self.A = np.array([1. ,1.]).T
        self.B = np.array([0. ,0.]).T

        self.iteration = 0

    def iter(self):
        x0 = self.x

        d0 = self.func.df(x0)
        # print("x0:", x0)
        # print("d0:", d0)

        # important. Make sure the sample points are far in normalized space
        direction = - d0
        alpha = 0.1

        x1 = x0 + alpha * direction
        d1 = self.func.df(x1)

        b = np.hstack([d0, d1])
        A = np.identity(4)
        A[0:2, 0:2] = np.diag(2 * x0)
        A[0:2, 2:4] = np.identity(2)
        A[2:4, 0:2] = np.diag(2 * x1)
        A[2:4, 2:4] = np.identity(2)

        # print('A, b:', A, b)
        # prior
        K = 0.25
        prior_b = K * np.hstack([self.A, self.B])
        prior_A =  K * np.identity(4)


        # p = np.linalg.solve(A , b) # solve for para directly

        p = np.linalg.solve(A.T @ A + prior_A, A.T @ b + prior_b) # least squares

        res_a, res_b = p[:2], p[2:4]
        update_weight = 0.9
        self.A = update_weight * self.A + (1- update_weight) * res_a
        self.B = update_weight * self.B + (1- update_weight) * res_b

        print('self.A', self.A)
        print('self.B', self.B)


        # self.x = x1

        method = 'c'
        
        if method == 'c': # center
            x_star_est = - self.B / self.A / 2 # since A is diag
            print('x_star_est:', x_star_est)
            self.x = x0 + 0.1 * (x_star_est - x0)
        elif method == 'gd':
            self.x = x0 - 0.01 * d0
        # elif method == 's': # scale
        #     normalized_d0 = d0 / abs(self.a)
        #     print("d0, d0_norm: ", d0, normalized_d0)
        #     self.x = x0 - 0.01 * normalized_d0
        else:
            raise
        
        print('x:', self.x)

def test4():
    func = function3()
    s = Solver(func, np.array([2,3]))
    for i in range(300):
        print('i:', i)
        s.iter()
    
    func.print_gt()

if __name__ == "__main__":
    

    test4()
