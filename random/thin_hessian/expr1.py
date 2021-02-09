"""
1. solve for a,b using grad
2. running avg + prior
3. gd
4. x* = -b / 2a
5. x -= 0.01 * (df / a)
"""

import numpy as np
import matplotlib.pyplot as plt
import math
import argparse

METHOD = None

class function1:
    def __init__(self):
        self.a = 0.1
        self.b = 2
        self.c = 10

    def print_gt(self):
        print('x*-gt:', -self.b / (2*self.a))


    def f(self, x):
        a,b,c = self.a, self.b, self.c
        
        return a * x * x + b * x + c

    def df(self, x):
        a,b,c = self.a, self.b, self.c
        
        grad = 2 * a * x + b
        # return grad

        noise = np.random.normal(0, abs(grad) / 5)
        
        # print("grad, noise:", grad, noise)

        return grad + noise

class function2:
    """
    a convex function:
        (1.01 * x + math.cos(x)) ** 2
    """
    def __init__(self):
        pass

    def f(self, x):
        r = 1.01 * x + math.cos(x)
        return r * r

    def df(self, x):
        r = 1.01 * x +  math.cos(x)
        grad = 2 * r * (1.01 - math.sin(x))

        noise = np.random.normal(0, abs(grad) / 10)

        return grad + noise

class Solver:
    def __init__(self,func, x0):
        self.func = func
        self.x = x0

        self.a = 1
        self.b = 1

        self.iteration = 0

    def converge(self):
        d = self.func.df(self.x)
        return np.linalg.norm(d) < 1e-8

    def iter(self):
        x0 = self.x

        d0 = self.func.df(x0)

        # important. Make sure the sample points are far in normalized space
        direction = - d0  / abs(self.a) 
        alpha = 0.1

        x1 = x0 + alpha * direction
        d1 = self.func.df(x1)

        b = np.array([d0, d1])
        A = np.array([
            [2*x0, 1],
            [2*x1, 1],
        ])

        # prior
        K = 1
        prior_b = np.array([ + K * self.a, + K * self.b])
        prior_A =   ([
            [K, 0],
            [0, K],
        ])


        # p = np.linalg.solve(A , b) # solve for para directly

        p = np.linalg.solve(A.T @ A + prior_A, A.T @ b + prior_b) # least squares

        res_a, res_b = p
        update_weight = 0.9
        self.a = update_weight * self.a + (1- update_weight) * res_a
        self.b = update_weight * self.b + (1- update_weight) * res_b


        method = METHOD
        
        if method == 'c': # center
            x_star_est = - self.b / (2 * self.a)
            self.x = x0 + 0.1 * (x_star_est - x0)
        elif method == 'gd':
            print('d0:', d0)
            self.x = x0 - 0.01 * d0
        elif method == 's': # scale
            normalized_d0 = d0 / abs(self.a)
            print("d0, d0_norm: ", d0, normalized_d0)
            self.x = x0 - 0.01 * normalized_d0
        else:
            raise

        print('a b:', self.a, self.b)
        print('x*: ', - self.b / (2 * self.a))
        print('x:', self.x)


def test1():
    def solve(f, x0):
        d0 = f.df(x0)

        x1 = x0 - 0.001 * d0
        d1 = f.df(x1)

        b = np.array([d0, d1])
        A = np.array([
            [2*x0, 1],
            [2*x1, 1],
        ])

        p = np.linalg.solve(A, b)
        print("x0:", x0)
        print("a, c : ", p)
    func = function1()
    
    for x in np.linspace(-100, 100, 200):
        solve(func, x)

def test2():
    func = function1()
    s = Solver(func, 100)
    for i in range(2000):
        print('i:', i)
        s.iter()
        if s.converge():
            print('converge!')
            break
    func.print_gt()

def test3():
    func = function2()
    s = Solver(func, 10)
    for i in range(2000):
        print('i:', i)
        s.iter()
        if s.converge():
            print('converge!')
            break

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('opt_method', type=str, help='c: quad center; gd: gradient descent; s: scaled gd')
    args = parser.parse_args()

    METHOD = args.opt_method
    # test1()
    # test2()
    test3()
