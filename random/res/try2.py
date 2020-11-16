# -*- coding: utf-8 -*-
import torch
import math

dtype = torch.float
device = torch.device("cpu")

def gt(x):
    w1, w2, w3, w4 = 2, 2, -2, 2.
    y = w1*w2*w3*w4*x \
        + w1*w2*w3*x + w1*w2*w4*x + w1*w3*w4*x + w2*w3*w4*x\
        + w1*w2*x + w1*w3*x + w1*w4*x + w2*w3*x + w2*w4*x + w3*w4*x\
        + w1*x + w2*x + w3*x + w4*x + x
    return y

def f1(x, W):
    w1, w2, w3, w4 = W
    return w1*x + w2*x + w3*x + w4*x + x

def f2(x, W):
    w1, w2, w3, w4 = W
    return w1*w2*x + w1*w3*x + w1*w4*x + w2*w3*x + w2*w4*x + w3*w4*x\
        + w1*x + w2*x + w3*x + w4*x + x

def f3(x, W):
    w1, w2, w3, w4 = W
    return w1*w2*w3*x + w1*w2*w4*x + w1*w3*w4*x + w2*w3*w4*x\
        + w1*w2*x + w1*w3*x + w1*w4*x + w2*w3*x + w2*w4*x + w3*w4*x\
        + w1*x + w2*x + w3*x + w4*x + x

def f4(x, W):
    z = x
    for w in W:
        z = (w + 1.) * z
    y_pred = z
    return y_pred

INIT_WEIGHTS = [0.01, -0.01, 0.01, -0.01]
X = [1,2,3,4,5]
Y = [gt(x) + 0.1 for x in X]
learning_rate = 1e-6

def test2():
    print("test2 start!")
    weights = [torch.tensor([[INIT_WEIGHTS[i]]], device=device,
                            dtype=dtype, requires_grad=True) for i in range(4)]

    def f(x):
        z = x
        for w in weights:
            z = (w + 1.) * z
        y_pred = z
        return y_pred

    for t in range(2000):
        
        loss = 0
        for x, y in zip(X, Y):
            y_pred = f(x)
            loss += (y_pred - y).pow(2)

        if t % 50 == 0:
            print('iter:', t, 'loss:', loss.item())

        loss.backward()

        with torch.no_grad():
            for _, w in enumerate(weights):
                w -= learning_rate * w.grad
                w.grad.zero_()
    
    for x, y in zip(X, Y):
        y_pred = f(x)
        print('y:', y ,' y_pred:', y_pred.item())

def test3():
    print("test2 start!")
    weights = [torch.tensor([[INIT_WEIGHTS[i]]], device=device,
                            dtype=dtype, requires_grad=True) for i in range(4)]


    for t in range(10000):
        
        loss = 0
        for x, y in zip(X, Y):
            if t < 2000:
                f = f1
            elif 2000 <= t < 4000:
                f = f2
            elif 4000 < t < 6000:
                f = f3
            else:
                f = f4

            y_pred = f(x, weights)
            loss += (y_pred - y).pow(2)

        if t % 50 == 0:
            print('iter:', t, 'loss:', loss.item())

        loss.backward()

        with torch.no_grad():
            for _, w in enumerate(weights):
                w -= learning_rate * w.grad
                w.grad.zero_()
    
    for x, y in zip(X, Y):
        y_pred = f(x, weights)
        print('y:', y ,' y_pred:', y_pred.item())


if __name__ == "__main__":
    test3()

