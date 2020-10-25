# -*- coding: utf-8 -*-
import torch
import math

dtype = torch.float
device = torch.device("cpu")
INIT_WEIGHTS = [0.01, -0.01, 0.01, -0.01]
x = 1
y = 1
learning_rate = 1e-2


def test1():
    print("test1 start!")
    weights = [torch.tensor([[INIT_WEIGHTS[i]]], device=device,
                            dtype=dtype, requires_grad=True) for i in range(4)]

    for t in range(500):
        z = x
        for w in weights:
            z = w * z
        y_pred = z

        loss = (y_pred - y).pow(2).sum()
        if t % 50 == 0:
            print('iter:', t, 'loss:', loss.item(), 'y_pred:', y_pred.item())

        loss.backward()

        with torch.no_grad():
            for l, w in enumerate(weights):
                w -= learning_rate * w.grad
                w.grad.zero_()


def test2():
    print("test2 start!")
    weights = [torch.tensor([[INIT_WEIGHTS[i]]], device=device,
                            dtype=dtype, requires_grad=True) for i in range(4)]

    for t in range(500):

        z = x
        for w in weights:
            z = (w + 1.) * z
        y_pred = z

        loss = (y_pred - y).pow(2).sum()
        if t % 50 == 0:
            print('iter:', t, 'loss:', loss.item(), 'y_pred:', y_pred.item())

        loss.backward()

        with torch.no_grad():
            for l, w in enumerate(weights):
                w -= learning_rate * w.grad
                w.grad.zero_()


def test3():
    print("test3 start!")
    weights = [torch.tensor([[INIT_WEIGHTS[i]]], device=device,
                            dtype=dtype, requires_grad=True) for i in range(4)]
    w1, w2, w3, w4 = weights

    for t in range(500):

        y_pred = w1*w2*w3*w4*x \
            + w1*w2*w3*x + w1*w2*w4*x + w1*w3*w4*x + w2*w3*w4*x\
            + w1*w2*x + w1*w3*x + w1*w4*x + w2*w3*x + w2*w4*x + w3*w4*x\
            + w1*x + w2*x + w3*x + w4*x + x
        loss = (y_pred - y).pow(2).sum()
        if t % 50 == 0:
            print('iter:', t, 'loss:', loss.item(), 'y_pred:', y_pred.item())

        loss.backward()

        with torch.no_grad():
            for l, w in enumerate(weights):
                w -= learning_rate * w.grad
                w.grad.zero_()


if __name__ == "__main__":
    test1()
    test2()
    test3()
