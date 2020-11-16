# -*- coding: utf-8 -*-
import torch
import math
import matplotlib.pyplot as plt
import numpy as np

dtype = torch.float
device = torch.device("cpu")
INIT_WEIGHTS = [0.01, -0.01, 0.01, -0.01]
x = 1
y = 2
learning_rate = 1e-2



def plot_loss():
    weights = INIT_WEIGHTS[:]
    def loss1(xx, W):
        z = xx
        for w in W:
            z = (w + 1) * z
        y_pred = z
        loss = (y_pred - y) ** 2
        return loss
    
    def loss2(xx, W):
        w1,w2,w3,w4 = W
        y_pred = w1*w2*w3*w4*x
        loss = (y_pred - y) ** 2
        return loss

    loss1_all = []
    loss2_all = []
    w1_range = np.linspace(-3, 3, 100)
    for w1 in w1_range:
        weights[0] = w1
        loss1_all.append(loss1(x, weights))
        loss2_all.append(loss2(x, weights))
    
    plt.plot(w1_range, loss1_all, 'r')
    plt.plot(w1_range, loss2_all, 'b')
    plt.show()


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
    plot_loss()
    
    test1()
    test2()
    test3()
