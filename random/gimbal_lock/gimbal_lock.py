import numpy as np
from math import cos, sin
from scipy.linalg import logm, expm

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d


class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)


def rotate_lock(a):
    r = np.array([
        [0, 0, 1],
        [sin(a), cos(a), 0],
        [-cos(a), sin(a), 0],
    ])

    return r


def rotate_x(a):
    r = np.array([
        [cos(a), -sin(a), 0],
        [sin(a), cos(a), 0],
        [0, 0, 1],
    ])

    return r


def solve_axis_svd(R):
    A = R - np.identity(3)
    u, s, v = np.linalg.svd(A)
    n = v[2, :].T

    direction = np.array([1, 0, 0.])
    if n @ direction < 0:
        n = - n

    return n


def solve_axis_log(R):
    w_skew = logm(R)

    w1 = w_skew[2, 1]
    w2 = w_skew[0, 2]
    w3 = w_skew[1, 0]
    return np.array([w1, w2, w3])


def try_plot(R, fig):
    fig.clf()

    ax1 = fig.add_subplot(111, projection='3d')
    # ax1.view_init(1, 1)
    ax1.set_xlabel('X')
    ax1.set_xlim(-1, 1)
    ax1.set_ylabel('Y')
    ax1.set_ylim(-1, 1)
    ax1.set_zlabel('Z')
    ax1.set_zlim(-1, 1)

    # Here we create the arrows:
    arrow_prop_dict = dict(
        mutation_scale=20, arrowstyle='->', shrinkA=0, shrinkB=0)

    x1, y1, z1 = R[:, 0]
    a = Arrow3D([0, x1], [0, y1], [0, z1], **arrow_prop_dict, color='r')
    ax1.add_artist(a)

    x2, y2, z2 = R[:, 1]
    a = Arrow3D([0, x2], [0, y2], [0, z2], **arrow_prop_dict, color='b')
    ax1.add_artist(a)

    x3, y3, z3 = R[:, 2]
    a = Arrow3D([0, x3], [0, y3], [0, z3], **arrow_prop_dict, color='g')
    ax1.add_artist(a)

    # Give them a name:
    ax1.text(0.0, 0.0, -0.1, r'$0$')
    ax1.text(x1, y1, z1, r'$x-new$')
    ax1.text(x2, y2, z2, r'$y-new$')
    ax1.text(x3, y3, z3, r'$z-new$')

    # ref
    a = Arrow3D([0, 1], [0, 0], [0, 0], **
                arrow_prop_dict, color='r', alpha=0.3)
    ax1.add_artist(a)

    a = Arrow3D([0, 0], [0, 1], [0, 0], **
                arrow_prop_dict, color='b', alpha=0.3)
    ax1.add_artist(a)

    a = Arrow3D([0, 0], [0, 0], [0, 1], **
                arrow_prop_dict, color='g', alpha=0.3)
    ax1.add_artist(a)

    # Give them a name:
    ax1.text(1.1, 0, 0, r'$x-ref$')
    ax1.text(0, 1.1, 0, r'$y-ref$')
    ax1.text(0, 0, 1.1, r'$z-ref$')

    # rotation_axis = solve_axis_log(R)
    rotation_axis = solve_axis_svd(R)

    w1, w2, w3 = rotation_axis
    a = Arrow3D([0, w1], [0, w2], [0, w3], **
                arrow_prop_dict, color='m', alpha=0.5)
    ax1.text(w1, w2, w3, r'$w$')
    ax1.add_artist(a)
    # plt.show()


def main():
    fig = plt.figure()
    for i, a in enumerate(np.linspace(0, 2 * np.pi, 60)):
        R = rotate_x(a)
        R = rotate_lock(a)
        try_plot(R, fig)
        plt.pause(0.1)
        plt.savefig('res/{0:03d}.png'.format(i))
        # plt.show()
    # solve_axis()


if __name__ == "__main__":
    main()
