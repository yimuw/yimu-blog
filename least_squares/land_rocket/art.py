import os
from math import cos, sin

import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
import numpy as np
import pylab
import argparse


def save_cur_figure(dir, file_name):
    if not os.path.exists(dir):
        os.mkdir(dir)

    path = os.path.join(dir, file_name)
    plt.savefig(path)


class Artist:
    def __init__(self):
        import pathlib
        current_file_path = pathlib.Path(__file__).parent.absolute()
        fire_im = plt.imread(os.path.join(current_file_path, 'art_files/blue_fire.png'))

        # may need filter
        self.fire_im = fire_im[::5, ::5, :]
        self.earth_im = plt.imread(os.path.join(current_file_path, 'art_files/rocket.jpg'))

    def draw_at(self, x, y, theta_raw, fire_len_scale, scale=1.):
        theta = theta_raw - np.pi / 2.

        im_earth = plt.imshow(self.earth_im)
        elen_y, elen_x, _ = self.earth_im.shape
        trans_data = \
            mtransforms.Affine2D().translate(-elen_x / 2., -elen_y / 2.) \
            + mtransforms.Affine2D().scale(0.1 * scale) \
            + mtransforms.Affine2D().rotate(theta + np.pi) \
            + mtransforms.Affine2D().translate(x, y)

        trans0 = im_earth.get_transform()
        trans_data = trans_data + trans0
        im_earth.set_transform(trans_data)

        im_fire = plt.imshow(self.fire_im)
        flen_y, flen_x, _ = self.fire_im.shape
        trans_data = mtransforms.Affine2D().translate(-flen_x / 2., -flen_y * 0.8) \
                     + mtransforms.Affine2D().scale(0.25 * scale, 0.5 * scale * fire_len_scale) \
                     + mtransforms.Affine2D().rotate(theta) \
                     + mtransforms.Affine2D().translate(x, y)

        trans0 = im_fire.get_transform()
        trans_data = trans_data + trans0
        im_fire.set_transform(trans_data)


def movie(trajectory_mat):
    """
    show a trajectory moive
    """
    DT_IDX = 0
    X_IDX = 1
    Y_IDX = 2
    HEADING_IDX = 3
    VX_IDX = 4
    VY_IDX = 5
    ACCL_IDX = 6
    HEADING_DOT_IDX = 7
    # dt, x, y, heading, vx, vy, accl, heading_dot

    max_x = np.max(trajectory_mat[:, X_IDX])
    max_y = np.max(trajectory_mat[:, Y_IDX])
    min_x = np.min(trajectory_mat[:, X_IDX])
    min_y = np.min(trajectory_mat[:, Y_IDX])
    max_diff = max(max_x - min_x, max_y - min_y)

    max_thurst = max(trajectory_mat[:, ACCL_IDX])

    speed = np.linalg.norm(trajectory_mat[:, VX_IDX:(VY_IDX+1)], axis=1)
    max_speed = max(speed)

    scale = 1  # (states[-1] - states[0])[0] / 30

    num_states, state_size = trajectory_mat.shape
    assert state_size == 8

    time = 0.
    for i in range(num_states):
        dt, x, y, theta, v_x, v_y, thrust, theta_dot = trajectory_mat[i]

        plt.clf()
        time += dt

        thrust_x = cos(theta) * thrust
        thrust_y = sin(theta) * thrust

        ax1 = plt.subplot(1, 1, 1)
        plt.plot(trajectory_mat[:, X_IDX], trajectory_mat[:, Y_IDX], '-o', alpha=0.2)
        # plt.axis('equal')
        x_max = max(trajectory_mat[:, X_IDX])
        x_min = min(trajectory_mat[:, X_IDX])
        y_max = max(trajectory_mat[:, Y_IDX])
        y_min = min(trajectory_mat[:, Y_IDX])
        margin = 3
        plt.xlim([x_min - margin, x_max + margin])
        plt.ylim([y_min - margin, y_max + margin])
        plt.autoscale(False)

        if abs(thrust) > 0:
            plt.arrow(x - thrust_x * scale, y - thrust_y * scale,
                      thrust_x * scale, thrust_y * scale,
                      width=0.05)
        artist = Artist()
        artist.draw_at(x,y,theta, thrust, scale=0.03)
        plt.title('time :{:10.4f}sec, pos:({:10.4f}{:10.4f})'\
            .format(time, x, y))

        # save_cur_figure('test', '{}.png'.format(i))
        plt.pause(0.01)
    plt.show()


def movie_and_plots(trajectory_mat):
    """
    show a trajectory moive
    """
    DT_IDX = 0
    X_IDX = 1
    Y_IDX = 2
    HEADING_IDX = 3
    VX_IDX = 4
    VY_IDX = 5
    ACCL_IDX = 6
    HEADING_DOT_IDX = 7
    # dt, x, y, heading, vx, vy, accl, heading_dot

    max_x = np.max(trajectory_mat[:, X_IDX])
    max_y = np.max(trajectory_mat[:, Y_IDX])
    min_x = np.min(trajectory_mat[:, X_IDX])
    min_y = np.min(trajectory_mat[:, Y_IDX])
    max_diff = max(max_x - min_x, max_y - min_y)

    max_thurst = max(trajectory_mat[:, ACCL_IDX])

    speed = np.linalg.norm(trajectory_mat[:, VX_IDX:(VY_IDX+1)], axis=1)
    max_speed = max(speed)

    scale = 1  # (states[-1] - states[0])[0] / 30

    num_states, state_size = trajectory_mat.shape
    assert state_size == 8

    time = 0.
    for i in range(num_states):
        dt, x, y, theta, v_x, v_y, thrust, theta_dot = trajectory_mat[i]

        plt.clf()
        time += dt

        thrust_x = cos(theta) * thrust
        thrust_y = sin(theta) * thrust

        ax1 = plt.subplot(2, 3, 1)
        plt.plot(trajectory_mat[:, X_IDX], trajectory_mat[:, Y_IDX], '-o', alpha=0.2)
        # plt.axis('equal')
        x_max = max(trajectory_mat[:, X_IDX])
        x_min = min(trajectory_mat[:, X_IDX])
        y_max = max(trajectory_mat[:, Y_IDX])
        y_min = min(trajectory_mat[:, Y_IDX])
        margin = 3
        plt.xlim([x_min - margin, x_max + margin])
        plt.ylim([y_min - margin, y_max + margin])
        plt.autoscale(False)

        if abs(thrust) > 0:
            plt.arrow(x - thrust_x * scale, y - thrust_y * scale,
                      thrust_x * scale, thrust_y * scale,
                      width=0.05)
        artist = Artist()
        artist.draw_at(x,y,theta, thrust, scale=0.03)
        plt.title('time :{:10.4f}sec, pos:({:10.4f}{:10.4f})'\
            .format(time, x, y))

        plt.subplot(2, 3, 2)
        plt.plot(trajectory_mat[:, ACCL_IDX:(HEADING_DOT_IDX+1)], 'g', alpha=0.1)
        plt.plot(trajectory_mat[:i+1, HEADING_DOT_IDX], 'r', label='theta_dot')
        plt.plot(trajectory_mat[:i+1, ACCL_IDX], 'b', label='thurst')
        pylab.legend(loc='upper left')
        plt.xlabel('index')
        plt.ylabel('thrust / theta_dot')
        plt.title('controls,theta:{:10.4f} thrust:{:10.4f}'.format(trajectory_mat[i, HEADING_DOT_IDX], trajectory_mat[i, ACCL_IDX]))

        plt.subplot(2, 3, 3)
        plt.plot(trajectory_mat[:, VX_IDX:(VY_IDX+1)], 'g', alpha=0.1)
        plt.plot(trajectory_mat[:i+1, VX_IDX], 'r', label='vx')
        plt.plot(trajectory_mat[:i+1, VY_IDX], 'b', label='vy')
        pylab.legend(loc='upper left')
        plt.title('speed')
        plt.xlabel('idx')
        plt.ylabel('meter/sec')

        gx = 0
        gy = -1
        plt.subplot(2, 3, 4)
        plt.plot([0, thrust_x], [0, thrust_y], alpha=0.5, label='thrust')
        plt.scatter(thrust_x, thrust_y)
        plt.plot([0, 0], [gx, gy], alpha=0.5, color='g', label='gravitation')
        plt.scatter(gx, gy, color='g')
        max_acc = max(max_thurst, 1)
        pylab.legend(loc='upper left')
        plt.xlim([-max_acc, max_acc])
        plt.ylim([-max_acc, max_acc])
        plt.title('thrust and gravitation')

        plt.subplot(2, 3, 5)
        plt.plot([0, thrust_x + gx], [0, thrust_y + gy], alpha=0.5, label='accl')
        plt.scatter(thrust_x + gx, thrust_y + gy)
        max_acc = max(max_thurst, 1)
        plt.xlim([-max_acc, max_acc])
        plt.ylim([-max_acc, max_acc])
        plt.title('accl=thrust+g, ({:10.4f},{:10.4f})'.format(thrust_x + gx, thrust_y + gy))

        plt.subplot(2, 3, 6)
        plt.plot([0, v_x], [0, v_y], color='b', alpha=0.5)
        plt.scatter(v_x, v_y, color='b', label='speed')
        plt.plot([0, cos(theta)], [0, sin(theta)], color='g', alpha=0.5)
        plt.scatter(cos(theta), sin(theta), color='g', label='heading')
        plt.xlim([-max_speed, max_speed])
        plt.ylim([-max_speed, max_speed])
        plt.title('speed vector, heading vector')
        pylab.legend(loc='upper left')
        plt.xlabel('vx')
        plt.ylabel('vy')

        plt.pause(0.01)

        # save_cur_figure('test', '{}.png'.format(i))
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='plot the rocket trajectory')
    parser.add_argument('--path', help='path to rocket csv file')
    args = parser.parse_args()

    trajectory_mat = np.genfromtxt(args.path, delimiter=' ')
    
    movie_and_plots(trajectory_mat)
    movie(trajectory_mat)