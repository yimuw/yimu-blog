import numpy as np
# from scipy.linalg import logm, expm
from math import cos, sin, pi
import matplotlib.pyplot as plt


def SO2_expm(theta):
    return np.array([
        [cos(theta), -sin(theta)],
        [sin(theta), cos(theta)],
    ])

def SO2_log(R):
    return np.arctan2(R[1, 0], R[0, 0])

def vee(l1):
    assert(l1.size == 2)
    a, b = l1
    return np.array([b, -a])


class ManipulatorOptimization:
    def __init__(self, target):
        self.R1_init = SO2_expm(0.01)
        self.R2_init = SO2_expm(0.01)
        self.R1 = self.R1_init
        self.R2 = self.R2_init
        self.l1 = np.array([1., 0])
        self.l2 = np.array([2., 0])

        self.target = target

    def get_theta(self):
        assert(abs(np.linalg.det(self.R1) - 1.) < 1e-8)
        assert(abs(np.linalg.det(self.R2) - 1.) < 1e-8)

        return SO2_log(self.R1), SO2_log(self.R2)

    def end_effector_position(self, R1, R2):
        p1_position = R1 @ self.l1
        p2_position = p1_position + R2 @ R1 @ self.l2
        return p2_position

    def end_effector_position_jacobi_wrt_theta(self):
        jacobi = np.zeros([2, 2])
        # dr / d w1
        jacobi[:, 0] =  - self.R1 @ vee(self.l1) - self.R2 @ self.R1 @ vee(self.l2)
        # dr / d w2
        jacobi[:, 1] =  - self.R2 @ vee(self.R1 @ self.l2)
        return jacobi

    def gradient_checking(self):
        jacobi_analytic = self.end_effector_position_jacobi_wrt_theta()
        jacobi_nu = np.zeros([2, 2])
        DELTA = 1e-8
        jacobi_nu[:, 0] = (self.residual(self.R1 @ SO2_expm(DELTA), self.R2) \
            - self.residual(self.R1 @ SO2_expm(- DELTA), self.R2)) / (2 * DELTA)
        jacobi_nu[:, 1] = (self.residual(self.R1, self.R2 @ SO2_expm(DELTA)) \
            - self.residual(self.R1, self.R2 @ SO2_expm(-DELTA))) / (2 * DELTA)
        print('jacobi_analytic:', jacobi_analytic)
        print('jacobi_nu      :', jacobi_nu)

    def residual(self, R1, R2):
        return self.end_effector_position(R1, R2) - self.target

    def SO2_generalized_plus(self, delta_local_params):
        w1, w2 = delta_local_params
        self.R1 = self.R1 @ SO2_expm(w1)
        self.R2 = self.R2 @ SO2_expm(w2)

    def optimize(self):
        for iters in range(20):
            # self.gradient_checking()
            jacobi = self.end_effector_position_jacobi_wrt_theta()
            b = self.residual(self.R1, self.R2)

            delta_local_params = np.linalg.solve(jacobi.T @ jacobi, -jacobi.T @ b)

            self.SO2_generalized_plus(delta_local_params)

            cost = b.T @ b
            print('cost: ', cost)
            if cost < 1e-8:
                print('converged at iteration: ', iters)
                break

    def interp_and_show_video(self):
        steps = 100
        dt = 0.02

        # basically so2
        R1_speed = SO2_log(self.R1_init.T @ self.R1)
        R2_speed = SO2_log(self.R2_init.T @ self.R2)

        for k in np.linspace(0, 1, 100):
            R1_k = self.R1_init @ SO2_expm(k * R1_speed)
            R2_k = self.R2_init @ SO2_expm(k * R2_speed)

            p1_position = R1_k @ self.l1
            p2_position = p1_position + R2_k @ R1_k @ self.l2

            p1x, p1y = p1_position
            p2x, p2y = p2_position
            plt.cla()
            plt.plot([0, p1x, p2x], [0, p1y, p2y])
            plt.xlim([-3, 3])
            plt.ylim([-3, 3])

            plt.pause(.001)

        plt.show()


def main():
    end_effector_target = np.array([0, 1.2])
    manipulator_on_the_manifold = ManipulatorOptimization(end_effector_target)
    manipulator_on_the_manifold.optimize()
    manipulator_on_the_manifold.interp_and_show_video()
    


if __name__ == "__main__":
    main()
