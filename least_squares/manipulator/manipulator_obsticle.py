import numpy as np
# from scipy.linalg import logm, expm
from math import cos, sin, pi, sqrt
import matplotlib.pyplot as plt


def SO2_expm(theta):
    return np.array([
        [cos(theta), -sin(theta)],
        [sin(theta), cos(theta)],
    ])

def SO2_log(R):
    return np.arccos(R[0, 0])

def vee(l1):
    assert(l1.size == 2)
    a, b = l1
    return np.array([-b, a])


class ManipulatorOptimization:
    def __init__(self, target):
        self.R1_init = SO2_expm(0.01)
        self.R2_init = SO2_expm(0.01)

        self.obsticle_position = np.array([0, 3.])
        self.obs_k = 2

        self.R1 = self.R1_init.copy()
        self.R2 = self.R2_init.copy()
        self.l1 = np.array([1., 0])
        self.l2 = np.array([2., 0])

        self.target = target

    def get_theta(self):
        assert(abs(np.linalg.det(self.R1) - 1.) < 1e-8)
        assert(abs(np.linalg.det(self.R2) - 1.) < 1e-8)

        return SO2_log(self.R1), SO2_log(self.R2)

    def end_effector_position(self, R1, R2):
        p1_position = R1 @ self.l1
        p2_position = p1_position + R1 @ R2 @ self.l2
        return p2_position

    def jacobi_end_effector_position_wrt_theta(self):
        jacobi = np.zeros([2, 2])
        # dr / d w1
        jacobi[:, 0] = self.R1 @ vee(self.l1) + self.R1 @ vee(self.R2 @ self.l2)
        # dr / d w2
        jacobi[:, 1] = self.R1 @ self.R2 @ vee(self.l2)
        return jacobi

    def end_effector_to_obsticle_distance(self, R1, R2):
        # only sample end effector on the robot for simplicity
        # Ideally, we should find the distance to the nearest point on the robot.
        p1_position = R1 @ self.l1
        p2_position = p1_position + R1 @ R2 @ self.l2
        d2 = np.linalg.norm(p2_position- self.obsticle_position)
        return d2
    
    def jacobi_obsticle_distance_wrt_theta(self):
        jacobi = np.zeros([1, 2])
        dr_ddist = 1. / sqrt(2 * self.obs_k)

        p2 = self.end_effector_position(self.R1, self.R2)
        dist = self.end_effector_to_obsticle_distance(self.R1, self.R2)
        if dist < self.obs_k:
            ddist_dp2 = 1. / dist * (p2 - self.obsticle_position)
            dp2_dtheta = self.jacobi_end_effector_position_wrt_theta()
            jacobi = dr_ddist * ddist_dp2 @ dp2_dtheta
        return jacobi

    def jacobi(self):
        jacobi_goal = self.jacobi_end_effector_position_wrt_theta()
        jacobi_obsticle = self.jacobi_obsticle_distance_wrt_theta()
        return np.vstack([jacobi_goal, jacobi_obsticle])

    def gradient_checking(self):
        jacobi_analytic = self.jacobi()
        jacobi_nu = np.zeros([3, 2])
        DELTA = 1e-8
        jacobi_nu[:, 0] = (self.residual(self.R1 @ SO2_expm(DELTA), self.R2) \
            - self.residual(self.R1 @ SO2_expm(- DELTA), self.R2)) / (2 * DELTA)
        jacobi_nu[:, 1] = (self.residual(self.R1, self.R2 @ SO2_expm(DELTA)) \
            - self.residual(self.R1, self.R2 @ SO2_expm(-DELTA))) / (2 * DELTA)
        #print('jacobi_analytic:', jacobi_analytic)
        #print('jacobi_nu      :', jacobi_nu)

    def end_effector_residual(self, R1, R2):
        return self.end_effector_position(R1, R2) - self.target
    
    def obsticle_residual(self, R1, R2):
        d2 = self.end_effector_to_obsticle_distance(R1, R2)
        if d2 < self.obs_k:
            return np.array([(d2 - self.obs_k) / sqrt(2 * self.obs_k)])
        else:
            return np.array([0.])

    def residual(self, R1, R2):
        r_goal = self.end_effector_residual(R1, R2)
        r_obsticle = self.obsticle_residual(R1, R2)
        return np.concatenate([r_goal, r_obsticle])

    def SO2_generalized_plus(self, delta_local_params):
        w1, w2 = delta_local_params
        self.R1 = self.R1 @ SO2_expm(w1)
        self.R2 = self.R2 @ SO2_expm(w2)

    def optimize(self):
        for iters in range(100):
            self.gradient_checking()
            jacobi = self.jacobi()
            b = self.residual(self.R1, self.R2)

            delta_local_params = np.linalg.solve(jacobi.T @ jacobi, -jacobi.T @ b)

            step = 0.1
            self.SO2_generalized_plus(step * delta_local_params)

            cost = b.T @ b
            print('cost: ', cost)
            print('residual: ', b)
            # print('delta_local_params: ', delta_local_params)
            # print('theta:', self.get_theta())
            if cost < 1e-4:
                print('converged at iteration: ', iters)
                break
    
    def interp_and_show_video(self):
        steps = 100
        dt = 0.02

        # basically so2
        R1_speed = SO2_log(self.R1_init.T @ self.R1)
        R2_speed = SO2_log(self.R2_init.T @ self.R2)

        for i in range(steps):
            k = float(i) / steps
            R1_k = self.R1_init @ SO2_expm(k * R1_speed)
            R2_k = self.R2_init @ SO2_expm(k * R2_speed)

            p1_position = R1_k @ self.l1
            p2_position = p1_position + R1_k @ R2_k @ self.l2

            p1x, p1y = p1_position
            p2x, p2y = p2_position
            plt.cla()
            plt.plot([0, p1x, p2x], [0, p1y, p2y])
            plt.xlim([-3, 3])
            plt.ylim([-3, 3])

            plt.pause(.001)
            # print(self.end_effector_to_obsticle_distance(R1_k, R2_k))
        plt.show()

def main():
    end_effector_target = np.array([-2, 0])
    manipulator_on_the_manifold = ManipulatorOptimization(end_effector_target)
    manipulator_on_the_manifold.optimize()
    manipulator_on_the_manifold.interp_and_show_video()
    


if __name__ == "__main__":
    main()
