# from scipy.linalg import logm, expm
from math import cos, pi, sin
from scipy.linalg import expm

import matplotlib.pyplot as plt
import numpy as np

"""
Problem

  cost(R, P) = || D - take_first_col(R) * P ||^2

  =>

  cost(w, P) = || D - take_first_col(R @ exp(W)) * P ||
  cost(w, P) = || D - R @ take_first_col(exp(W)) * P  ||
  First order approximation, exp(W) = I + W
  cost(W, P) = ||D - R @ take_first_col(I + W) * P||
  Only need to solve for n-1 parameters
  cost(W_c1, P) = ||D - R @ ([1,w1,w2,w3,..]) * P|| 

  => get W_c1
  update: R = R * exp([W_c1,0,..])
"""
def generate_point_cloud():
    mean = np.array([0, 0, 0, 0.])
    
    # R = utils.euler_angle_to_rotation(0.4, -1., 6.6666)
    Mat = np.random.rand(4, 4)
    W = Mat.transpose() - Mat
    R = expm(W)
    np.testing.assert_almost_equal(R.transpose() @ R, np.identity(4))

    # Transformation for covariance
    cov = R @ np.diag([0.001, 1, 9., 0.01]) @ R.transpose()
    num_point = 500
    points = np.random.multivariate_normal(mean, cov, size=num_point)

    points_mean = np.mean(points, axis=0)
    print('points_mean:', points_mean)
    points = points - points_mean

    print('R_gt: ', R)
    print('cov_gt:', cov)
    print('numpy.linalg.eig(cov):', np.linalg.eig(cov))

    return points.transpose()


def take_first_cols(R):
    return R[:, :1]

class PCA_SO4_first_principle_component:
    """
    https://stats.stackexchange.com/questions/10251/what-is-the-objective-function-of-pca
    """
    def __init__(self, points):
        self.num_data = points.shape[1]
        self.num_residuals = points.size
        points_mean = np.mean(points, axis=1)
        self.points = (points.transpose() - points_mean).transpose()
        self.points_covariance = points @ points.transpose() / self.num_data
        # print(self.points_covariance)
        self.var_SO4 = np.identity(4)
        self.var_projection = np.zeros([1, self.num_data])

    def residaul(self, var_SO4, var_projection):
        """
          r_i = p_i - first_k_cols(R) * w
        """
        # self.point.shape == (3, n)
        r = self.points - take_first_cols(var_SO4) @ var_projection
        # make r col major
        return r.transpose().flatten()

    def compute_projection(self):
        self.var_projection = take_first_cols(self.var_SO4).transpose() @ self.points

    def hat_local_so4_first_col(self, local_params):
        W = np.zeros([4,4])
        w1, w2, w3 = local_params
        W[1, 0] = w1
        W[2, 0] = w2
        W[3, 0] = w3
        W[0, 1] = -w1
        W[0, 2] = -w2
        W[0, 3] = -w3
        return W

    def local_so4_first_col_to_SO4(self, local_params):
        W = self.hat_local_so4_first_col(local_params)
        return expm(W)

    def numerical_jacobi_wrt_first_col(self):
        """
            r_i = p_i - first_k_cols(R) * w
        """
        DELTA = 1e-8

        jacobian = np.zeros([self.num_residuals, 3])

        w_so4_local_first_col = np.array([0, 0, 0.])
        curret_params = w_so4_local_first_col.copy()
        for p_idx in range(3):
            params_plus = curret_params.copy()
            params_plus[p_idx] += DELTA
            residual_plus = self.residaul(self.var_SO4 @ self.local_so4_first_col_to_SO4(params_plus), self.var_projection)

            params_minus = curret_params.copy()
            params_minus[p_idx] -= DELTA
            residual_minus = self.residaul(self.var_SO4 @ self.local_so4_first_col_to_SO4(params_minus), self.var_projection)

            dr_dpidx = (residual_plus - residual_minus) / (2. * DELTA)
            jacobian[:, p_idx] = dr_dpidx

        return jacobian

    def jacobi_wrt_so3(self):
        """
        """
        pass

    def solve_normal_equation_and_update_wrt_so3(self):
        """
        """
        # jacobi = self.jacobi_wrt_so3()    
        jacobi = self.numerical_jacobi_wrt_first_col()
        # print('jacobian', jacobi)
        r = self.residaul(self.var_SO4, self.var_projection)

        # rhs is invertable when rank == 1
        regulization = 0
        rhs = jacobi.transpose() @ jacobi + regulization * np.identity(3)
        lhs = - jacobi.transpose() @ r
        # print('rhs:', rhs)
        # print('lhs:', lhs)
        delta = np.linalg.solve(rhs, lhs)
        print('delta:', delta)
        self.var_SO4 = self.var_SO4 @ self.local_so4_first_col_to_SO4(delta)


    def cost(self):
        r = self.residaul(self.var_SO4, self.var_projection)
        return r.transpose() @ r

    def print_variable(self):
        print('cost:', self.cost())
        np.testing.assert_almost_equal(self.var_SO4.transpose() @ self.var_SO4, np.identity(4))
        print('Principal vector:', take_first_cols(self.var_SO4).transpose())
        for i in range(1):
            p = self.var_projection[i, :]
            print(p.transpose() @ p / self.num_data)

    def minimize(self):
        for iter in range(10):
            if self.cost() < 1e-8:
                break
            self.print_variable()
            self.compute_projection()
            self.solve_normal_equation_and_update_wrt_so3()


def point_statis(points):
    cov_stats = points @ points.transpose() / points.shape[1]
    e, v = np.linalg.eig(cov_stats)
    idx = np.argsort(e)[::-1]
    e = e[idx]
    v = v[:,idx]
    print('e(cov_stats):', e)
    print('v(cov_stats):', v)


def main():
    points = generate_point_cloud()

    pca = PCA_SO4_first_principle_component(points)
    print("pca.cost():", pca.cost())
    print("pca jocibian:", pca.numerical_jacobi_wrt_first_col())
    pca.minimize()

    point_statis(points)

    



if __name__ == "__main__":
    main()
