import numpy as np
# from scipy.linalg import logm, expm
from math import cos, sin, pi
import matplotlib.pyplot as plt
import utils


def generate_point_cloud():
    mean = np.array([0, 0, 0.])
    
    # R = utils.euler_angle_to_rotation(0.4, -1., 6.6666)
    R = utils.euler_angle_to_rotation(0.3, 0.5, 0.0)
    # Transformation for covariance
    cov = R @ np.diag([0.001, 1, 9.]) @ R.transpose()
    num_point = 100
    points = np.random.multivariate_normal(mean, cov, size=num_point)

    points_mean = np.mean(points, axis=0)
    print('points_mean:', points_mean)
    points = points - points_mean

    np.testing.assert_almost_equal(R.transpose() @ R, np.identity(3))
    print('R_gt: ', R)
    print('cov_gt:', cov)
    print('numpy.linalg.eig(cov):', np.linalg.eig(cov))

    return points.transpose()


def take_n_cols(R, n):
    return R[:, :n]

class PCA_SO3_projection_minimization:
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
        self.var_SO3 = np.identity(3)
        self.rank = 1
        self.var_projection = np.zeros([self.rank, self.num_data])

    def residaul(self, var_SO3, var_projection):
        """
          r_i = p_i - first_k_cols(R) * w
        """
        # self.point.shape == (3, n)
        r = self.points - take_n_cols(var_SO3, self.rank) @ var_projection
        # make r col major
        return r.transpose().flatten()

    def compute_projection(self):
        self.var_projection = take_n_cols(self.var_SO3, self.rank).transpose() @ self.points

    def numerical_jacobi_wrt_so3(self):
        """
            r_i = p_i - first_k_cols(R) * w
        """
        DELTA = 1e-8

        jacobian = np.zeros([self.num_residuals, 3])

        w_so3_local = np.array([0, 0, 0.])
        curret_params = w_so3_local.copy()
        for p_idx in range(3):
            params_plus = curret_params.copy()
            params_plus[p_idx] += DELTA
            residual_plus = self.residaul(self.var_SO3 @ utils.so3_exp(params_plus), self.var_projection)

            params_minus = curret_params.copy()
            params_minus[p_idx] -= DELTA
            residual_minus = self.residaul(self.var_SO3 @ utils.so3_exp(params_minus), self.var_projection)

            dr_dpidx = (residual_plus - residual_minus) / (2. * DELTA)
            jacobian[:, p_idx] = dr_dpidx

        return jacobian

    def jacobi_wrt_so3(self):
        """
        The derivation is here, 
            r_i(w) = p_i - first_k_cols(R) * proj
                   = p_i - first_k_cols(R * exp(w)) * proj
                     (3,1)            (3, k)        (k, 1)
            let f3 = - first_k_cols(R * exp(w)) * proj
                f2 = first_k_cols(R * exp(w))
            f3 = - f2 * proj
            f3 =  - (proj.T * f2.T).T

            df3/ dw = - (proj.T * f2dw.T).T,  (3, 3)
                         (1,k)   ((k,3), 3) = (( 3*1), 3)

            let f2 = first_k_cols(R * exp(w))
                f1 = R * exp(w)

            df2/dw = df2/df1 * df1/dw, ((3,k), 3)

            df2/df1 = [1 when taking the element, 0 otherwise],   ((3,k), (3,3))

            df1/dw = [R * G1 , R * G2, R * G3], ((3,3), 3)
        """
        

        G1 = np.array([
            [0, 0, 0],
            [0, 0, -1],
            [0, 1, 0.],
        ])
        G2 = np.array([
            [0, 0, 1],
            [0, 0, 0],
            [-1, 0, 0.],
        ])
        G3 = np.array([
            [0, -1, 0],
            [1, 0, 0],
            [0, 0, 0.],
        ])
        f1 = np.zeros([3,3,3])
        f1[:, :, 0] = self.var_SO3 @ G1
        f1[:, :, 1] = self.var_SO3 @ G2
        f1[:, :, 2] = self.var_SO3 @ G3

        # ((k, 3), 3)
        f2_t = np.zeros([self.rank, 3, 3])
        for i in range(3):
            f2_t[:, :, i] = f1[:, :self.rank, i].transpose()

        jacobi_all_points = np.zeros([self.num_data, 3, 3])
        for i in range(3):
            #          ((n,3), 3)      =                (n, k)                  @    ((k, 3), 3) 
            jacobi_all_points[:, :, i] = - (self.var_projection.transpose() @ f2_t[:, :, i])

        jacobi = jacobi_all_points.reshape(self.num_data * 3, 3)
        return jacobi

    def solve_normal_equation_and_update_wrt_so3(self):
        """
        """
        jacobi = self.jacobi_wrt_so3()    
        # jacobi = self.numerical_jacobi_wrt_so3()
        # print('jacobian', jacobi)
        r = self.residaul(self.var_SO3, self.var_projection)

        # rhs is invertable when rank == 1
        regulization = 1e-6
        rhs = jacobi.transpose() @ jacobi + regulization * np.identity(3)
        lhs = - jacobi.transpose() @ r
        # print('rhs:', rhs)
        # print('lhs:', lhs)
        delta_so3 = np.linalg.solve(rhs, lhs)
        print('delta_so3:', delta_so3)
        self.var_SO3 = self.var_SO3 @ utils.so3_exp(delta_so3)


    def cost(self):
        r = self.residaul(self.var_SO3, self.var_projection)
        return r.transpose() @ r

    def print_variable(self):
        print('cost:', self.cost())
        np.testing.assert_almost_equal(self.var_SO3.transpose() @ self.var_SO3, np.identity(3))
        print('R_k:', take_n_cols(self.var_SO3, self.rank).transpose())
        for i in range(self.rank):
            p = self.var_projection[i, :]
            print(p.transpose() @ p / self.num_data)

    def minimize(self):
        for iter in range(10):
            if self.cost() < 1e-8:
                break
            self.print_variable()
            self.compute_projection()
            self.solve_normal_equation_and_update_wrt_so3()

            np.testing.assert_almost_equal(self.var_SO3.transpose() @ self.var_SO3, np.identity(3))


def main():
    points = generate_point_cloud()
    
    # pca = PCA_SO3_Covariance(points)
    # print("pca.cost():", pca.cost())
    # print("pca jocibian:", pca.numerical_jacobi())
    # pca.minimize()

    pca_2 = PCA_SO3_projection_minimization(points)

    pca_2.compute_projection()
    # pca_2.var_projection = np.ones([1, 1])
    # j1 = pca_2.jacobi_wrt_so3()
    # j2 =pca_2.numerical_jacobi_wrt_so3()
    # print('j1:', j1)
    # print('j2:', j2)
    # 1 / 0

    print("pca_2.cost()", pca_2.cost())
    pca_2.minimize()
    print('finial so3:', pca_2.var_SO3)

    cov_stats = points @ points.transpose() / points.shape[1]
    # print('cov_stats:', cov_stats)
    e, v = np.linalg.eig(cov_stats)
    idx = np.argsort(e)[::-1]
    e = e[idx]
    v = v[:,idx]
    print('e(cov_stats):', e)
    print('v(cov_stats):', v)

    pca_using_eig = PCA_SO3_projection_minimization(points)
    pca_using_eig.var_SO3 = v
    pca_using_eig.compute_projection()
    pca_using_eig.print_variable()
    print("pca_using_eig.cost() by svd:", pca_using_eig.cost())




if __name__ == "__main__":
    main()
