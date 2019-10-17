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
DIM = 10

def generate_data(dim, num_point = 500):
    mean = np.zeros(dim)
    
    # exp of skew symatric mat is SO(N)
    Mat = np.random.rand(dim, dim)
    W = Mat.transpose() - Mat
    R = expm(W)
    np.testing.assert_almost_equal(R.transpose() @ R, np.identity(dim))

    # Transformation for covariance
    principal_axis = 0.01 * np.ones(dim)
    principal_axis[0] = 1.
    principal_axis[1] = 0.3
    principal_axis[2] = 0.2

    cov = R @ np.diag(principal_axis) @ R.transpose()
    points = np.random.multivariate_normal(mean, cov, size=num_point)

    # For simplicity
    points_mean = np.mean(points, axis=0)
    points = points - points_mean

    return points.transpose()


def take_first_cols(R):
    return R[:, :1]

class PCAHighDimFirstPrincipleComponent:
    """
    https://stats.stackexchange.com/questions/10251/what-is-the-objective-function-of-pca
    """
    def __init__(self, points, dim):
        self.num_data = points.shape[1]
        self.num_residuals = points.size
        points_mean = np.mean(points, axis=1)
        self.points = (points.transpose() - points_mean).transpose()
        self.dim = dim

        self.var_SO = np.identity(dim)
        self.var_projection = np.zeros([1, self.num_data])

    def residaul(self, var_SO, var_projection):
        """
          r_i = p_i - first_k_cols(R) * w
        """
        # self.point.shape == (3, n)
        r = self.points - take_first_cols(var_SO) @ var_projection
        # make r col major
        return r.transpose().flatten()

    def compute_projection(self):
        self.var_projection = take_first_cols(self.var_SO).transpose() @ self.points

    def hat_local_so_first_col(self, local_params):
        """
        Take SO3 as example
         W = [[0   -w1  -w2],
              [w1  0    -w3],
              [w2  w3   0 ]]
         local_params = [w1, w2]
        """
        W = np.zeros([self.dim, self.dim])
        for i in range(1, self.dim):
            W[i, 0] = local_params[i - 1]
            W[0, i] = -local_params[i - 1]

        return W

    def local_so_first_col_to_SO(self, local_params):
        W = self.hat_local_so_first_col(local_params)
        return expm(W)

    def numerical_jacobi_wrt_first_col(self):
        """
            r_i = p_i - first_k_cols(R) * w
        """
        DELTA = 1e-8
        num_local_variables = self.dim - 1
        jacobian = np.zeros([self.num_residuals, num_local_variables])

        w_so_local_first_col = np.zeros(num_local_variables)
        curret_params = w_so_local_first_col.copy()
        for p_idx in range(num_local_variables):
            params_plus = curret_params.copy()
            params_plus[p_idx] += DELTA
            residual_plus = self.residaul(self.var_SO @ self.local_so_first_col_to_SO(params_plus), self.var_projection)

            params_minus = curret_params.copy()
            params_minus[p_idx] -= DELTA
            residual_minus = self.residaul(self.var_SO @ self.local_so_first_col_to_SO(params_minus), self.var_projection)

            dr_dpidx = (residual_plus - residual_minus) / (2. * DELTA)
            jacobian[:, p_idx] = dr_dpidx

        return jacobian

    def jacobi_wrt_so(self):
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

        num_variables = self.dim - 1

        f2_t = np.zeros([self.dim,num_variables])
        for var_idx in range(num_variables):
            f2_t[:, var_idx] = self.var_SO[:, var_idx + 1]
        f2_t = f2_t.reshape(1, self.dim, num_variables)

        jacobi_all_points = np.zeros([self.num_data, self.dim, num_variables])
        for i in range(num_variables):
            #          ((n,3), 3)      =                (n, k)                  @    ((k, 3), 3) 
            jacobi_all_points[:, :, i] = - (self.var_projection.transpose() @ f2_t[:,:, i])

        jacobi = jacobi_all_points.reshape(self.num_data * self.dim, num_variables)
        return jacobi

    def solve_normal_equation_and_update_wrt_so(self):
        """
        """
        jacobi = self.jacobi_wrt_so()   

        if False: 
            jacobi_nu = self.numerical_jacobi_wrt_first_col()
            print('jacobi checking:', jacobi_nu - jacobi)
        
        # print('jacobian', jacobi)
        r = self.residaul(self.var_SO, self.var_projection)

        rhs = jacobi.transpose() @ jacobi
        lhs = - jacobi.transpose() @ r
        # print('rhs:', rhs)
        # print('lhs:', lhs)
        delta = np.linalg.solve(rhs, lhs)
        self.var_SO = self.var_SO @ self.local_so_first_col_to_SO(delta)

        return delta


    def cost(self):
        r = self.residaul(self.var_SO, self.var_projection)
        return r.transpose() @ r

    def print_variable(self):
        print('cost:', self.cost())
        np.testing.assert_almost_equal(self.var_SO.transpose() @ self.var_SO, np.identity(self.dim))
        print('Principal vector:', take_first_cols(self.var_SO).transpose())
        p = self.var_projection[0, :]
        print(p.transpose() @ p / self.num_data)

    def minimize(self):
        for iter in range(10):
            print('iter:', iter)
            self.print_variable()
            self.compute_projection()
            delta = self.solve_normal_equation_and_update_wrt_so()
            if np.linalg.norm(delta) < 1e-4:
                break


def point_statis(points):
    cov_stats = points @ points.transpose() / points.shape[1]
    e, v = np.linalg.eig(cov_stats)
    idx = np.argsort(e)[::-1]
    e = e[idx]
    v = v[:,idx]
    print('largest e(cov_stats):', e[0])
    print('largest v(cov_stats):', v[:, 0].transpose())


def main():
    dim = 100
    points = generate_data(dim)

    pca = PCAHighDimFirstPrincipleComponent(points, dim)
    print("pca.cost():", pca.cost())
    print("pca jocibian:", pca.numerical_jacobi_wrt_first_col())
    pca.minimize()

    point_statis(points)

    



if __name__ == "__main__":
    main()
