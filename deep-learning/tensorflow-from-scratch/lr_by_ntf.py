import number_tree_flow as ntf
import numpy as np


class LinearRegression:
    def __init__(self):
        pass

    def fit(self, X, y):
        assert len(X) == len(y)
        self.data_num = len(X)
        self.len_theta = X.shape[1]

        theta = [ntf.Number(value=0, id='t{}'.format(i))
                 for i in range(self.len_theta)]
        bias = ntf.Number(value=0, id='b')

        cost = ntf.Number(value=0, id='cost', ntype='intermediate')
        for i in range(self.data_num):
            data = X[i]
            label = ntf.Number(y[i], id='y{}'.format(i), ntype='const')
            linear_comb = ntf.Number(
                value=0, id='diff{}'.format(i), ntype='intermediate')
            for d in range(self.len_theta):
                f = ntf.Number(
                    data[d], id='d{}_{}'.format(i, d), ntype='const')
                linear_comb += f * theta[d]
            linear_comb += bias

            diff = linear_comb - label
            cost += diff * diff

        scale = ntf.Number(value=1 / self.data_num, id='scale', ntype='const')
        cost *= scale

        core = ntf.NumberFlowCore(cost)
        for i in range(1000):
            core.forward('recur')
            if i % 50 == 0:
                print("cost.val:", cost.value, " iter:", i)
            core.clear_grad()
            core.backward()
            core.gradient_desent(rate=0.001)

        for var in set(core.varible_nodes):
            print(var.id, var.value)

        self.coef_ = np.array([t.value for t in theta])
        self.intercept_ = bias.value

    def predict(self, X):
        res = X @ self.coef_ + self.intercept_
        return res
