from sklearn.datasets import load_iris
from sklearn.linear_model import LogisticRegression
import numpy as np
import variables_tree_flow as vtf


class MyLogisticRegression:
    def fit(self, X, y):
        return self.__fit_numpy_impl(X, y)

    def __fit_numpy_impl(self, X, y):
        assert len(X) == len(y)
        # bias
        ones = np.ones((X.shape[0], 1))
        X = np.hstack([X, ones])

        self.data_num = len(X)
        self.num_classes = max(y) + 1
        self.len_theta = X.shape[1]

        theta = np.array([vtf.Variable(value=0., id='t{}'.format(i))
                          for i in range(self.len_theta * self.num_classes)]).reshape([self.len_theta, self.num_classes])
        linear_comb = X @ theta
        pred = np.vectorize(vtf.ntf_sigmoid)(linear_comb)

        cost = 0
        for pred_row, gt_class in zip(pred, y):
            for class_idx, pred_class in enumerate(pred_row):
                # idx encoding
                # multiple logistic regression to handle multiclasses.
                if class_idx == gt_class:
                    cost += - vtf.ntf_log(pred_class)
                else:
                    cost += - vtf.ntf_log(1 - pred_class)
        cost = cost * (1 / self.data_num)

        core = vtf.NumberFlowCore(cost)
        for i in range(2000):
            core.forward()
            if i % 200 == 0:
                print("cost.val:", cost.value, " iter:", i)
            core.clear_grad()
            core.backward()
            core.gradient_desent(rate=0.001)

        for var in set(core.varible_nodes):
            print(var.id, var.value)

        self.coef_ = np.array([[theta.value for theta in row]
                               for row in theta])

    def predict(self, X):
        res = X @ self.coef_[:-1, :] + self.coef_[-1, :]
        return np.argmax(res, axis=1)


if __name__ == "__main__":

    X, y = load_iris(return_X_y=True)

    mlg = MyLogisticRegression()
    mlg.fit(X, y)
    pred = mlg.predict(X)
    print('pred:', pred)
    print('y_gt:', y)
    print('train accuracy:', np.sum((y == pred)) / len(y))
