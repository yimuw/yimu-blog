import number_tree_flow as ntf
import numpy as np


class LinearRegression:
    def __init__(self):
        pass

    def fit(self, X, y):
        return self.__fit_numpy_impl(X,y)

    def __fit_numpy_impl(self, X, y):
        assert len(X) == len(y)
        self.data_num = len(X)
        self.len_theta = X.shape[1]

        theta = np.array([ntf.Number(value=0, id='t{}'.format(i))
                 for i in range(self.len_theta)])
        bias = ntf.Number(value=0, id='b')

        pred = X @ theta + bias
        diff = pred - y
        cost = diff.T @ diff * (1 / len(X))

        core = ntf.NumberFlowCore(cost)
        for i in range(15000):
            core.forward()
            if i % 500 == 0:
                print("cost.val:", cost.value, " iter:", i)
            core.clear_grad()
            core.backward()
            core.gradient_desent(rate=0.001)

        for var in set(core.varible_nodes):
            print(var.id, var.value)

        self.coef_ = np.array([t.value for t in theta])
        self.intercept_ = bias.value

    def __fit_iter_impl(self, X, y):
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
        for i in range(15000):
            core.forward('recur')
            if i % 500 == 0:
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


# this is basically a sklearn linear regression example.
# change the METHOD to see difference.
from sklearn.metrics import mean_squared_error, r2_score
from sklearn import datasets, linear_model
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    # 'sklearn' or 'ntf'
    METHOD = 'ntf'

    # Create linear regression object
    if METHOD == 'sklearn':
        regr = linear_model.LinearRegression()
    elif METHOD == 'ntf':
        regr = LinearRegression()

    # Load the diabetes dataset
    diabetes_X, diabetes_y = datasets.load_diabetes(return_X_y=True)

    # Use only one feature
    diabetes_X = diabetes_X[:, np.newaxis, 2]

    # Split the data into training/testing sets
    diabetes_X_train = diabetes_X[:-400]
    diabetes_X_test = diabetes_X[-400:]

    # Split the targets into training/testing sets
    diabetes_y_train = diabetes_y[:-400]
    diabetes_y_test = diabetes_y[-400:]

    # Train the model using the training sets
    regr.fit(diabetes_X_train, diabetes_y_train)

    # Make predictions using the testing set
    diabetes_y_pred = regr.predict(diabetes_X_test)

    # # The coefficients
    print('Coefficients:', regr.coef_)
    print('Intercept:', regr.intercept_)

    # Plot outputs
    plt.scatter(diabetes_X_test, diabetes_y_test,  color='black')
    plt.plot(diabetes_X_test, diabetes_y_pred, color='blue', linewidth=3)

    plt.xticks(())
    plt.yticks(())

    plt.show()
