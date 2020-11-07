import variables_tree_flow as vtf
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

        theta = np.array([vtf.Variable(value=0, id='t{}'.format(i))
                 for i in range(self.len_theta)])
        bias = vtf.Variable(value=0, id='b')

        pred = X @ theta + bias
        diff = pred - y
        cost = diff.T @ diff * (1 / len(X))

        core = vtf.NumberFlowCore(cost)
        for i in range(20000):
            core.forward()
            if i % 500 == 0:
                print("cost.val:", cost.value, " iter:", i)
            core.clear_grad()
            core.backward()
            core.gradient_desent(rate=0.1)

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
    # 'sklearn' or 'vtf'
    METHOD = 'vtf'

    # Create linear regression object
    if METHOD == 'sklearn':
        regr = linear_model.LinearRegression()
    elif METHOD == 'vtf':
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
