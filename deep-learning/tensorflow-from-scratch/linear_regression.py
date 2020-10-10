from sklearn.metrics import mean_squared_error, r2_score
from sklearn import datasets, linear_model
import numpy as np
import matplotlib.pyplot as plt
import lr_by_ntf

# 'sklearn' or 'ntf'
METHOD = 'ntf'

# Create linear regression object
if METHOD == 'sklearn':
    regr = linear_model.LinearRegression()
elif METHOD == 'ntf':
    regr = lr_by_ntf.LinearRegression()

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
