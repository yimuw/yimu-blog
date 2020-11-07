from sklearn.datasets import load_iris
from sklearn.linear_model import LogisticRegression
from sklearn import datasets
import numpy as np
import variables_tree_flow as vtf
import matplotlib.pyplot as plt
import random


class NeuralNet:
    def fit_and_test(self, images, labels):
        assert len(images) == len(labels)

        num_data = len(labels)

        len0 = 64
        len1 = 10
        len2 = 10
        self.theta0 = np.array([vtf.Variable(value=random.gauss(0, 0.01), id='t{}'.format(i))
                                for i in range((len0 + 1) * len1)]).reshape([len1, (len0 + 1)])
        self.theta1 = np.array([vtf.Variable(value=random.gauss(0, 0.01), id='t{}'.format(i))
                                for i in range((len1 + 1) * len2)]).reshape([len2, (len1 + 1)])

        cost = 0
        for i, (im, label) in enumerate(zip(images, labels)):
            if i == 2:
                break
            pred = self.__predict_func(im)
            assert(len(pred) == 10)
            cost_this = 0
            for idx, pred_val in enumerate(pred):
                # idx encoding
                # multiple logistic regression to handle multiclasses.
                if idx == label:
                    cost_this += - vtf.ntf_log(pred_val)
                else:
                    cost_this += - vtf.ntf_log(1 - pred_val)
            cost += cost_this * (1 / num_data)

        core = vtf.NumberFlowCore(cost)
        for i in range(10000):
            core.forward()
            if i % 100 == 0:
                print("cost.val:", cost.value, " iter:", i)

            core.clear_grad()
            core.backward()
            core.gradient_desent(rate=0.05)

        # replace graph node type by np array
        self.theta0 = [[e.value for e in r] for r in self.theta0]
        self.theta1 = [[e.value for e in r] for r in self.theta1]

        for i in range(2):
            print(self.predict(images[i]))
            print('gt:', labels[i])

    def __predict_func(self, data):
        x0 = data.reshape(-1)

        x0b = np.hstack([x0, 1])
        z0 = self.theta0 @ x0b
        x1 = np.vectorize(vtf.ntf_sigmoid)(z0)
        x1b = np.hstack([x1, 1])
        z1 = self.theta1 @ x1b
        x2 = np.vectorize(vtf.ntf_sigmoid)(z1)

        return x2

    def predict(self, data):
        print('========================')
        pred = self.__predict_func(data)
        print('pred:', pred)
        return np.argmax(pred, axis=0)


class LogisticRegression:
    def fit_and_test(self, images, labels):
        assert len(images) == len(labels)

        num_data = len(labels)

        len0 = 64
        len1 = 10
        len2 = 10
        self.theta0 = np.array([vtf.Variable(value=random.gauss(0, 0.01), id='t{}'.format(i))
                                for i in range((len0 + 1) * len1)]).reshape([len1, (len0 + 1)])
        self.theta1 = np.array([vtf.Variable(value=random.gauss(0, 0.01), id='t{}'.format(i))
                                for i in range((len1 + 1) * len2)]).reshape([len2, (len1 + 1)])

        cost = 0
        for i, (im, label) in enumerate(zip(images, labels)):
            if i == 10:
                break
            pred = self.__predict_func(im)
            assert(len(pred) == 10)
            cost_this = 0
            for idx, pred_val in enumerate(pred):
                # idx encoding
                # multiple logistic regression to handle multiclasses.
                if idx == label:
                    cost_this += - vtf.ntf_log(pred_val)
                else:
                    cost_this += - vtf.ntf_log(1 - pred_val)
            cost += cost_this * (1 / num_data)

        core = vtf.NumberFlowCore(cost)
        for i in range(1000):
            core.forward()
            if i % 100 == 0:
                print("cost.val:", cost.value, " iter:", i)

            core.clear_grad()
            core.backward()
            core.gradient_desent(rate=0.05)

        # replace graph node type by np array
        self.theta0 = [[e.value for e in r] for r in self.theta0]

        for i in range(10):
            print(self.predict(images[i]))
            print('gt:', labels[i])

    def __predict_func(self, data):
        x0 = data.reshape(-1)

        x0b = np.hstack([x0, 1])
        z0 = self.theta0 @ x0b
        x1 = np.vectorize(vtf.ntf_sigmoid)(z0)
        return x1

    def predict(self, data):
        print('========================')
        pred = self.__predict_func(data)
        print('pred:', pred)
        return np.argmax(pred, axis=0)


if __name__ == "__main__":

    # The digits dataset
    digits = datasets.load_digits()

    # The data that we are interested in is made of 8x8 images of digits, let's
    # have a look at the first 4 images, stored in the `images` attribute of the
    # dataset.  If we were working from image files, we could load them using
    # matplotlib.pyplot.imread.  Note that each image must have the same size. For these
    # images, we know which digit they represent: it is given in the 'target' of
    # the dataset.
    _, axes = plt.subplots(2, 4)
    images_and_labels = list(zip(digits.images, digits.target))
    for ax, (image, label) in zip(axes[0, :], images_and_labels[:4]):
        ax.set_axis_off()
        ax.imshow(image, cmap=plt.cm.gray_r, interpolation='nearest')
        ax.set_title('Training: %i' % label)
    plt.show()

    # nn = NeuralNet()
    # nn.fit_and_test(digits.images, digits.target)

    lg = LogisticRegression()
    lg.fit_and_test(digits.images, digits.target)

