import numpy as np
import matplotlib.pyplot as plt



def generate_quadratic_data():
    quadratic_a = 2.4
    quadratic_b = -2.
    quadratic_c = 1.

    num_data = 100

    noise = np.random.randn(num_data)

    sampled_x = np.linspace(-10, 10., num_data)
    sampled_y = quadratic_a + sampled_x * sampled_x + quadratic_b * sampled_x + quadratic_c + noise

    return sampled_x, sampled_y


def main():
    x, y = generate_quadratic_data()
    plt.plot(x, y)
    plt.title('Data points')
    plt.show()

if __name__ == "__main__":
    main()
