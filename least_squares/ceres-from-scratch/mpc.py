from number_forward_flow import *


def motion_case():
    print('=============== motion (nonlinear) case ==============')

    def motion_func(state):
        x, y, v, theta = state

        xnext = x + cosine(theta) * v
        ynext = y + sine(theta) * v

        return [xnext, ynext, v, theta]

    def observation_func(state):
        x, y, v, theta = state
        return x, y

    s0 = [0., 0., 0.5, math.pi / 9]
    s1 = motion_func(s0)

    o0 = observation_func(s0)
    o1 = observation_func(s1)

    v0 = [0., 0., 1., math.pi / 4]
    v1 = [0., 0., 0., 0.]
    states = v0 + v1

    def motion_residaul(state0andstate1):
        state0 = state0andstate1[:4]
        state1 = state0andstate1[4:]
        pred = motion_func(state0)
        return [b - a for a, b in zip(pred, state1)]

    def observation_residual0(state0andstate1):
        state0 = state0andstate1[:4]
        pred = observation_func(state0)
        return [a - b for a, b in zip(pred, o0)]

    def observation_residual1(state0andstate1):
        state1 = state0andstate1[4:]
        pred = observation_func(state1)
        return [a - b for a, b in zip(pred, o1)]

    for iter in range(10):
        full_jacobian = np.zeros([0, 8])
        residaul = np.zeros([0])

        r, J = ResidualBlock(observation_residual0, states).evaluate()
        r, J = np.array(r).T, np.array(J)
        residaul = np.hstack([residaul, r])
        full_jacobian = np.vstack([full_jacobian, J])

        r, J = ResidualBlock(observation_residual1, states).evaluate()
        r, J = np.array(r).T, np.array(J)
        residaul = np.hstack([residaul, r])
        full_jacobian = np.vstack([full_jacobian, J])

        r, J = ResidualBlock(motion_residaul, states).evaluate()
        r, J = np.array(r).T, np.array(J)
        residaul = np.hstack([residaul, r])
        full_jacobian = np.vstack([full_jacobian, J])

        delta = np.linalg.solve(
            full_jacobian.T @ full_jacobian, - full_jacobian.T @ residaul)
        states = [s + 0.8 * d for s, d in zip(states, delta)]
        print('cost:', residaul.T @ residaul)

    print('result s0:', states[:4])
    print('gt     s0:', s0)
    print('result s1:', states[4:])
    print('gt     s1:', s1)


if __name__ == "__main__":
    motion_case()
