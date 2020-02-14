import numpy as np
from matplotlib import pyplot as plt


class DiseaseModel:
    def __init__(self, infection_para_per_hour):
        self.infection_para_per_hour = infection_para_per_hour
        self.recover_para_per_hour = 0.005
        self.dead_para_per_hour = 0.0005

    def dynamic(self, state):
        susceptible, infected,  immune, dead = state

        # 一个被感染者一小时可以感染多少人
        infection_rate = self.infection_para_per_hour * susceptible

        # 因为以上 self.infection_para_per_hour * susceptible 非常粗糙。
        # 比如当susceptible=100000， infection_para_per_hour=0.0001的时候，
        # self.infection_para_per_hour * susceptible=100是一个很大的数。
        # 也就在一个被感染者一小时可以感染100个人。
        # 有点不现实。
        # 于是当self.infection_para_per_hour * susceptible太大的时候，
        # 让他的最大值为一个定值。
        # 实际意义就是一个人一天接触的人有限。
        if self.infection_para_per_hour * susceptible > 0.5:
            infection_rate = 0.5

        new_infected = infection_rate * infected

        print('new_infected:', new_infected)

        next_susceptible = susceptible - new_infected
        next_infected = infected + new_infected \
            - self.recover_para_per_hour * infected - self.dead_para_per_hour * infected
        next_immune = immune + self.recover_para_per_hour * infected
        next_dead = dead + self.dead_para_per_hour * infected

        next_state = np.array(
            [next_susceptible, next_infected, next_immune, next_dead])

        if next_susceptible < 0:
            return state.copy()
        else:
            return next_state


def simulation(init_susceptible, init_infected, infection_para_per_hour, total_hours):
    init_recover = 0
    init_dead = 0

    init_state = np.array(
        [init_susceptible, init_infected, init_recover, init_dead])

    disease_model = DiseaseModel(infection_para_per_hour)

    all_states = [init_state]
    cur_state = init_state

    for i in range(total_hours):
        next_state = disease_model.dynamic(cur_state)
        all_states.append(next_state)
        cur_state = next_state

    all_states = np.vstack(all_states)

    print('final state: susceptible, infected,  immune, dead', cur_state)
    time = np.linspace(0, total_hours / 24, total_hours + 1)
    plt.plot(time, all_states[:, 0], "r", label=u"Susceptible")
    plt.plot(time, all_states[:, 1], "g", label=u"Infective")
    plt.plot(time, all_states[:, 2], "b", label=u"Immune")
    plt.plot(time, all_states[:, 3], "m", label=u"Dead")
    plt.title('disease model')
    plt.xlabel('time(days)')
    plt.ylabel('X10000 people')
    plt.title('init-state: infection para: {}, (susceptible:{:.2f}, infective:{:.2f})\n\
               end-state: susceptible:{:.2f}, infective:{:.2f}, covered:{:.2f}, dead:{:.2f}'.format(
               infection_para_per_hour, init_susceptible, init_infected, \
               cur_state[0], cur_state[1], cur_state[2], cur_state[3]))
    plt.legend()
    plt.show()


if __name__ == "__main__":
    simulation(init_susceptible=1000, init_infected=0.1,
               infection_para_per_hour=0.0001, total_hours=24 * 60)

    simulation(init_susceptible=1000, init_infected=0.1,
               infection_para_per_hour=0.00001, total_hours=24 * 180)

    simulation(init_susceptible=1000, init_infected=0.1,
               infection_para_per_hour=0.000005, total_hours=24 * 180)
