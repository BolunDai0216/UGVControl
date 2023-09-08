import pickle

import numpy as np
import pandas as pd

from unicycle_env import UnicycleEnv
from utils import get_p_control


def main():
    env = UnicycleEnv(dt=0.01)
    np.set_printoptions(precision=3)

    df = pd.read_csv("data/path.csv")
    p_des_array = df[["x", "y"]].values

    state = env.reset(set_init_state=[p_des_array[0, 0], p_des_array[0, 1], np.pi])

    p_des_index = 20
    p_des = p_des_array[p_des_index, :]
    p_des_list = [p_des]

    states = []
    controls = []

    for i in range(100000):
        control, p_error, yaw_error = get_p_control(state, p_des, kp=0.1, kw=1.0)
        state = env.step(control)

        states.append(state)
        controls.append(control)

        if p_error < 0.01:
            print(i, state, control, p_des, p_error)

            if p_des_index == p_des_array.shape[0] - 1:
                break
            else:
                p_des_index = np.minimum(p_des_index + 20, p_des_array.shape[0] - 1)
                p_des = p_des_array[p_des_index, :]
                p_des_list.append(p_des)

    data = {
        "states": states,
        "p_des_list": p_des_list,
        "controls": controls,
    }

    with open("../data/unicycle_exp.pkl", "wb") as f:
        pickle.dump(data, f)


if __name__ == "__main__":
    main()
