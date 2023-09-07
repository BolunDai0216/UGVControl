import pickle

import numpy as np

from unicycle_env import UnicycleEnv


def main():
    env = UnicycleEnv()
    state = env.reset()
    
    p_des = np.array([2, 2])
    kv = 0.5
    kω = 2.0
    
    states = []
    
    for i in range(10000):
        target_θ = np.arctan2(p_des[1] - env.state[1], p_des[0] - env.state[0])
        v = kv * np.sqrt(
            (p_des[0] - env.state[0]) ** 2 + (p_des[1] - env.state[1]) ** 2
        )
        ω = kω * (target_θ - env.state[2])
        control = np.array([v, ω])
        
        state = env.step(control)
        states.append(state)
    
    with open("unicycle_exp.pkl", "wb") as f:
        pickle.dump(states, f)
    


if __name__ == "__main__":
    main()
