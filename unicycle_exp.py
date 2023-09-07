import numpy as np

from unicycle_env import UnicycleEnv


def main():
    env = UnicycleEnv()
    state = env.reset()
    
    p_des = np.array([2, 2])
    kv = 0.5
    kω = 2.0
    
    for i in range(10000):
        target_θ = np.arctan2(p_des[1] - env.state[1], p_des[0] - env.state[0])
        v = kv * np.sqrt(
            (p_des[0] - env.state[0]) ** 2 + (p_des[1] - env.state[1]) ** 2
        )
        ω = kω * (target_θ - env.state[2])
        control = np.array([v, ω])
        
        state = env.step(control)
        
        print(state)


if __name__ == "__main__":
    main()
