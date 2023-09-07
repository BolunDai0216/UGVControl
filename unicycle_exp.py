import pickle

import numpy as np
import pandas as pd

from unicycle_env import UnicycleEnv


def main():
    env = UnicycleEnv()
    
    df = pd.read_csv('data/path.csv')
    p_des_list = df[["x", "y"]].values
    p_des_index = -1
    p_des = p_des_list[p_des_index, :]
    
    state = env.reset(set_init_state=[20, 8, 0])

    kv = 0.1
    kω = 2.0
    
    states = []
    controls = []
    
    for i in range(10000):
        target_θ = np.arctan2(p_des[1] - env.state[1], p_des[0] - env.state[0])
        p_error = np.sqrt(
            (p_des[0] - env.state[0]) ** 2 + (p_des[1] - env.state[1]) ** 2
        )
        θ_error = target_θ - env.state[2]
        
        if np.abs(θ_error) < 0.5:
            v = np.clip(kv * p_error, -1.5, 1.5)
            ω = 0.0
            # breakpoint()
        else:
            v = 0.0
            ω = np.clip(kω * (target_θ - env.state[2]), -0.5, 0.5)
            
        control = np.array([v, ω])
        
        if np.max(control) >= 2:
            breakpoint()
            print("Max Control Exceeds 1, Exiting...")
            break
        
        state = env.step(control)
        
        states.append(state)
        controls.append(control)
        
        # if p_error < 0.000001:
        #     p_des_index = np.min([p_des_index + 1, len(p_des_list) - 1])
        #     p_des = p_des_list[p_des_index]
        
        # if i % 100 == 0:
        #     p_des_index = np.min([p_des_index + 1, len(p_des_list) - 1])
        #     p_des = p_des_list[p_des_index]
        
        print(state, p_des)
            
    data = {
        "states": states,
        "p_des_list": p_des_list,
        "controls": controls,
    }
    
    with open("unicycle_exp.pkl", "wb") as f:
        pickle.dump(data, f)
    


if __name__ == "__main__":
    main()
