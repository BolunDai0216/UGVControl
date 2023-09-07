import copy
import math

import numpy as np
from scipy.spatial.transform import Rotation as R


class UnicycleEnv:
    def __init__(self, dt=0.1):
        # initialize all member attributes
        self.state = None
        self.robot_r = None
        self.robot_q = None
        self.dt = dt

    def reset(self, set_init_state=None):
        """
        The robot state is [x, y, θ]
        """

        if set_init_state is not None:
            self.state = np.array(
                [set_init_state[0], set_init_state[1], set_init_state[2]]
            )
        else:
            self.state = np.array([-1.0, -3.0, np.pi / 4])

        self.dstate = np.zeros(3)

        self.F_mat = np.array(
            [[np.cos(self.state[2]), 0.0], [np.sin(self.state[2]), 0.0], [0.0, 1.0]]
        )

        return copy.deepcopy(self.state)

    def step(self, action):
        """safe_control
        The system dynamics is:
        dx = vcos(θ)
        dy = vsin(θ)
        dθ = ω

        The control action is [v, ω].
        """
        if action.shape == (2,):
            action = action[:, np.newaxis]

        self.F_mat = np.array(
            [[np.cos(self.state[2]), 0.0], [np.sin(self.state[2]), 0.0], [0.0, 1.0]]
        )
        self.dstate = (self.F_mat @ action)[:, 0]

        nextstate = self.state + self.dstate * self.dt

        nextstate[2] = math.remainder(nextstate[2], 2 * math.pi)

        self.state = nextstate

        return copy.deepcopy(self.state)
