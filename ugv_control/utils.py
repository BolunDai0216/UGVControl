import numpy as np
from scipy.spatial.transform import Rotation as R


def sigmoid(z):
    return 1 / (1 + np.exp(-z))


def get_yaw_error(yaw, yaw_des):
    R1 = R.from_euler("z", yaw).as_matrix()
    R2 = R.from_euler("z", yaw_des).as_matrix()
    R_error = R.from_matrix(R1.T @ R2)
    yaw_error = R_error.as_euler("zyx")[0]

    return yaw_error


def get_p_control(
    state, p_des, kp=0.1, kw=2.0, min_v=-1.0, max_v=1.0, min_w=-1.0, max_w=1.0
):
    p_error = np.linalg.norm(state[:2] - p_des)

    yaw_des = np.arctan2(p_des[1] - state[1], p_des[0] - state[0])
    yaw_error = get_yaw_error(state[2], yaw_des)

    v = np.clip(kp * p_error, min_v, max_v)
    w = np.clip(kw * yaw_error, min_w, max_w)

    control = np.array([v, w])

    return control, p_error, yaw_error
