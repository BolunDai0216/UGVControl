import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MultipleLocator
from scipy.spatial.transform import Rotation as R

mpl.rcParams["text.usetex"] = True
mpl.rcParams["text.latex.preamble"] = r"\usepackage{amsmath}"
plt.rcParams["font.size"] = 16


def get_yaw_error(yaw, yaw_des):
    R1 = R.from_euler("z", yaw).as_matrix()
    R2 = R.from_euler("z", yaw_des).as_matrix()
    R_error = R.from_matrix(R1.T @ R2)
    yaw_error = R_error.as_rotvec()[-1]

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


def visualize_exp(data, img_name, xgrid=2.0, ygrid=2.0, show=False):
    states = np.concatenate(data["states"]).reshape(-1, 3)
    p_des = np.concatenate(data["p_des_list"]).reshape(-1, 2)

    fig, ax = plt.subplots(1, 1, figsize=(5, 5))

    ax.plot(
        states[:, 0],
        states[:, 1],
        color="cornflowerblue",
        linewidth=3,
        zorder=20,
        label="Trajectory",
    )
    ax.plot(
        p_des[:, 0],
        p_des[:, 1],
        "o",
        color="darkorange",
        markersize=5,
        zorder=10,
        label="Waypoints",
    )

    ax.xaxis.set_major_locator(MultipleLocator(xgrid))
    ax.xaxis.set_minor_locator(MultipleLocator(xgrid / 4))
    ax.yaxis.set_major_locator(MultipleLocator(ygrid))
    ax.yaxis.set_minor_locator(MultipleLocator(ygrid / 4))

    ax.grid(True, "minor", color="0.85", linewidth=0.50, zorder=-20)
    ax.grid(True, "major", color="0.65", linewidth=0.75, zorder=-10)
    ax.tick_params(which="both", bottom=False, left=False)

    ax.set_xlim([states[:, 0].min() - 1.0, states[:, 0].max() + 1.0])
    ax.set_ylim([states[:, 1].min() - 1.0, states[:, 1].max() + 1.0])
    ax.set_aspect("equal")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.legend(frameon=False, ncol=2, loc="upper center", bbox_to_anchor=(0.5, 1.6))

    plt.savefig(
        img_name,
        dpi=200,
        transparent=False,
        bbox_inches="tight",
    )

    if show:
        plt.show()
