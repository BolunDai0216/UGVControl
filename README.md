# UGVControl

## Dynamics

We model the UGV as a unicycle with the dynamics

$$
\begin{bmatrix}
    \dot{x}\\
    \dot{y}\\
    \dot{\theta}
\end{bmatrix} = \begin{bmatrix}
    v\cos\theta\\
    v\sin\theta\\
    \omega
\end{bmatrix}.
$$

where $x\in\mathbb{R}$ and $y\in\mathbb{R}$ represents the position of the UGV in the world frame, $\theta\in\mathbb{R}$ is the heading of the UGV, $v\in\mathbb{R}$ is the linear velocity, and $\omega\in\mathbb{R}$ is the angular velocity.

## Control Algorithm

The control algorithm is a simple proportional controller with 

$$v = K_v\|\Delta p\|,\quad \omega = K_\omega\Delta\theta.$$
 
The positional error $\Delta p$ is computed as

$$
\Delta p = \begin{bmatrix}
    \Delta x\\
    \Delta y
\end{bmatrix} = \begin{bmatrix}
    x_\mathrm{des} - x\\
    y_\mathrm{des} - y
\end{bmatrix}.
$$

To compute the heading error $\Delta\theta$, we first compute the desired heading $\theta_\mathrm{des}$ as

$$\theta_\mathrm{des} = \mathrm{atan2}(\Delta y, \Delta x).$$

Then, we have the relationship between the body orientation $R_\mathcal{B} \in \mathrm{SO}(3)$, the orientation error represented as a rotation matrix $R_\epsilon \in \mathrm{SO}(3)$, and the desired heading $R_\mathrm{des} \in \mathrm{SO}(3)$ as

$$R_\mathcal{B}R_\epsilon = R_\mathrm{des}.$$

Then, we get the orientation error as

$$R_\epsilon = R_\mathcal{B}^\top R_\mathrm{des} = \begin{bmatrix}
    \cos\Delta\theta & -\sin\Delta\theta & 0\\
    \sin\Delta\theta & \cos\Delta\theta & 0\\
    0 & 0 & 1
\end{bmatrix}.$$

Note that when tuning $K_v$ and $K_\omega$, we must ensure that $K_v << K_\omega$ to ensure stability, see [here](https://www.youtube.com/watch?v=Lgy92yXiyqQ) for a nice explanation of why this helps. 