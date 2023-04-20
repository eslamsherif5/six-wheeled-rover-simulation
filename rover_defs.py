from math import *
import numpy as np
import matplotlib.pyplot as pyplot
from tictoc import tic, toc


class dotdict(dict):
    """dot.notation access to dictionary attributes"""
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__


def J_x_psi(b, dims, rho):
    return b * dims.k3 * cos(rho)


def J_x_theta(b, dims, rho, psi, delta):
    return dims.k10 * (b * sin(rho) * sin(delta) + cos(rho)+cos(psi * cos(delta)))


def J_x_delta12(b, dims, rho, psi):
    return dims.k5 * cos(rho) * cos(psi) + b * dims.k4 * sin(rho) * cos(psi) - dims.k3 * sin(rho) * sin(psi) + dims.k1 * cos(psi)


def J_y_psi(b, dims, rho):
    return dims.k4 + dims.k2 * cos(rho) + b


def J_y_theta(dims, psi, delta):
    return dims.k10 * sin(psi) * cos(delta)


def J_y_delta(b, dims, rho, psi):
    return -dims.k5 * sin(psi) + dims.k1 * cos(rho) * sin(psi) - b * dims.k2 * sin(rho) * sin(psi)


def J_z_psi(dims, rho):
    return dims.k3 * sin(rho)


def J_z_theta(b, dims, rho, psi, delta):
    return dims.k1*(b * sin(rho) * cos(psi) * cos(delta) - cos(rho) * sin(delta))


def J_z_delta12(b, dims, rho, psi):
    return -b * dims.k5 * sin(rho) * cos(psi) - dims.k4 * cos(rho) * cos(psi) + b * dims.k3 * cos(rho) * sin(psi) - dims.k2 * cos(psi)


def J_yaw_psi(b, rho):
    return b * sin(rho)


def J_yaw_delta(rho, psi):
    return cos(rho) * sin(psi)


def J_pitch_delta(psi):
    return -cos(psi)


def J_roll_psi(rho):
    return -cos(rho)


def J_roll_delta(b, rho, psi):
    return b * sin(rho) * sin(psi)


def J_x_beta1(b, dims, rho, delta):
    return 0.5*(b-1) * (dims.k6 * sin(dims.k9+rho)-dims.k1)


def J_x_beta2(b, dims, rho, delta):
    return -0.5*(b+1) * (dims.k6 * sin(dims.k9-rho)-dims.k1)


def J_x_delta3456(b, dims, rho, beta1, beta2, delta, whl_cfg):
    if whl_cfg == 3 or whl_cfg == 4:
        a = dims.k7
        sigma = rho + beta1 + delta
    elif whl_cfg == 5 or whl_cfg == 6:
        a = -dims.k7
        sigma = -rho + beta2 + delta
    return -dims.k6 * sin(dims.k9 - b * rho) - a * sin(sigma - delta) - dims.k8 * cos(sigma - delta) + dims.k1


def J_z_beta1(b, dims, rho):
    return 0.5 * (b - 1) * (dims.k6 * cos(dims.k9 + rho) + dims.k2)


def J_z_beta2(b, dims, rho):
    return -0.5 * (b + 1) * (dims.k6 * cos(dims.k9 - rho) + dims.k2)


def J_z_delta3456(b, dims, rho, beta1, beta2, delta, whl_cfg):
    if whl_cfg == 3 or whl_cfg == 4:
        a = dims.k7
        sigma = rho + beta1 + delta
    elif whl_cfg == 5 or whl_cfg == 6:
        a = -dims.k7
        sigma = -rho + beta2 + delta
    return -dims.k6 * cos(dims.k9 - b * rho) - a * cos(sigma - delta) + dims.k8 * sin(sigma - delta) - dims.k2


def J_pitch_beta1(b):
    return 0.5 * (b - 1)


def J_pitch_beta2(b):
    return -0.5 * (b + 1)


def delta12(dims, u_dot, alpha_dot, alpha_curr, beta_curr, wq1, wq2, wq3):
    if u_dot.curr.x == 0:
        return [wq1.prev.delta]
    
    r1 = dims.l_b * sin(pi/2 + wq3.curr.delta - alpha_curr) / sin(wq3.curr.delta - wq2.curr.delta)
    r2 = dims.l_b * sin(pi/2 - wq3.curr.delta + alpha_curr) / sin(wq3.curr.delta - wq2.curr.delta)
    d = abs(sqrt(dims.k7**2 + dims.k8**2))
    lambda_angle = atan2(dims.k8, dims.k7)
    r_B = sqrt(r2**2 + d**2 - 2*r2*d*cos(pi/2 + wq3.curr.delta - alpha_curr - lambda_angle))
    v_B = r_B * alpha_dot
    delta = acos(v_B/u_dot.curr.delta * cos(beta_curr - alpha_curr))
    
    return [delta]


def delta3456(dims, u_dot1, u_dot2, alpha_dot, alpha_curr, wq1, wq2):
    # output: [delta1, delta2] which correspond to [delta3, delta5] or [delta4, delta6]
    if u_dot1.curr.x == 0 or u_dot2.curr.x == 0:
        return [wq1.prev.delta, wq2.prev.delta]
        
    a = dims.l_b * alpha_dot / u_dot1.curr.x
    b = u_dot2.curr.x / u_dot1.curr.x
    
    delta1 = alpha_curr + asin((a**2 - b**2) / 2 / a)
    delta2 = alpha_curr + asin((1 + a**2 - b**2) / 2 / a)
    return [delta1, delta2]


def psi_desired(R_d, whl_cfg, dims, rho_prev):
    psi = 0.0
    if (whl_cfg == 1):
        x_c = dims.k4 * cos(radians(rho_prev)) + \
            dims.k5 * sin(radians(rho_prev))
        x_R = dims.k2 - 0.5 * dims.k6 * \
            (sin(dims.k9 - radians(rho_prev) - pi/2.0) +
             sin(dims.k9 - radians(radians(-rho_prev)) - pi/2.0))
        psi = atan((x_c + x_R)/(R_d - dims.k3))
    elif (whl_cfg == 2):
        x_c = dims.k4 * cos(radians(-rho_prev)) + \
            dims.k5 * sin(radians(-rho_prev))
        x_R = dims.k2 - 0.5 * dims.k6 * \
            (sin(dims.k9 - radians(-rho_prev) - pi/2.0) +
             sin(dims.k9 - radians(radians(rho_prev)) - pi/2.0))
        psi = atan((x_c + x_R)/(R_d + dims.k3))
    return psi


def upate_jacobian_12(whl_cfg, dims, q, wq):
    if whl_cfg == 1:
        psi = q.curr.psi1
    elif whl_cfg == 2:
        psi = q.curr.psi2

    b = (-1) ** whl_cfg

    J = np.zeros([6, 4])

    # row 0
    J[0, 0] = -b * dims.k1
    J[0, 1] = J_x_psi(b, dims, q.curr.rho)
    J[0, 2] = J_x_theta(b, dims, q.curr.rho, psi, wq.curr.delta)
    J[0, 3] = J_x_delta12(b, dims, q.curr.rho, psi)

    # row 1
    J[1, 0] = 0
    J[1, 1] = J_y_psi(b, dims, q.curr.rho)
    J[1, 2] = J_y_theta(dims, psi, wq.curr.delta)
    J[1, 3] = J_y_delta(b, dims, q.curr.rho, psi)

    # row 2
    J[2, 0] = b * dims.k2
    J[2, 1] = J_z_psi(dims, q.curr.rho)
    J[2, 2] = J_z_theta(b, dims, q.curr.rho, psi, wq.curr.delta)
    J[2, 3] = J_z_delta12(b, dims, q.curr.rho, psi)

    # row 3
    J[3, 0] = 0
    J[3, 1] = J_yaw_psi(b, q.curr.rho)
    J[3, 2] = 0
    J[3, 3] = J_yaw_delta(q.curr.rho, psi)

    # row 4
    J[4, 0] = b
    J[4, 1] = 0
    J[4, 2] = 0
    J[4, 3] = J_pitch_delta(psi)

    # row 5
    J[5, 0] = 0
    J[5, 1] = J_roll_psi(q.curr.rho)
    J[5, 2] = 0
    J[5, 3] = J_roll_delta(b, q.curr.rho, psi)

    return J


def upate_jacobian_3456(whl_cfg, dims, q, wq):
    if whl_cfg == 3 or whl_cfg == 4:
        a = dims.k7
        sigma = q.curr.rho + q.curr.beta1 + wq.curr.delta
    elif whl_cfg == 5 or whl_cfg == 6:
        a = -dims.k7
        sigma = -q.curr.rho + q.curr.beta2 + wq.curr.delta

    b = (-1) ** whl_cfg

    J = np.zeros([6, 5])

    # row 0
    J[0, 0] = -b * dims.k1
    J[0, 1] = J_x_beta1(b, dims, q.curr.rho, wq.curr.delta)
    J[0, 2] = J_x_beta2(b, dims, q.curr.rho, wq.curr.delta)
    J[0, 3] = dims.k10 * cos(sigma)
    J[0, 4] = J_x_delta3456(b, dims, q.curr.rho,
                            q.curr.beta1, q.curr.beta2, wq.curr.delta, whl_cfg)

    # row 1
    J[1, 0] = 0
    J[1, 1] = 0
    J[1, 2] = 0
    J[1, 3] = 0
    J[1, 4] = 0

    # row 2
    J[2, 0] = b * dims.k2
    J[2, 1] = J_z_beta1(b, dims, q.curr.rho)
    J[2, 2] = J_z_beta2(b, dims, q.curr.rho)
    J[2, 3] = -dims.k10 * sin(sigma)
    J[2, 4] = J_z_delta3456(b, dims, q.curr.rho,
                            q.curr.beta1, q.curr.beta2, wq.curr.delta, whl_cfg)

    # row 3
    J[3, 0] = 0
    J[3, 1] = 0
    J[3, 2] = 0
    J[3, 3] = 0
    J[3, 4] = 0

    # row 4
    J[4, 0] = b
    J[4, 1] = J_pitch_beta1(b)
    J[4, 2] = J_pitch_beta2(b)
    J[4, 3] = 0
    J[4, 4] = -1

    # row 5
    J[5, 0] = 0
    J[5, 1] = 0
    J[5, 2] = 0
    J[5, 3] = 0
    J[5, 4] = 0

    return J


def wheel12(V_d, w_d, whl_cfg, dims, q, q_dot, wq_dot, J):
    b = (-1) ** whl_cfg
    theta_dot_d = (V_d + b * dims.k1 * q_dot.prev.rho_dot -
                   J[0, 1] * q_dot.prev.psi_dot - J[0, 3] * wq_dot.prev.delta_dot) / J[0, 2]

    if w_d == 0.0 or V_d == 0.0:
        return [theta_dot_d, 0.0]

    # Radius of rotation (desired)
    R_d = V_d/w_d
    psi_d = psi_desired(R_d, whl_cfg, dims, q.prev.rho)

    return [theta_dot_d, psi_d]     # pass to control module


def wheel3456(V_d, whl_cfg, dims, q, q_dot, wq, wq_dot, J):
    if whl_cfg == 3 or whl_cfg == 4:
        sigma = q.curr.rho + q.curr.beta1 + wq.curr.delta
    elif whl_cfg == 5 or whl_cfg == 6:
        sigma = -q.curr.rho + q.curr.beta2 + wq.curr.delta

    b = (-1) ** whl_cfg

    theta_dot = (V_d + b * dims.k1 * q_dot.prev.rho_dot -
                 J[0, 1] * q_dot.prev.beta1_dot -
                 J[0, 2] * q_dot.prev.beta2_dot -
                 J[0, 4] * wq_dot.prev.delta_dot) / (dims.k10 * cos(sigma))

    return [theta_dot]
