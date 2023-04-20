#!/usr/bin/python3

from rover_defs import *


def update_jacobians(dims, q, wq):
    ## wheels 1,2 ##
    J_1 = np.zeros([6, 4])
    J_2 = np.zeros([6, 4])

    J_1 = upate_jacobian_12(1, dims, q, wq)
    J_2 = upate_jacobian_12(2, dims, q, wq)

    ## wheels 3, 4, 5, 6 ##
    J_3 = np.zeros([6, 5])
    J_4 = np.zeros([6, 5])
    J_5 = np.zeros([6, 5])
    J_6 = np.zeros([6, 5])

    J_3 = upate_jacobian_3456(3, dims, q, wq)
    J_4 = upate_jacobian_3456(4, dims, q, wq)
    J_5 = upate_jacobian_3456(5, dims, q, wq)
    J_6 = upate_jacobian_3456(6, dims, q, wq)

    return [J_1, J_2, J_3, J_4, J_5, J_6]


def update_setpoints(V_d, w_d, dims, q, q_dot, wq, wq_dot, J):
    whl1_setpoints = wheel12(V_d, w_d, 1, dims, q, q_dot,
                             wq_dot, J[0])    # [theta_dot_d, psi_d]
    whl2_setpoints = wheel12(V_d, w_d, 2, dims, q, q_dot,
                             wq_dot, J[1])    # [theta_dot_d, psi_d]
    whl3_setpoints = wheel3456(
        V_d, 3, dims, q, q_dot, wq, wq_dot, J[2])   # [theta_dot_d]
    whl4_setpoints = wheel3456(
        V_d, 4, dims, q, q_dot, wq, wq_dot, J[3])   # [theta_dot_d]
    whl5_setpoints = wheel3456(
        V_d, 5, dims, q, q_dot, wq, wq_dot, J[4])   # [theta_dot_d]
    whl6_setpoints = wheel3456(
        V_d, 6, dims, q, q_dot, wq, wq_dot, J[5])   # [theta_dot_d]

    return [whl1_setpoints, whl2_setpoints, whl3_setpoints, whl4_setpoints, whl5_setpoints, whl6_setpoints]


def update_ground_contact_angles(dims, u_dot1, u_dot2, u_dot3, u_dot4, u_dot5, u_dot6, alpha_dot, alpha_curr, beta_curr, wq1, wq3, wq5):
    delta1 = delta12(dims, u_dot1, alpha_dot, alpha_curr,
                     beta_curr, wq1, wq3, wq5)
    delta2 = delta12(dims, u_dot2, alpha_dot, alpha_curr,
                     beta_curr, wq2, wq4, wq6)
    delta35 = delta3456(dims, u_dot3, u_dot5, alpha_curr, wq3, wq5)
    delta46 = delta3456(dims, u_dot4, u_dot6, alpha_curr, wq4, wq6)
    return [delta1[0], delta2[0], delta35[0], delta35[1], delta46[0], delta46[1]]


def update(V_d, w_d, dims, q, q_dot, wq, wq_dot, t, dt):
    # Read sensed angles (rho, beta1, beta2, alpha_dot, gamma1_dot, gamma2_dot)
    # Hardware ->

    # Calculating jacobians for all wheels
    J = update_jacobians(dims, q, wq)

    # Calculate reference points for wheels -> theta_dot_d (for all wheels), psi_d (for wheel 1,2)
    # Calculate unsensed angles (psi1, psi2, delta_i) -> this step is implicitly done inside the desired values calculation
    update_ground_contact_angles(dims, u_dot1, u_dot2, u_dot3, u_dot4, u_dot5, u_dot6, alpha_dot, alpha_curr, beta_curr, wq1, wq3, wq5)
    whl_refs = update_setpoints(V_d, w_d, dims, q, q_dot, wq, wq_dot, J)


V_d = 1.0  # m/s, for the whole vehicle
w_d = 1.0  # rad/s, for the whole vehicle

# Geometric parameters:
dims = dotdict()
# vertical offset between rover reference R to differential D
dims.k1 = 0.105    # [m]
# forward offset between R and D
dims.k2 = 0.12075  # [m]
# horizontal offset between D and wheels / l1
dims.k3 = 0.20     # [m]
# distance from D to steering axis of front wheels / l6
dims.k4 = 0.288    # [m]
# height of D from wheel axles / l7
dims.k5 = 0.125    # [m]
# length of link from rocker joint to bogie joint / l3
dims.k6 = 0.16     # [m]
# length from bogie joint to forward/rear bogie / l4 / l8
dims.k7 = 0.069    # [m]
# height of bogie joint from wheel axles / l5
dims.k8 = 0.02     # [m]
# angle of link between rocker and bogie joints / l2
dims.k9 = 139.0*pi/180.0  # [rad]
dims.l2 = 0.1208   # [m]
# wheel radius
dims.k10 = 0.065    # [m]
# distance between rocker wheel and rocker joint
dims.l_r = 0.313957
# distance between bogie wheels' centers
dims.l_b = 0.138

# fl = 1    # front left
# fr = 2    # front right
# ml = 3    # middle left
# mr = 4    # middle right
# rl = 5    # rear left
# rr = 6    # rear right

wq1 = dotdict()
wq2 = dotdict()
wq3 = dotdict()
wq4 = dotdict()
wq5 = dotdict()
wq6 = dotdict()

wq1.curr = {'theta': 0.0,
            'delta': 0.0}
wq1.prev = {'theta': 0.0,
            'delta': 0.0}
wq1.curr = dotdict(wq1.curr)
wq1.prev = dotdict(wq1.prev)

wq2.curr = {'theta': 0.0,
            'delta': 0.0}
wq2.prev = {'theta': 0.0,
            'delta': 0.0}
wq2.curr = dotdict(wq2.curr)
wq2.prev = dotdict(wq2.prev)

wq3.curr = {'theta': 0.0,
            'delta': 0.0}
wq3.prev = {'theta': 0.0,
            'delta': 0.0}
wq3.curr = dotdict(wq3.curr)
wq3.prev = dotdict(wq3.prev)

wq4.curr = {'theta': 0.0,
            'delta': 0.0}
wq4.prev = {'theta': 0.0,
            'delta': 0.0}
wq4.curr = dotdict(wq4.curr)
wq4.prev = dotdict(wq4.prev)

wq5.curr = {'theta': 0.0,
            'delta': 0.0}
wq5.prev = {'theta': 0.0,
            'delta': 0.0}
wq5.curr = dotdict(wq5.curr)
wq5.prev = dotdict(wq5.prev)

wq6.curr = {'theta': 0.0,
            'delta': 0.0}
wq6.prev = {'theta': 0.0,
            'delta': 0.0}
wq6.curr = dotdict(wq6.curr)
wq6.prev = dotdict(wq6.prev)

wq1_dot = dotdict()
wq2_dot = dotdict()
wq3_dot = dotdict()
wq4_dot = dotdict()
wq5_dot = dotdict()
wq6_dot = dotdict()

wq1_dot.curr = {'theta_dot': 0.0,
                'delta_dot': 0.0}
wq1_dot.prev = {'theta_dot': 0.0,
                'delta_dot': 0.0}
wq1_dot.curr = dotdict(wq1_dot.curr)
wq1_dot.prev = dotdict(wq1_dot.prev)

wq2_dot.curr = {'theta_dot': 0.0,
                'delta_dot': 0.0}
wq2_dot.prev = {'theta_dot': 0.0,
                'delta_dot': 0.0}
wq2_dot.curr = dotdict(wq2_dot.curr)
wq2_dot.prev = dotdict(wq2_dot.prev)

wq3_dot.curr = {'theta_dot': 0.0,
                'delta_dot': 0.0}
wq3_dot.prev = {'theta_dot': 0.0,
                'delta_dot': 0.0}
wq3_dot.curr = dotdict(wq3_dot.curr)
wq3_dot.prev = dotdict(wq3_dot.prev)

wq4_dot.curr = {'theta_dot': 0.0,
                'delta_dot': 0.0}
wq4_dot.prev = {'theta_dot': 0.0,
                'delta_dot': 0.0}
wq4_dot.curr = dotdict(wq4_dot.curr)
wq4_dot.prev = dotdict(wq4_dot.prev)

wq5_dot.curr = {'theta_dot': 0.0,
                'delta_dot': 0.0}
wq5_dot.prev = {'theta_dot': 0.0,
                'delta_dot': 0.0}
wq5_dot.curr = dotdict(wq5_dot.curr)
wq5_dot.prev = dotdict(wq5_dot.prev)

wq6_dot.curr = {'theta_dot': 0.0,
                'delta_dot': 0.0}
wq6_dot.prev = {'theta_dot': 0.0,
                'delta_dot': 0.0}
wq6_dot.curr = dotdict(wq6_dot.curr)
wq6_dot.prev = dotdict(wq6_dot.prev)

q = dotdict()
q_dot = dotdict()

q.curr = {'rho': 0.0,
          'beta1': 0.0,
          'beta2': 0.0,
          'psi1': 0.0,
          'psi2': 0.0}
q.prev = {'rho': 0.0,
          'beta1': 0.0,
          'beta2': 0.0,
          'psi1': 0.0,
          'psi2': 0.0}

q.curr = dotdict(q.curr)
q.prev = dotdict(q.prev)

q_dot.curr = {'rho_dot': 0.0,
              'beta1_dot': 0.0,
              'beta2_dot': 0.0,
              'psi1_dot': 0.0,
              'psi2_dot': 0.0}
q_dot.prev = {'rho_dot': 0.0,
              'beta1_dot': 0.0,
              'beta2_dot': 0.0,
              'psi1_dot': 0.0,
              'psi2_dot': 0.0}

q_dot.curr = dotdict(q_dot.curr)
q_dot.prev = dotdict(q_dot.prev)

u1 = dotdict()
u1_dot = dotdict()

u1.curr = {'x': 0.0,
           'y': 0.0,
           'z': 0.0,
           'yaw': 0.0,
           'pitch': 0.0,
           'roll': 0.0}
u1.prev = {'x': 0.0,
           'y': 0.0,
           'z': 0.0,
           'yaw': 0.0,
           'pitch': 0.0,
           'roll': 0.0}
u1.curr = dotdict(u1.curr)
u1.prev = dotdict(u1.prev)

u1_dot.curr = {'x_dot': 0.0,
               'y_dot': 0.0,
               'z_dot': 0.0,
               'yaw_dot': 0.0,
               'pitch_dot': 0.0,
               'roll_dot': 0.0}
u1_dot.prev = {'x_dot': 0.0,
               'y_dot': 0.0,
               'z_dot': 0.0,
               'yaw_dot': 0.0,
               'pitch_dot': 0.0,
               'roll_dot': 0.0}
u1_dot.curr = dotdict(u1_dot.curr)
u1_dot.prev = dotdict(u1_dot.prev)


# print(upate_jacobian_12(1, dims, q, wq1))

t = np.linspace(0, 100, 1001)
dt = 0.1

x1 = np.zeros_like(t)
y1 = np.zeros_like(t)
z1 = np.zeros_like(t)
yaw1 = np.zeros_like(t)
pitch1 = np.zeros_like(t)
roll1 = np.zeros_like(t)

delta = np.zeros_like(t)

for i in range(t.shape[0]):
    delta[i] = degrees(0.1 * (cos(0.15*t[i]) + sin(0.15*t[i])))

pyplot.plot(t, delta)
pyplot.show()
