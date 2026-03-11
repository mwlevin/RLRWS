import numpy as np
import os
import sys
import optparse

import math
import matplotlib.pyplot as plt

from ILC_constants import Veh_Parameter
from ILC_constants import IDM_Param
from derivation_fast import Derivation_class
# from SUMO_section import SUMO_TRACI_CLASS
import time

# to be organzied with the function arguments
# 1. set of constants (free flow speed, delta)
# 2. set of variable inputs : (warning_msg, acceleration, speed, spacing, traffic light status tls)
# 3. set of parameters inputs to be calibrated: (a, d , c, T)


# IDM variables definitions
it = IDM_Param().it   # number of iterations 
iteration = IDM_Param().iteration
con_range = Veh_Parameter().cv_range
dxx = IDM_Param().dxx
# mu = IDM_Param().mu
buffer = IDM_Param().buffer
high_buffer = IDM_Param().high_buffer
lb = IDM_Param().lb  # a lower bound for a, b, T, s0 values
lb_speed = IDM_Param().lb_speed # lower band for v0
max_a = IDM_Param().ub_ac
min_a = IDM_Param().lb_ac
min_T = IDM_Param().lb_time
max_T = IDM_Param().ub_time
min_d = IDM_Param().lb_d
max_d = IDM_Param().ub_d
min_c = IDM_Param().lb_c
max_c = IDM_Param().ub_c 
 
small_num_war = IDM_Param().small_num_warning
num_init = IDM_Param().num_init  # number of initial points
# initial values for parametrs + lb and ub for each
# s0 = np.zeros((num_init, it))
# s0[: , 0] = IDM_Param().s0

# a = np.zeros((num_init, it))
# a[: , 0] = IDM_Param().a
# # print(a[1][0])

# b = np.zeros((num_init, it))
# b[: , 0] =  IDM_Param().b

# T = np.zeros((num_init, it))
# T[: , 0] = IDM_Param().T

# intitial values for parameters
a_val = IDM_Param().a[0]
d_val = IDM_Param().d[0]
c_val = IDM_Param().c[0]    
T_val = IDM_Param().T[0]

v0 = IDM_Param().v0[0]

delta = IDM_Param().delta
epsilon = IDM_Param().epsilon
small_num = IDM_Param().small_num



# function: find derivative for a sum (all simulation steps) in J
df_class = Derivation_class()
large_number = df_class.large_num

# input is list of dynamics (spd, spacing , etc.)
def dGradient_da(v0, delta, warning, acc_ego , spd_ego, spacing_ego, tls, a_fix , d_fix, c_fix, T_fix):
    dJ = 0
    for i in range(len(spacing_ego)):
        dJ += df_class.a_derivative(v0, delta, warning[i], acc_ego[i] , spd_ego[i], spacing_ego[i], tls[i], a_fix , d_fix, c_fix, T_fix)

    dJ = dJ/ len(spacing_ego)
    return dJ

def dGradient_dd(v0, delta, warning, acc_ego , spd_ego, spacing_ego, tls, a_fix , d_fix, c_fix, T_fix):
    dJ = 0
    for i in range(len(spacing_ego)):
        dJ += df_class.d_derivative(v0, delta, warning[i], acc_ego[i] , spd_ego[i], spacing_ego[i], tls[i], a_fix , d_fix, c_fix, T_fix)

    dJ = dJ/ len(spacing_ego)
    return dJ

def dGradient_dc(v0, delta, warning, acc_ego , spd_ego, spacing_ego, tls, a_fix , d_fix, c_fix, T_fix):
    dJ = 0
    for i in range(len(spacing_ego)):
        dJ += df_class.c_derivative(v0, delta, warning[i], acc_ego[i] , spd_ego[i], spacing_ego[i], tls[i], a_fix , d_fix, c_fix, T_fix)

    dJ = dJ/ len(spacing_ego)
    return dJ

def dGradient_dT(v0, delta, warning, acc_ego , spd_ego, spacing_ego, tls, a_fix , d_fix, c_fix, T_fix):
    dJ = 0
    for i in range(len(spacing_ego)):
        dJ += df_class.T_derivative(v0, delta, warning[i], acc_ego[i] , spd_ego[i], spacing_ego[i], tls[i], a_fix , d_fix, c_fix, T_fix)

    dJ = dJ/ len(spacing_ego)
    return dJ

print("derivative functions are defined")



# find the value of J, [warning, acc_ego, spacing_ego, speed_ego, tls] are lists of variables recorded from data
def J(v0, delta, warning, acc_ego , spd_ego, spacing_ego, tls, a_fix , d_fix, c_fix, T_fix):
    J = 0
    # print("man", a, b)
    for i in range(len(spacing_ego)):
        # function from eq 4
        J = J + ( acc_ego[i] - v_dot_value(v0, delta, warning[i] , acc_ego[i], spd_ego[i], spacing_ego[i], tls[i], a_fix, d_fix, c_fix, T_fix) )**2 

    
    J = J/ len(spacing_ego)
    return J


# find the estimated acceleration, modified IDM formula
def v_dot_value(v0, delta, warning, acc_ego , spd_ego, spacing_ego, tls, a_fix , d_fix, c_fix, T_fix):
    warning = max(small_num_war, warning)
    v_dot = a_fix * w_value(v0, delta, spd_ego, spacing_ego, tls, T_fix) - d_fix * warning ** c_fix
    # print( "v dot is ", v_dot, " a_fixed " , a_fix,  " speed is ", spd_ego, " spacing is ", spacing_ego, " tls is ", tls, " w value is ", w_value(v0, delta, spd_ego, spacing_ego, tls, T_fix), " warning is ", warning, " c fix is ", c_fix, " d fix is ", d_fix)
    # print("math gives " , a_fix * w_value(v0, delta, spd_ego, spacing_ego, tls, T_fix) , " minus ", d_fix * warning ** c_fix)
    # print("v dot is ", v_dot, " d_fix is ", d_fix, " w val " , w_value(v0, delta, spd_ego, spacing_ego, tls, T_fix) - d_fix * warning ** c_fix, " warning is ", warning, " c_fix is ", c_fix)
    return v_dot
    

# find the value of w (inside IDM paranthesis), used for finding v_dot
def w_value(v0, delta, spd, spacing, tls, T):
    if tls == 1: # traffic light to be green, spacing is easier
        w = w = 1 - (spd/v0)**delta - ( (0 ) / (spacing) )**2
    
    else: # red light
        if spacing > con_range: # far from intersection, spacing has no effect
            w = w = 1 - (spd/v0)**delta - ( (0 ) / (spacing) )**2
        
        else:
            w = 1 - (spd/v0)**delta - ( (spacing + spd * T) / (spacing) )**2
    # print("w is ", w)
    return w

# print("ideal value is", J(2.5, 3.2, 3, 1, 35, delta, spacing_ego, speed_ego, relspd_ego, acc_ego ))
# print("max values ", max_a, max_v, min_a, min_s)
last_step = 0



J_prev = 10e11


# main calibration function
def mainIDM(v0, delta, warning, acc_ego, spd_ego, spacing_ego, tls):

    step = 0
    params = IDM_Param()

    a_val = params.a[0]
    d_val = params.d[0]
    c_val = params.c[0]
    T_val = params.T[0]
    v0    = params.v0[0]

    # Bounds
    min_a, max_a = params.lb_ac,   params.ub_ac    # [0.5, 4.5]
    min_d, max_d = params.lb_d,    params.ub_d     # [-10, 10]
    min_c, max_c = params.lb_c,    params.ub_c     # [-5,  5]
    min_T, max_T = params.lb_time, params.ub_time  # [0.5, 6]

    def clamp(val, lo, hi):
        return max(lo, min(hi, val))

    # Armijo condition constant (sufficient decrease factor)
    armijo_c = 1e-4
    
    J_prev = float('inf')
    J_actual = J(v0, delta, warning, acc_ego, spd_ego, spacing_ego, tls,
                 a_val, d_val, c_val, T_val)
    print(f"Initial J: {J_actual:.6f}  a: {a_val}  d: {d_val}  c: {c_val}  T: {T_val}")

    while step < it:

        if abs(J_actual - J_prev) < epsilon:
            print(f"Converged at step {step}, J={J_actual:.6f}")
            break

        # --- Compute raw gradients ---
        G_a = dGradient_da(v0, delta, warning, acc_ego, spd_ego, spacing_ego, tls, a_val, d_val, c_val, T_val)
        G_d = dGradient_dd(v0, delta, warning, acc_ego, spd_ego, spacing_ego, tls, a_val, d_val, c_val, T_val)
        G_c = dGradient_dc(v0, delta, warning, acc_ego, spd_ego, spacing_ego, tls, a_val, d_val, c_val, T_val)
        G_T = dGradient_dT(v0, delta, warning, acc_ego, spd_ego, spacing_ego, tls, a_val, d_val, c_val, T_val)

        # --- Normalize gradient to unit vector (prevents exploding steps) ---
        grad_norm = (G_a**2 + G_d**2 + G_c**2 + G_T**2) ** 0.5
        if grad_norm < 1e-10:
            print(f"Gradient vanished at step {step}. Stopping.")
            break
        G_a_n = G_a / grad_norm
        G_d_n = G_d / grad_norm
        G_c_n = G_c / grad_norm
        G_T_n = G_T / grad_norm

        print(f"\n  Gradients (raw):  G_a={G_a:.4f}  G_d={G_d:.4f}  G_c={G_c:.4f}  G_T={G_T:.4f}")
        print(f"  Gradients (norm): G_a={G_a_n:.4f}  G_d={G_d_n:.4f}  G_c={G_c_n:.4f}  G_T={G_T_n:.4f}")

        # --- Armijo backtracking line search ---
        # Start mu at a reasonable scale (range of each parameter)
        mu = 1.0
        max_iters_ls = 60
        ls_iter = 0
        found = False

        while ls_iter < max_iters_ls:
            a_new = clamp(a_val - mu * G_a_n, min_a, max_a)
            d_new = clamp(d_val - mu * G_d_n, min_d, max_d)
            c_new = clamp(c_val - mu * G_c_n, min_c, max_c)
            T_new = clamp(T_val - mu * G_T_n, min_T, max_T)

            J_hat = J(v0, delta, warning, acc_ego, spd_ego, spacing_ego, tls,
                      a_new, d_new, c_new, T_new)

            # Armijo sufficient decrease: J_hat <= J_actual - c * mu * ||grad||
            # Since grad is normalized, ||grad_n|| = 1, so: J_hat <= J_actual - armijo_c * mu
            if J_hat <= J_actual - armijo_c * mu:
                found = True
                break

            mu /= 2
            ls_iter += 1

        print(f"  Line search: {ls_iter} iters, mu={mu:.2e}, found={found}")

        if found:
            J_prev   = J_actual
            a_val, d_val, c_val, T_val = a_new, d_new, c_new, T_new
            J_actual = J_hat
            step    += 1
            print(f" step: {step}  J: {J_actual:.6f}  a: {a_val:.4f}  d: {d_val:.4f}  c: {c_val:.4f}  T: {T_val:.4f}  mu: {mu:.2e}")
        else:
            print(f"Line search failed at step {step}, J={J_actual:.6f}. At local minimum or flat region.")
            break

    return a_val, d_val, c_val, T_val, J_actual



# given a calibrated IDM model and warning, find the presented warning value to the driver, values are scalar (not lists)
def rev_warning_calc(v0, delta, warning, acc_ego, spd_ego, spacing_ego, tls, a_cal , d_cal, c_cal, T_cal):
    mpc_acc = -1/20 * warning
    u = (a_cal * w_value(v0, delta, spd_ego, spacing_ego, tls, T_cal) / d_cal) ** (1/c_cal) - (mpc_acc / d_cal) ** (1/c_cal)
    return u


# given a trained IDM model, find the anticipated acceleration for a given state (warning, acc, spd, sapcing, tls)
def pred_acc(v0, delta, warning, acc_ego, spd_ego, spacing_ego, tls, a_cal , d_cal, c_cal, T_cal):
    max_a = -1 * IDM_Param().ub_ac
    acc_pred = v_dot_value(v0, delta, warning, acc_ego , spd_ego, spacing_ego, tls, a_cal , d_cal, c_cal, T_cal)

    return max(acc_pred, max_a)






# offline testing against vehicle data
import pandas as pd
import re

df = pd.read_csv('NS4218.csv')

pos_ego           = df['pos_ego'].tolist()
spd_ego           = df['spd_ego'].tolist()
warning_0         = df['warning_0'].tolist()

def extract_first_tl(tl_str):
    numbers = re.findall(r'[\d.]+', str(tl_str))
    return float(numbers[0]) if numbers else None

# 4. acc_ego list (acceleration = delta_spd / delta_t, timestep = 1s)
acc_ego = [0.0]  # first step assumed 0

for i in range(1, len(spd_ego)):
    acc = (spd_ego[i] - spd_ego[i-1]) / 1.0  # timestep = 1s
    acc_ego.append(acc)

# 5. traffic light status list (1 for green, 0 for red)
predicted_tl_first = df['predicted_tl_state'].apply(extract_first_tl).tolist()

print("data length is ", len(pos_ego), len(spd_ego), len(warning_0), len(predicted_tl_first))
# print(" spacing is " , pos_ego)
# print( " speed is ", spd_ego)

# test IDM
a, d, c, T, J_final = mainIDM(v0, delta, warning_0 , acc_ego, spd_ego, pos_ego, predicted_tl_first)


print("final values are ", a, d, c, T, J_final)



# now check it against the second data file
df2 = pd.read_csv('NS4218_2_clean.csv')
pos_ego           = df2['pos_ego'].tolist()
spd_ego           = df2['spd_ego'].tolist()
warning_0         = df2['warning_0'].tolist()

def extract_first_tl(tl_str):
    numbers = re.findall(r'[\d.]+', str(tl_str))
    return float(numbers[0]) if numbers else None

# 4. acc_ego list (acceleration = delta_spd / delta_t, timestep = 1s)
acc_ego = [0.0]  # first step assumed 0

for i in range(1, len(spd_ego)):
    acc = (spd_ego[i] - spd_ego[i-1]) / 1.0  # timestep = 1s
    acc_ego.append(acc)
    
# 5. traffic light status list (1 for green, 0 for red)
predicted_tl_first = df['predicted_tl_state'].apply(extract_first_tl).tolist()
    
# check acc values
acc_pred_list = []
for i in range(len(acc_ego)):
    acc_pred = pred_acc(v0, delta, warning_0[i], acc_ego[i], spd_ego[i], pos_ego[i], predicted_tl_first[i], a, d, c, T)
    acc_pred_list.append(acc_pred)
    print("predicted acc is ", acc_pred, " actual acc is ", acc_ego[i])
    
    
# plt.figure(figsize=(12, 5))
# plt.plot(acc_ego,       label='Actual acc',    color='steelblue')
# plt.plot(acc_pred_list, label='Predicted acc', color='tomato', linestyle='--')
# plt.xlabel('Index')
# plt.ylabel('Acceleration (m/s²)')
# plt.title('Actual vs Predicted Acceleration')
# plt.legend()
# plt.grid(True)
# plt.tight_layout()
# plt.show()