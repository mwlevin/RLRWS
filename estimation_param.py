import numpy as np
from constants import *

class UKFParam:
    def __init__(self, num_cell, dt=0.2, dx=20, num_param=3, dt_simulation = 0.2):
        self.num_cell = num_cell
        self.dt = dt
        self.dx = dx
        self.num_param = num_param
        # free flow speed
        v0 = 28.78 
        # maximum density
        rhomax = 0.16 
        # initial density
        rho0=0.02
        self.v0 = v0
        self.rho0 = rho0
        self.dt_simulation = dt_simulation
        self.list_param = np.zeros((num_param, 1))
        self.list_param_lb = np.zeros((num_param, 1))
        self.list_param_ub = np.zeros((num_param, 1))
        self.list_param_lb_tf = np.zeros((num_param, 1))
        self.list_param_ub_tf = np.zeros((num_param, 1))
        self.list_param_k = np.zeros((num_param, 1))
        self.list_param_b = np.zeros((num_param, 1))
        self.list_param_std = np.zeros((num_param, 1))
        self.rho_max = rhomax
        # parameter for PW Model
        self.list_param[PW_ID["c"], 0] = self.c_0 = 10
        self.list_param[PW_ID["c0"], 0] = self.c0_0 = 11
        self.list_param[PW_ID["tau"], 0] = self.tau_0 = 6
        # scaling factor
        self.k_rho = 1/0.01
        self.b_rho = -0.1
        
        self.k_v = 1/1.5
        self.b_v = -5

        self.list_param_k[PW_ID["c"], 0] = self.k_c = 1/1
        self.list_param_b[PW_ID["c"], 0] = self.b_c = -5

        self.list_param_k[PW_ID["c0"], 0] = self.k_c0 = 1/1
        self.list_param_b[PW_ID["c0"], 0] = self.b_c0 = -5

        self.list_param_k[PW_ID["tau"], 0] = self.k_tau = 1/1
        self.list_param_b[PW_ID["tau"], 0] = self.b_tau = -5

        # lower and upper bound for parameter
        self.rho_lb = 0.001 
        self.rho_ub = rhomax

        self.v_lb = 0.0
        self.v_ub = 30

        self.list_param_lb[PW_ID["c"], 0] = self.c_lb = 7
        self.list_param_ub[PW_ID["c"], 0] = self.c_ub = 13

        self.list_param_lb[PW_ID["c0"], 0] = self.c0_lb = 3
        self.list_param_ub[PW_ID["c0"], 0] = self.c0_ub = 10

        self.list_param_lb[PW_ID["tau"], 0] = self.tau_lb = 5 
        self.list_param_ub[PW_ID["tau"], 0] = self.tau_ub = 12

        # lower and upper bound for parameter
        self.rho_lb_tf = (self.rho_lb-self.b_rho)*self.k_rho
        self.rho_ub_tf = (self.rho_ub - self.b_rho)*self.k_rho

        self.v_lb_tf = (self.v_lb - self.b_v)*self.k_v
        self.v_ub_tf = (self.v_ub - self.b_v)*self.k_v

        self.list_param_lb_tf[PW_ID["c"], 0] = self.c_lb_tf = (
            self.c_lb - self.b_c)*self.k_c
        self.list_param_ub_tf[PW_ID["c"], 0] = self.c_ub_tf = (
            self.c_ub - self.b_c)*self.k_c

        self.list_param_lb_tf[PW_ID["c0"], 0] = self.c0_lb_tf = (
            self.c0_lb - self.b_c0)*self.k_c0
        self.list_param_ub_tf[PW_ID["c0"], 0] = self.c0_ub_tf = (
            self.c0_ub - self.b_c0)*self.k_c0

        self.list_param_lb_tf[PW_ID["tau"], 0] = self.tau_lb_tf = (
            self.tau_lb - self.b_tau)*self.k_tau
        self.list_param_ub_tf[PW_ID["tau"], 0] = self.tau_ub_tf = (
            self.tau_ub - self.b_tau)*self.k_tau
        # standard deviation
        # state
        self.std_rho = 0.002*self.k_rho 
        self.std_v = 0.001*self.k_v 
        # parameter
        self.list_param_std[PW_ID["c"], 0] = self.std_c = 0.01*self.k_c
        self.list_param_std[PW_ID["c0"], 0] = self.std_c0 = 0.005*self.k_c0
        self.list_param_std[PW_ID["tau"], 0] = self.std_tau = 0.005*self.k_tau
        # measurement
        self.std_measure = 0.005*self.k_v
