import numpy as np
import math


# target_vehicle = "ego.0"

# class Target_Vehicle:
#     def __init__(self):
#         self.ego_id = target_vehicle

# def platoon_information(self):
#     # CV id in the simulation, target vehicle is not included
#     CV_list = ["car.2", "car.1"]
#     # target vehicle name (speed that to be predicted)
#     target_veh = target_vehicle
#     return CV_list, target_veh

target_vehicle = "CV.0"

class Target_Vehicle:
    def __init__(self):
        self.ego_id = target_vehicle

def platoon_information(self):
    # CV id in the simulation, target vehicle is not included
    CV_list = ["car.2", "car.1"]
    # target vehicle name (speed that to be predicted)
    target_veh = target_vehicle
    return CV_list, target_veh

# # parameters for IDM model
# class IDM_Param():
#     def __init__(self):
#         self.num_init = 5     # number of intital points

#         # initial values for parametrs + lower and upper bound for them

#         self.s0 = [8, 5, 2.5, 3.5, 0.5]     # min gap


#         self.a = [5, 3, 2.5, 1.5, 3.1]         # acceleration


#         self.b = [5, 3, 4.5, 1.5, 2.2]   # deceleration


#         self.T = [2, 1.5, 0.9, 1.1, 0.3]          # time headway


#         self.v0 = [Veh_Parameter().max_spd+10, 12, 30, 35, 20 ]  # desired speed

#         self.delta = 4
#         self.epsilon = 0.0001
#         self.small_num = 1

        
#         self.it = 1001   # number of iterations 
#         self.iteration = np.arange(0, self.it, 1)
#         self.dxx = 0.02
#         self.mu = 0.0005   # step size
#         self.buffer = 1.5  # a confidence buffer greater than 1
#         self.high_buffer = 1.3
#         self.lb = 1  # a lower bound for a, b, T, s0 values
#         self.lb_speed = 5 # lower band for v0


# parameters for IDM model
class IDM_Param():
    def __init__(self):
        self.num_init = 1     # number of intital points

        # initial values for parametrs + lower and upper bound for them
# 24.016563642758417 1.6084034166031316 0.9449282323439725 2.5 16.016849585586613
        self.s0 = [5]     # min gap


        self.a = [1.2]         # acceleration


        self.b = [0.94]   # deceleration


        self.T = [2.5]          # time headway


        self.v0 = [24.5]  # desired speed (free flow speed)
        
        self.d = [0.1]
        
        self.c = [1]

        self.delta = 2  # changed to test
        self.epsilon = 0.0000001
        self.small_num = 1
        
        self.small_num_warning = 0.3

        
        self.it = 101   # number of iterations 
        self.iteration = np.arange(0, self.it, 1)
        self.dxx = 0.02
        self.mu = 0.005   # step size
        self.buffer = 1.3  # a confidence buffer greater than 1
        self.high_buffer = 1.2
        self.lb = 1  # a lower bound for a, b, T, s0 values
        self.lb_speed = 5 # lower band for v0
        self.lb_ac = 0.2      # 0.4 test
        self.ub_ac = 4.5
        self.lb_dec = 0.5
        self.ub_dec = 4.5
        self.lb_time = 0.5
        self.ub_time = 2
        
        self.lb_d =  0.06   #  0.08  test
    
        self.ub_d = 2
        
        self.lb_c =  0.5         # 0.8 test    
        self.ub_c = 2
        
    def set_a (self, a_cal):
        self.a = [a_cal]
    def set_d (self, d_cal):
        self.d = [d_cal]
    def set_c (self, c_cal):
        self.c = [c_cal]
    def set_T (self, T_cal):
        self.T = [T_cal]

        
        
        







class Veh_Parameter:
    def __init__(self):
        # timestep in SUMO (sec)
        self.dt_sumo = 0.5
        # tiemstep for traffic prediction (sec)
        self.dt_model = 1
        # tiemstep for mpc optimization (sec)
        self.dt_mpc = 0.5
        # length of cell (m)
        self.dx = 15
        # total simulation time
        self.totaltime = int(180/self.dt_sumo)
        # id of the last CV, to be changed
        self.lastCV = 0
        # prediction horizon in time (sec)
        self.horizon_time = 10
        # prediction horizon
        self.horizon_step = int(self.horizon_time / self.dt_sumo)
        # communication range (m)
        self.range_V2V = 500
        self.range_V2I = 500
        # prediction horizon in distance (m)
        self.MPC_range = 500
        self.spatoptimal_range = 300
        # perception range in distance (m)
        self.perception_range = 100
        # number of cell
        # self.num_cell = round((self.cell_range+self.dx)/self.dx)
        # maximum following distance (m), without considering t0*spd_following
        self.max_following_distance = 40
        # minimum following distance (m), without considering t0*spd_following
        self.min_following_distance = 2.5
        # desired time headway
        self.t_0 = 1.5        
        # maximum deceleration
        self.max_dec = -4.5
        # maximum acceleartion
        self.max_acc = 2
        # free flow speed
        self.max_spd = 22
        # minimum gap
        self.min_gap = 2.5
        # max gap or distance
        self.max_gap = 80
        # length of typical car
        self.car_len = 5
        # small value
        self.epsilon = 0.00000001
        # max spacing
        self.max_spacing = self.max_spd * 3.5

        # for the road testing
        self.length_test = 250

        # length of edges in the network
        self.edge1_length = 500
        self.edge2_length = 300
        # self.edge0_length = 500
        
        self.cv_range = 500

        # MPR
        self.MPR = 0.1


# parameters for the lane changing
class LC_Parameter:
    def __init__(self):
        self.target_lane = 1   # target lane for LC, example: from 0 to 1


# class Platoon_numbers:
#     def __init__(self):
#         # set target CV or CAV
#         self.ego_id = target_vehicle
#         # total number of vehicles
#         self.total = 5
#         # MPR
#         self.MPR = 0.4
#         #number of CVs in platoon
#         self.num_CV = self.MPR * self.total
#         #number of HVs
#         self.num_HV = self.total - self.num_CV

class Platoon_numbers:
    def __init__(self):
        # set target CV or CAV
        self.ego_id = target_vehicle
        # total number of vehicles
        self.total = 1
        # MPR
        self.MPR = 0.4
        #number of CVs in platoon
        self.num_CV = self.MPR * self.total
        #number of HVs
        self.num_HV = self.total - self.num_CV




#signal phasing and timing + traffic parameters
class Signal_traffic:
    def __init__(self):
        self.t_green = 15
        self.t_red = 18 + 2 #red and yellow are together one phase
        self.t_cycle = self.t_red + self.t_green
        self.d_sig = 499.9 # location of signal in current scenario
        # self.cell_sig = int(math.floor(d_sig/SimParameter.dx))
        self.tau = 1 # 0.9
        self.lis = 20  # maximum number of vehicles
        self.c = 10.14  #from MATLAB code
