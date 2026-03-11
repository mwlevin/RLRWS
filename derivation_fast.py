# import sympy
# from sympy import *
import math
from ILC_constants import IDM_Param, Veh_Parameter
import time

# copied from main text
# find the estimated acceleration, modified IDM formula
con_range = Veh_Parameter().cv_range
small_num_warning = IDM_Param().small_num_warning
def v_dot_value(v0, delta, warning, acc_ego , spd_ego, spacing_ego, tls, a_fix , d_fix, c_fix, T_fix):
    v_dot = a_fix * w_value(v0, delta, spd_ego, spacing_ego, tls, T_fix) - d_fix * warning ** c_fix
    return v_dot
    

# find the value of w (inside IDM paranthesis), used for finding v_dot
def w_value(v0, delta, spd, spacing, tls, T):
    if tls == 1: # traffic light to be green, spacing is easier
        w = w = 1 - (spd/v0)**delta - ( (spacing ) / (spacing) )**2
    
    else: # red light
        if spacing > con_range: # far from intersection, spacing has no effect
            w = w = 1 - (spd/v0)**delta - ( (spacing ) / (spacing) )**2
        
        else:
            w = 1 - (spd/v0)**delta - ( (spacing + spd * T) / (spacing) )**2
    return w





# this file obtains partial derivation of the s function in respect to each parameter. This file is named fast because the aritmatic is done before and the
# final terms are substituded. 

# strategy is if the value of "-speed**4/v0**4 + 1 - accel/a > 0", we set low values for b, s, T derivstive until a and v values are adjusted.
class Derivation_class:
    def __init__(self):
        self.small_num = 1   # samall number
        self.large_num = 1000000
        self.counter = 0
        self.sum = 0
        self.buffer2 = 1.5



        # function gets derivative in terms of a, parametrs are d , c , T , other argumets are just number (scalar)
    def a_derivative(self, v0, delta, warning, acc_ego , spd_ego, spacing_ego, tls, a_fix , d_fix, c_fix, T_fix):

        # check value of a
        # if (1 - (speed / v0_value)**delta - (accel / a_value)) <= 0:
        #     a_value = accel / (1 - ((speed / v0_value)**delta )) * self.buffer2
        warning = max(small_num_warning, warning)
        w = w_value(v0, delta, spd_ego, spacing_ego, tls, T_fix)
        v_dot_hat = v_dot_value(v0, delta, warning, acc_ego , spd_ego, spacing_ego, tls, a_fix , d_fix, c_fix, T_fix) 

        number = 2 * v_dot_hat * w -  2 * acc_ego * w
        return number




        # function gets derivative in terms of b, parametrs are s0,b T, v0  , other argumets are just number
    def d_derivative(self, v0, delta, warning, acc_ego , spd_ego, spacing_ego, tls, a_fix , d_fix, c_fix, T_fix):

        # # find the value at any point x
        # if (-speed**4/v0**4 + 1 - accel/a) > 0 :
        #     helper = -speed**4/v0**4 + 1 - accel/a
        # else:
        #     helper = self.large_num
        
        warning = max(small_num_warning, warning)
        
        v_dot_hat = v_dot_value(v0, delta, warning, acc_ego , spd_ego, spacing_ego, tls, a_fix , d_fix, c_fix, T_fix)
        # print(" insde d derivative, v dot hat is ", v_dot_hat, " warning is ", warning, " c fix is ", c_fix, " acc ego is ", acc_ego)
        number = - 2 * v_dot_hat * warning ** c_fix + 2 * acc_ego * warning ** c_fix

        return number
    
    
    
            # function gets derivative in terms of a, parametrs are d , c , T , other argumets are just number (scalar)
    def c_derivative(self, v0, delta, warning, acc_ego , spd_ego, spacing_ego, tls, a_fix , d_fix, c_fix, T_fix):

        # check value of a
        # if (1 - (speed / v0_value)**delta - (accel / a_value)) <= 0:
        #     a_value = accel / (1 - ((speed / v0_value)**delta )) * self.buffer2
        
        warning = max(small_num_warning, warning)
        
        v_dot_hat = v_dot_value(v0, delta, warning, acc_ego , spd_ego, spacing_ego, tls, a_fix , d_fix, c_fix, T_fix) 

        number = -2 * v_dot_hat * (d_fix * warning**c_fix * math.log( warning)) + 2 * acc_ego * (d_fix * warning**c_fix * math.log(warning))
        return number
    
    
    
        # function gets derivative in terms of a, parametrs are d , c , T , other argumets are just number (scalar)
    def T_derivative(self, v0, delta, warning, acc_ego , spd_ego, spacing_ego, tls, a_fix , d_fix, c_fix, T_fix):

        # check value of a
        # if (1 - (speed / v0_value)**delta - (accel / a_value)) <= 0:
        #     a_value = accel / (1 - ((speed / v0_value)**delta )) * self.buffer2
        
        warning = max(small_num_warning, warning)
        
        w = w_value(v0, delta, spd_ego, spacing_ego, tls, T_fix)
        v_dot_hat = v_dot_value(v0, delta, warning, acc_ego , spd_ego, spacing_ego, tls, a_fix , d_fix, c_fix, T_fix) 

        number = 2* v_dot_hat * ( -2* spd_ego * a_fix / spacing_ego - 2 * spd_ego**2 * T_fix * a_fix / spacing_ego**2) + 2 * acc_ego * ( -2* spd_ego * a_fix / spacing_ego - 2 * spd_ego**2 * T_fix * a_fix / spacing_ego**2)
        return number








    #     # function gets derivative in terms of v0, parametrs are s0,b T, v0  , other argumets are just number
    # def v0_derivative(self, s0_value, a_value, b_value, T_value, v0_value, delta, speed, rel_speed, accel):
    #     a = a_value
    #     b = b_value
    #     T = T_value
    #     s0 = s0_value
    #     v0 = v0_value

    #     # if (1 - (speed / v0_value)**delta - (accel * a_value**-1)) <= 0:
    #     #     v0_value = speed * max(self.small_num, (1 - (accel * a**-1))**-0.25 ) * self.buffer2


    #     if -speed**4/v0**4 + 1 - accel/a > 0.1 :
    #         helper = -speed**4/v0**4 + 1 - accel/a
    #     else:
    #         helper = self.large_num
    #     # find the value at any point x
    #     number = -2.0*speed**4*(T*speed + rel_speed*speed/(2*a**0.5*b**0.5) + s0)/(v0**5*(helper)**1.5)

    #     return number






    #     # function gets derivative in terms of T, parametrs are s0,b T, v0  , other argumets are just number
    # def T_derivative(self, s0_value, a_value, b_value, T_value, v0_value, delta, speed, rel_speed, accel):
    #     # s0, a, b, T, v0= symbols('s0, a, b, T, v0', real=True)
    #     a = a_value
    #     v0 = v0_value

    #     # find the value at any point x
    #     if -speed**4/v0**4 + 1 - accel/a > 0.1 :
    #         helper = -speed**4/v0**4 + 1 - accel/a
    #     else:
    #         helper = self.large_num
    #     number = speed/(helper)**0.5


    #     return number


        # get the run time average
        # # Record the start time
        # start_time = time.time() 
        # self.counter = self.counter + 1

        # # Calculate the elapsed time
        # elapsed_time = time.time() - start_time
        # self.sum = self.sum + elapsed_time

        # if self.counter % 100 == 0:
        #     print("a average Time taken to run the line of code: {:.6f} seconds".format(self.sum/self.counter))