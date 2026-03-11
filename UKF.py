import sys
import matplotlib
import numpy as np
from scipy.linalg import cholesky
from helper import *
from copy import deepcopy
import os



class UnscentedKalmanFilter:
    def __init__(self, dt, param, fn_dynamics, fn_measure):
        # spd; den; param
        # x measurement
        self.vec_x = None
        self.mat_P = None
        self.vec_x_prior = None
        self.vec_z_posterior = None
        self.mat_P_prior = None

        self.sigma_x_prior = None
        self.sigma_z = None
        self.mat_V = None
        self.dt = dt

        self.fn_dynamics = fn_dynamics
        self.fn_measure = fn_measure

        self.sqrt = cholesky
        self.subtract = np.subtract
        self.add = np.add
        self.param = param

        self.tl_index_in_range = None
        self.tl_status_in_range = None
        self.tl_pos_in_range = None
        self.spd_target = None
        self.id_0 = None

    def initialize(self, spd_target):
        est_param = self.param
        num_cell = est_param.num_cell
        num_param = est_param.num_param
        # store all states, initialize speed and density with v0 and rho_0
        spd = (est_param.v0 - est_param.b_v) * est_param.k_v * np.ones((num_cell, 1))
        den = (
            (est_param.rho0 - est_param.b_rho)
            * est_param.k_rho
            * np.ones((num_cell, 1))
        )
        # the first cell's speed is same as the target vehicle's speed
        spd[0, 0] = (spd_target - est_param.b_v) * est_param.k_v
        # c; c0; tau;
        param = np.zeros((num_param, 1))
        for index in range(num_param):
            param[index, 0] = (
                est_param.list_param[index, 0] - est_param.list_param_b[index, 0]
            ) * est_param.list_param_k[index, 0]
        self.vec_x = data2state(spd, den, param, est_param)
        # initialize standard deviation
        mat_std_v = est_param.std_v**2 * np.eye(num_cell)
        mat_std_rho = est_param.std_rho**2 * np.eye(num_cell)
        list_std_param = []
        for index in range(num_param):
            list_std_param.append(est_param.list_param_std[index, 0] ** 2)
        mat_std_param = np.diag(list_std_param)
        self.mat_P = diag_mat(
            [
                mat_std_v,
                mat_std_rho,
                mat_std_param,
            ]
        )
        self.mat_V = deepcopy(self.mat_P)

    def prior_update(
        self,
        tl_pos_in_range,
        tl_status_in_range,
        tl_index_in_range,
        spd_target,
        relative_pos_target,
        debug_value=False,
        is_pass=None,
        tl_index_dict=None,
        tl_pos_dict=None,
    ):
        x_dim = np.size(self.vec_x, 0)
        sigma_x = np.zeros((x_dim, 2 * x_dim))
        n_mat_P = nearestPD(deepcopy(x_dim * deepcopy(self.mat_P)))
        mat_U = self.sqrt(deepcopy(n_mat_P))
        sigma_x[:, 0:x_dim] = self.add(
            deepcopy(self.vec_x[:, 0:1]), deepcopy(mat_U[:, 0:x_dim])
        )
        sigma_x[:, x_dim : 2 * x_dim] = self.subtract(
            deepcopy(self.vec_x[:, 0:1]), deepcopy(mat_U[:, 0:x_dim])
        )
        sigma_x_inverse_tf = deepcopy(sigma_x)
        for index in range(x_dim):
            # inverse transform
            sigma_x_inverse_tf[:, index : index + 1] = inverse_transform(
                deepcopy(sigma_x_inverse_tf[:, index : index + 1]), self.param
            )
            sigma_x_inverse_tf[
                :, index + x_dim : index + 1 + x_dim
            ] = inverse_transform(
                deepcopy(sigma_x_inverse_tf[:, index + x_dim : index + 1 + x_dim]),
                self.param,
            )
            # clip
            sigma_x_inverse_tf[:, index : index + 1] = clip_state(
                deepcopy(sigma_x_inverse_tf[:, index : index + 1]), self.param
            )
            sigma_x_inverse_tf[:, index + x_dim : index + 1 + x_dim] = clip_state(
                deepcopy(sigma_x_inverse_tf[:, index + x_dim : index + 1 + x_dim]),
                self.param,
            )
        # compute the prior sigma-points
        sigma_x_prior = np.zeros((x_dim, 2 * x_dim))
        num_cell = self.param.num_cell
        dx = self.param.dx
        self.tl_index_in_range = tl_index_in_range
        self.tl_status_in_range = tl_status_in_range
        index_target = int(relative_pos_target / dx + 1)
        if len(tl_pos_in_range) == 0:
            tl_close_status = 0
            id_tl_close_target = 0
        else:
            tl_close_status = tl_status_in_range[0]
            id_tl_close_target = 0
            
        self.tl_pos_in_range = tl_pos_in_range
        self.spd_target = spd_target
        for index in range(x_dim):
            sigma_x_prior[:, index : index + 1],_ = self.fn_dynamics(
                deepcopy(sigma_x_inverse_tf[:, index : index + 1]),
                self.param,
                tl_status_in_range,
                spd_target,
                index_target,
                tl_index_in_range,
                tl_close_status,
                id_tl_close_target,
                is_pass=is_pass,
                tl_index_dict=tl_index_dict,
                tl_pos_dict=tl_pos_dict,
            )
            sigma_x_prior[:, index + x_dim : index + 1 + x_dim],_ = self.fn_dynamics(
                deepcopy(sigma_x_inverse_tf[:, index + x_dim : index + 1 + x_dim]),
                self.param,
                tl_status_in_range,
                spd_target,
                index_target,
                tl_index_in_range,
                tl_close_status,
                id_tl_close_target,
                is_pass=is_pass,
                tl_index_dict=tl_index_dict,
                tl_pos_dict=tl_pos_dict,
            )

            sigma_x_prior[:, index : index + 1] = clip_state(
                deepcopy(sigma_x_prior[:, index : index + 1]), self.param
            )

            sigma_x_prior[:, index + x_dim : index + 1 + x_dim] = clip_state(
                deepcopy(sigma_x_prior[:, index + x_dim : index + 1 + x_dim]),
                self.param,
            )
            # linear transform
            sigma_x_prior[:, index : index + 1] = transform(
                deepcopy(sigma_x_prior[:, index : index + 1]), self.param
            )
            sigma_x_prior[:, index + x_dim : index + 1 + x_dim] = transform(
                deepcopy(sigma_x_prior[:, index + x_dim : index + 1 + x_dim]),
                self.param,
            )
        # compute the prior statistics
        self.vec_x_prior = np.average(deepcopy(sigma_x_prior), axis=1)
        self.vec_x_prior = self.vec_x_prior.reshape((len(self.vec_x_prior), 1))
        self.mat_P_prior = np.zeros((x_dim, x_dim))
        for index in range(x_dim):
            delta_0 = self.subtract(
                deepcopy(sigma_x_prior[:, index : index + 1]),
                deepcopy(self.vec_x_prior[:, 0:1]),
            )
            self.mat_P_prior = self.add(
                deepcopy(self.mat_P_prior),
                np.dot(deepcopy(delta_0), deepcopy(delta_0).T),
            )
            delta_1 = self.subtract(
                deepcopy(sigma_x_prior[:, index + x_dim : index + 1 + x_dim]),
                deepcopy(self.vec_x_prior[:, 0:1]),
            )
            self.mat_P_prior = self.add(
                deepcopy(self.mat_P_prior),
                np.dot(deepcopy(delta_1), deepcopy(delta_1).T),
            )
        self.mat_P_prior = deepcopy(self.mat_P_prior) / (2 * x_dim)
        self.mat_P_prior = nearestPD(
            self.add(deepcopy(self.mat_P_prior), deepcopy(self.mat_V))
        )
        self.sigma_x_prior = deepcopy(sigma_x_prior)

    def posteriori_update(
        self, pos_ego, spd_ego, pos_preced, spd_preced, pos_CV, spd_CV, debug_value=False, 
    ):
        x_dim = np.size(self.vec_x, 0)
        num_CV = 1 # must have ego vehicle
        if pos_preced is not None:
            num_CV += 1
        num_CV += len(pos_CV)
        mat_W = self.param.std_measure**2 * np.eye(num_CV)
        sigma_z = np.zeros((num_CV, 2 * x_dim))
        
        for index in range(x_dim):
            sigma_tmp = deepcopy(self.sigma_x_prior[:, index : index + 1])
            sigma_z[:, index : index + 1] = self.fn_measure(
                deepcopy(sigma_tmp),
                pos_ego,
                pos_preced,
                pos_CV,
                self.param,
                num_CV,
            )
            sigma_tmp = deepcopy(
                self.sigma_x_prior[:, index + x_dim : index + 1 + x_dim]
            )
            sigma_z[:, index + x_dim : index + 1 + x_dim] = self.fn_measure(
                deepcopy(sigma_tmp),
                pos_ego,
                pos_preced,
                pos_CV,
                self.param,
                num_CV,
            )
        self.sigma_z = deepcopy(sigma_z)
        vec_z_posterior = np.average(deepcopy(sigma_z), axis=1)
        vec_z_posterior = vec_z_posterior.reshape((len(vec_z_posterior), 1))
        mat_P_zz = np.zeros((num_CV, num_CV))
        for index in range(x_dim):
            delta_0 = self.subtract(
                sigma_z[:, index : index + 1], vec_z_posterior[:, 0:1]
            )
            mat_P_zz = self.add(
                deepcopy(mat_P_zz), np.dot(deepcopy(delta_0), deepcopy(delta_0).T)
            )

            delta_1 = self.subtract(
                sigma_z[:, index + x_dim : index + 1 + x_dim], vec_z_posterior[:, 0:1]
            )

            mat_P_zz = self.add(
                deepcopy(mat_P_zz), np.dot(deepcopy(delta_1), deepcopy(delta_1).T)
            )
        mat_P_zz = deepcopy(mat_P_zz) / (2 * x_dim)
        mat_P_zz = self.add(deepcopy(mat_P_zz), deepcopy(mat_W))
        mat_P_zz = nearestPD(deepcopy(mat_P_zz))
        mat_P_xz = np.zeros((x_dim, num_CV))
        for index in range(x_dim):
            delta_0 = self.subtract(
                self.sigma_x_prior[:, index : index + 1], self.vec_x_prior[:, 0:1]
            )
            delta_1 = self.subtract(
                sigma_z[:, index : index + 1], vec_z_posterior[:, 0:1]
            )
            mat_P_xz = self.add(deepcopy(mat_P_xz), np.dot(delta_0, delta_1.T))
            delta_0 = self.subtract(
                self.sigma_x_prior[:, index + x_dim : index + 1 + x_dim],
                self.vec_x_prior[:, 0:1],
            )
            delta_1 = self.subtract(
                sigma_z[:, index + x_dim : index + 1 + x_dim], vec_z_posterior[:, 0:1]
            )
            mat_P_xz = self.add(deepcopy(mat_P_xz), np.dot(delta_0, delta_1.T))
        mat_P_xz = deepcopy(mat_P_xz) / (2 * x_dim)
        mat_K = np.dot(mat_P_xz, np.linalg.inv(mat_P_zz))
        vec_z = np.zeros((num_CV, 1))
        count = 0
        vec_z[count, 0] = (spd_ego - self.param.b_v) * self.param.k_v
        count += 1
        if pos_preced is not None:
            vec_z[count, 0] = (spd_preced - self.param.b_v) * self.param.k_v
            count += 1
        
        
        for index in range(len(pos_CV)):
            vec_z[count, 0] = (spd_CV[index] - self.param.b_v) * self.param.k_v
            count += 1

        vec_x_m = self.add(
            deepcopy(self.vec_x_prior),
            np.dot(mat_K, self.subtract(vec_z, vec_z_posterior)),
        )
        vec_x_m = inverse_transform(deepcopy(vec_x_m), self.param)
        vec_x_previous = inverse_transform(deepcopy(self.vec_x), self.param)
        # spd can not change abruptly
        vec_x_m[0 : self.param.num_cell, 0:1] = np.clip(
            vec_x_m[0 : self.param.num_cell, 0:1],
            vec_x_previous[0 : self.param.num_cell, 0:1] - 0.1*10,
            vec_x_previous[0 : self.param.num_cell, 0:1] + 0.1*5,
        )
        # density can not change abruptly
        vec_x_m[self.param.num_cell : 2*self.param.num_cell, 0:1] = np.clip(
            vec_x_m[self.param.num_cell : 2*self.param.num_cell, 0:1],
            vec_x_previous[self.param.num_cell : 2*self.param.num_cell, 0:1] - 0.1*0.1,
            vec_x_previous[self.param.num_cell : 2*self.param.num_cell, 0:1] + 0.1*0.1,
        )
        for index in range(self.param.num_cell):
            if index in self.tl_index_in_range:
                if self.tl_status_in_range[self.tl_index_in_range.index(index)] == 0:
                    vec_x_m[index, 0] = 0        
        vec_x_m = clip_state(deepcopy(vec_x_m), self.param)
        vec_x_m = transform(deepcopy(vec_x_m), self.param)
        self.vec_x = deepcopy(vec_x_m)
        mat_P_m = deepcopy(self.mat_P_prior) - np.dot(np.dot(mat_K, mat_P_zz), mat_K.T)
        self.mat_P = deepcopy(mat_P_m)
