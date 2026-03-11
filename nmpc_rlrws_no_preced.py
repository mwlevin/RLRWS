from casadi import *
from constants import *
import matplotlib.pyplot as plt
from est_pred_param import *
from copy import deepcopy

def func(x):
    # return 12 / (1 + np.exp(-1 * (x - 33))) + 12 / (1 + np.exp(-1 * (x - 66)))

    return x/3

def func_acc(x):
    return -2.5 / (1 + np.exp(-1 * (x - 33))) -5 / (1 + np.exp(-1 * (x - 66)))


def func_spd_eq(x, spd_limit, pos_stop_bar):
    tau = pos_stop_bar - 40
    v_eq = -spd_limit/(1+np.exp(-0.1*(x-tau)))+spd_limit
    return v_eq

# def func_spd_eq(x, spd_limit, pos_stop_bar):
#     tau = pos_stop_bar - 20
#     v_eq = -spd_limit/(1+np.exp(-0.2*(x-tau)))+spd_limit
#     return v_eq

def nmpc_no_preced(
    num_horizon,
    timestep,
    pos_ego,
    spd_ego,
    pos_stop_bar,
    acc_ego_old,
    tl_status_pred,
    sim_param,
    terminal_constraint=False,
    tl_constraint=True,
    debug_value=False,
): 
    terminal_pos = sim_param.dx
    if terminal_constraint:
        if pos_ego> -60 and pos_ego <=-40:#  in the last third cell
            num_horizon = 50
            terminal_pos = 15
        if pos_ego> -40 and pos_ego <=-20:#  in the last second cell
            num_horizon = 40
            terminal_pos = 10
        if pos_ego>-20: # in the last cell
            num_horizon = 30
            terminal_pos = 5
    
    t0 = sim_param.t_0
    X = SX.sym("X", X_DIM_W * (num_horizon + 1))
    U = SX.sym("U", U_DIM_W * num_horizon)
    constraint = []
    cost = 0

    spd_limit = 32
    
    spd_zero_bound = 2
    
    rl_pos_bound = np.zeros((num_horizon+1))
    
    acc_dot_dot_old = 0
    
    for i in range(0, num_horizon+1):
        if tl_status_pred[i] == 0:
            rl_pos_bound[i] = pos_stop_bar
        else:
            rl_pos_bound[i] = pos_stop_bar+10000    
    
    
    
    
    
    for i in range(0, num_horizon):
        
        
        
        # dynamics: pos
        constraint = vertcat(
            constraint,
            X[X_DIM_W * (i + 1) + X_ID_W["x"]]
            - (X[X_DIM_W * i + X_ID_W["x"]] + X[X_DIM_W * i + X_ID_W["v"]] * timestep),
        )
        # dynamics: spd
        # v_d = func(U[U_DIM_W * i + U_ID_W["sig"]])
        
        # acc = idm(
        #     X[X_DIM_W * i + X_ID_W["v"]],
        #     spd_pred_preced[i],
        #     pos_pred_min_preced[i] - X[X_DIM_W * i + X_ID_W["x"]],
        #     v_0=v_d,
        # )
        
        acc = -U[U_DIM_W * i + U_ID_W["sig"]]/20
        # acc = func_acc(U[U_DIM_W * i + U_ID_W["sig"]])
        
        
        constraint = vertcat(
            constraint,
            X[X_DIM_W * (i + 1) + X_ID_W["v"]]
            - (X[X_DIM_W * i + X_ID_W["v"]] + acc * timestep),
        )
        v_eq = func_spd_eq(X[X_DIM_W * (i + 1) + X_ID_W["x"]], spd_limit, pos_stop_bar)
        cost += 5 * (acc**2) + 200*((acc-acc_ego_old)**2)
        if i == 0:
            acc_dot_dot = acc-acc_ego_old
            acc_dot_dot_old = deepcopy(acc_dot_dot)
        else:
            acc_dot_dot = acc-acc_ego_old
            cost += 1000*((acc_dot_dot - acc_dot_dot_old)**2)
            acc_dot_dot_old = deepcopy(acc_dot_dot)
        
        if tl_constraint and tl_status_pred[i] == 0:
            cost += 0.1*(X[X_DIM_W * (i + 1) + X_ID_W["v"]] - v_eq)**2
        acc_ego_old = deepcopy(acc)
    # cost = cost - 10 * (X[X_DIM_W * (num_horizon) + X_ID_W["x"]] - X[X_DIM_W * (0) + X_ID_W["x"]]) ** 2
    # constraint on position w.r.t. the stop bar position and traffic signal status
    # if traffic signal is red, x(t) + v*t_0 < x_stop_bar
    if tl_constraint:
        for i in range(0, num_horizon):
            constraint = vertcat(
                constraint,
                X[X_DIM_W * (i + 1) + X_ID_W["x"]]
                - (rl_pos_bound[i + 1] - 3*X[X_DIM_W * (i + 1) + X_ID_W["v"]]),
            )
    
    if terminal_constraint:
        gamma_x = SX.sym("gamma_x", 1)
        # vehicle should stop at the last cell
        constraint = vertcat(
            constraint,
            X[X_DIM_W * (num_horizon) + X_ID_W["x"]]
            - (rl_pos_bound[num_horizon] - terminal_pos) + gamma_x,
        )
        gamma_v = SX.sym("gamma_v", 1)
        constraint = vertcat(
            constraint,
            X[X_DIM_W * (num_horizon) + X_ID_W["x"]]
            - (rl_pos_bound[num_horizon]),
        )
        
        
        constraint = vertcat(
            constraint,
            X[X_DIM_W * (num_horizon) + X_ID_W["v"]] - gamma_v,
        )
    opts = {
        "verbose": False,
        "ipopt.print_level": 2,
        "print_time": 0,
        "ipopt.mu_strategy": "adaptive",
        "ipopt.mu_init": 1e-5,
        "ipopt.mu_min": 1e-15,
        "ipopt.barrier_tol_factor": 1e-5,
    }
    if not terminal_constraint:
        nlp = {
            "x": vertcat(X, U),
            "f": cost,
            "g": constraint,
        }
    else:
        nlp = {
            "x": vertcat(X, U, gamma_x, gamma_v),
            "f": cost,
            "g": constraint,
        }
    solver = nlpsol("solver", "ipopt", nlp, opts)
    x = [pos_ego, spd_ego]
    # constraint on location, speed, warning signal
    if not terminal_constraint:
        lbx = x + [-inf, 0] * (num_horizon) + [-20] * (num_horizon)
        ubx = (
            x
            + [inf, spd_limit] * (num_horizon)
            + [100] * (num_horizon)
        )
    else:
        lbx = x + [-inf, 0] * (num_horizon) + [-20] * (num_horizon) + [0] + [0]
        ubx = (
            x
            + [inf, spd_limit] * (num_horizon)
            + [100] * (num_horizon)
            + [inf]
            + [inf]
        )
    lbg_dynamics = (
    # dynamics constraint
    [0] * ((X_DIM_W) * num_horizon))
    ubg_dynamics = (
    # dynamics constraint
    [0] * ((X_DIM_W) * num_horizon))
    if tl_constraint:
        # stop bar position
        ubg_dynamics = (ubg_dynamics + [0] * num_horizon)
        lbg_dynamics = (lbg_dynamics + [-inf] * num_horizon)
    if terminal_constraint:
        # stop in the cell next to stop bar when red
        # ubg_dynamics = (ubg_dynamics + [inf] + [0] + [0])
        # lbg_dynamics = (lbg_dynamics + [0] + [-inf] + [0])
        ubg_dynamics = (ubg_dynamics + [inf] + [0] + [0])
        lbg_dynamics = (lbg_dynamics + [0] + [-inf] + [0])
        cost += 10000*(gamma_x**2) + 10000*(gamma_v**2)
    if terminal_constraint:
        x_guess = np.zeros(((num_horizon + 1) * X_DIM_W + num_horizon * U_DIM_W + 2))
    else:
        x_guess = np.zeros(((num_horizon + 1) * X_DIM_W + num_horizon * U_DIM_W))
    x_guess[X_ID_W["x"]] = pos_ego
    x_guess[X_ID_W["v"]] = spd_ego
    for i in range(1, num_horizon + 1):
        x_guess[X_ID_W["x"] + i * X_DIM_W] = (
            x_guess[X_ID_W["x"] + (i - 1) * X_DIM_W]
            + x_guess[X_ID_W["v"] + (i - 1) * X_DIM_W] * timestep
        )
        x_guess[X_ID_W["v"] + i * X_DIM_W] = x_guess[X_ID_W["v"] + (i - 1) * X_DIM_W]

    sol = solver(
        lbx=lbx,
        ubx=ubx,
        lbg=lbg_dynamics,
        ubg=ubg_dynamics,
        x0=x_guess.tolist(),
    )

    x_sol = np.array(sol["x"])[0 : (num_horizon + 1) * X_DIM_W].reshape(
        (num_horizon + 1, X_DIM_W)
    )

    u_sol = np.array(sol["x"])[
        (num_horizon + 1) * X_DIM_W : ((num_horizon + 1) * X_DIM_W + U_DIM_W * num_horizon)
    ].reshape((num_horizon, U_DIM_W))
    
    
    
    
    
    
    # for index in range(num_horizon):
    #     error = x_sol[index+1, X_ID_W["x"]] - (x_sol[index, X_ID_W["x"]] + timestep*x_sol[index, X_ID_W["v"]])
    #     if abs(error) >= 1e-02:
    #         print("dynamics not satisfy: position dynamics error is ", error)
    #         os.system("pause")
    # debug_value = True
    if debug_value:
        fig, axs = plt.subplots(3)
        
        axs[0].plot(np.arange(0,num_horizon*0.2+0.2,0.2),x_sol[:, X_ID_W["x"]])
        # axs[0].plot(np.arange(0,15.2,0.2),pos_pred_max_preced,'r')
        # axs[0].plot(np.arange(0,15.2,0.2),pos_pred_min_preced,'k')
        
        
        
        axs[0].set_xlabel("Time (s)")
        axs[0].set_ylabel("Position (m)")
        
        # print('x',x_sol[:, X_ID_W["x"]])
        # print('pos boundary',rl_pos_bound)
        
        
        axs[0].plot(np.arange(0,num_horizon*0.2+0.2,0.2), rl_pos_bound, 'r')
        # axs[0].plot(pos_pred_max_preced[i + 1] - t0 * X[X_DIM_W * (i + 1) + X_ID_W["v"]],'b')
        axs[0].set_ylim(-600, 100)
        
        
        axs[1].plot(np.arange(0,num_horizon*0.2+0.2,0.2),x_sol[:, X_ID_W["v"]])
        
        axs[1].set_xlabel("Time (s)")
        axs[1].set_ylabel("Speed (m/s)")
        axs[1].set_ylim(0, 30)
        
        
        axs[2].plot(np.arange(0,num_horizon*0.2,0.2),-u_sol[:, U_ID_W["sig"]]/20)
        
        axs[2].set_xlabel("Time (s)")
        axs[2].set_ylabel("Acceleration (m/s^2)")
        axs[2].set_ylim(-3, 0.1)
        plt.show()


    return x_sol, u_sol