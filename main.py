import numpy as np 
import params 
from dynamics import Dynamics
from quadrotor_viewer import QuadRotor_Viewer
from scipy.spatial.transform import Rotation
from mpc import MPC, NMPC, LNMPC
from data_viewer import data_viewer
import time

if __name__=="__main__":
    dynamics = Dynamics(params.dt)
    # viewer = QuadRotor_Viewer()
    data_view = data_viewer()
    A, B = dynamics.get_SS(dynamics.state)

    #LNMPC doesn't seem to work any better
    # controller =  MPC(A, B, params.u_max, params.u_min, T=params.T) #Typicall MPC
    controller =  LNMPC(A, B, params.u_max, params.u_min, T=params.T) #Non-linear model predictive control relinearizing about the current state
    # controller = NMPC(params.nu_max, params.nu_min, T = params.T) #Way to slow

    t0 = params.t0
    F_eq = params.mass * 9.81
    T_eq = 0.0
    u_eq = np.array([F_eq, T_eq, T_eq, T_eq])

    xr = np.array([5.0, -5.0, -5.0, 0.0, 0.0, 0.0, 0.0, 0.0, np.deg2rad(0), 0.0, 0.0, 0.0]) 
    # x_ref = np.array([xr for i in range(params.T+1)]).T
    cmd_idx = [0, 1, 2, 8]
    state = dynamics.state

    x_hist = []
    t_hist = []

    while(t0 < params.tf):
        tp = t0 + params.t_plot
        while t0 < tp:
            x_hist.append(state.copy())
            x_ref = np.array([[5 * np.cos(0.2 * (t0 + i * params.dt)), 5 * np.sin(0.2 * (t0+i*params.dt)), -10 - (t0+i*params.dt) * 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0] for i in range(params.T+1)]).T
            u = controller.calculateControl(x_ref, state)
            state = dynamics.updateState(u + u_eq)
            # state = dynamics.updateState(u)
            t0 += params.dt
            if isinstance(controller, LNMPC):
                A, B = dynamics.get_SS(state)
                controller.A = A 
                controller.B = B
        if t0 >8.0:
            # x_ref = np.array([-xr for i in range(params.T+1)]).T
            debug = 1
        t = state[:3]
        ang = state[6:9]
        R_b_from_i = Rotation.from_euler('ZYX', [ang[2], ang[1], ang[0]]).as_dcm()
        # viewer.update(t, R_b_from_i)
        data_view.update(state, x_ref[cmd_idx, 0], params.t_plot) 
    
    viewer = QuadRotor_Viewer()
    input('Setup screen recorder')
    for x in x_hist:
        trans = x[:3]
        ang = x[6:9]
        R_b_from_i = Rotation.from_euler('ZYX', [ang[2], ang[1], ang[0]]).as_dcm()
        viewer.update(trans, R_b_from_i)
        time.sleep(params.dt)

    print('Finished')
    input('Press enter to end program')