import numpy as np 
import params 
from dynamics import Dynamics
from quadrotor_viewer import QuadRotor_Viewer
from scipy.spatial.transform import Rotation
from mpc import MPC, NMPC, LNMPC
from data_viewer import data_viewer

if __name__=="__main__":
    dynamics = Dynamics(params.dt)
    viewer = QuadRotor_Viewer()
    data_view = data_viewer()
    A, B = dynamics.get_SS(dynamics.state)

    #LNMPC doesn't seem to work any better
    # controller =  MPC(A, B, params.u_max, params.u_min, T=10) #Typicall MPC
    controller =  LNMPC(A, B, params.u_max, params.u_min, T=10) #Non-linear model predictive control relinearizing about the current state
    # controller = NMPC(params.nu_max, params.nu_min) #Way to slow

    t0 = params.t0
    F_eq = params.mass * 9.81
    T_eq = 0.0
    u_eq = np.array([F_eq, T_eq, T_eq, T_eq])

    xr = np.array([5.0, -5.0, -5.0, 0.0, 0.0, 0.0, 0.0, 0.0, np.deg2rad(0), 0.0, 0.0, 0.0]) 
    cmd_idx = [0, 1, 2, 8]
    state = dynamics.state

    while(t0 < params.tf):
        tp = t0 + params.t_plot
        while t0 < tp:
            u = controller.calculateControl(xr, state)
            state = dynamics.updateState(u + u_eq)
            # state = dynamics.updateState(u)
            t0 += params.dt
            if isinstance(controller, LNMPC):
                A, B = dynamics.get_SS(state)
                controller.A = A 
                controller.B = B

        t = state[:3]
        ang = state[6:9]
        R_b_from_i = Rotation.from_euler('ZYX', [ang[2], ang[1], ang[0]]).as_dcm()
        viewer.update(t, R_b_from_i)
        data_view.update(state, xr[cmd_idx], params.t_plot) 

    
    print('Finished')