import numpy as np 
import params 
from dynamics import Dynamics
from quadrotor_viewer import QuadRotor_Viewer
from scipy.spatial.transform import Rotation
from mpc import MPC
from data_viewer import data_viewer
# from tools import Euler2Rotation

if __name__=="__main__":
    dynamics = Dynamics(params.dt)
    viewer = QuadRotor_Viewer()
    data_view = data_viewer()
    A, B = dynamics.get_SS(dynamics.state)

    controller =  MPC(A, B, params.u_max, params.u_min)

    t0 = params.t0
    F_eq = params.mass * 9.81
    T_eq = 0.0
    u_eq = np.array([F_eq, T_eq, T_eq, T_eq])

    xr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 
    state = dynamics.state

    while(t0 < params.tf):
        tp = t0 + params.t_plot
        while t0 < tp:
            u = controller.calculateControl(xr, state)
            # u = np.zeros(4) #Order is F, Tx, Ty, Tz
            # u[0] = F_eq
            # u[1] = T_eq
            state = dynamics.updateState(u + u_eq)
            t0 += params.dt

        t = state[:3]
        # if t0 > 2:
            # t[2] = 10.0
        ang = state[6:9]
        R_b_from_i = Rotation.from_euler('ZYX', [ang[2], ang[1], ang[0]]).as_dcm()
        viewer.update(t, R_b_from_i)
        data_view.update(state, params.t_plot)

    
    print('Finished')