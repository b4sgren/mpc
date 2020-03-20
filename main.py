import numpy as np 
import params 
from dynamics import Dynamics
from quadrotor_viewer import QuadRotor_Viewer
from scipy.spatial.transform import Rotation

if __name__=="__main__":
    dynamics = Dynamics(params.dt)
    viewer = QuadRotor_Viewer()

    t0 = params.t0

    while(t0 < params.tf):
        u = np.zeros(4) #Order is F, Tx, Ty, Tz
        u[0] = 7.848 #Equilibrium force
        u[3] = -1e-3
        state = dynamics.updateState(u)

        t = state[:3]
        ang = state[6:9]
        R = Rotation.from_euler('ZYX', [ang[2], ang[1], ang[0]]).as_dcm()
        viewer.update(t, R)

        t0 += params.dt
    
    print('Finished')