import numpy as np
import params 
from scipy.spatial.transform import Rotation

class Dynamics:
    def __init__(self, Ts):
        self.dt = Ts

        #State will use a quaternion in it
        self.state = np.array([params.pn0, params.pe0, params.pd0, params.vx0, params.vy0, params.vz0, params.phi0, params.theta0, params.psi0, params.p0, params.r0, params.psi0])
        self.e3 = np.array([0, 0, 1.0])
        self.Cd = 0.3 #Not sure what this should be. Using this for now
        self.g = 9.81
        self.m = params.mass 
        self.J = np.diag([params.Jx, params.Jy, params.Jz])
    
    def updateState(self, u):
        k1 = self.derivatives(self.state, u)
        k2 = self.derivatives(self.state + k1 * self.dt/2.0, u)
        k3 = self.derivatives(self.state + k2 * self.dt/2.0, u)
        k4 = self.derivatives(self.state + k3 * self.dt, u)
        self.state += (k1 + 2 * k2 + 2 * k3 + k4) * self.dt/6.0

    def derivatives(self, state, u):
        xdot = np.zeros(12)
        phi = state[6]
        theta = state[7]
        psi = state[8]
        R_i_from_b = Rotation.from_euler("ZYX", [psi, theta, phi]).as_dcm()

        v = state[3:6]
        w = state[9:]
        p_dot = R_i_from_b @ v
        v_dot = -np.cross(w, v).T + R_i_from_b.T @ (self.g * self.e3)

        sp = np.sin(phi)
        cp = np.cos(phi)
        st = np.sin(theta)
        ct = np.cos(theta)
        S = np.array([[1.0, sp * st / ct, cp * st / ct],
                      [0.0, cp, -sp],
                      [0.0, sp / ct, cp / ct]])
        ang_dot = S @ w 
        w_dot = np.linalg.inv(self.J) @ np.cross(-w, (self.J @ w).T).T

        #Above doesn't include the forces. Need to add those in

        return xdot