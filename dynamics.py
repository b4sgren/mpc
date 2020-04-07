import numpy as np
import params 
from scipy.spatial.transform import Rotation
import scipy.signal as ss
from tools import Euler2Rotation

#Need to perturb some of the parameters such as mass, J

def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

class Dynamics:
    def __init__(self, Ts):
        self.dt = Ts

        self.state = np.array([params.pn0, params.pe0, params.pd0, params.vx0, params.vy0, params.vz0, params.phi0, params.theta0, params.psi0, params.p0, params.r0, params.psi0])
        self.e3 = np.array([0, 0, 1.0])
        self.Cd = 0.2 #Drag coefficient
        self.g = 9.81
        self.m = params.mass 
        self.J = np.diag([params.Jx, params.Jy, params.Jz])
    
    def updateState(self, u):
        k1 = self.derivatives(self.state, u)
        k2 = self.derivatives(self.state + k1 * self.dt/2.0, u)
        k3 = self.derivatives(self.state + k2 * self.dt/2.0, u)
        k4 = self.derivatives(self.state + k3 * self.dt, u)
        self.state += (k1 + 2 * k2 + 2 * k3 + k4) * self.dt/6.0

        return self.state

    def derivatives(self, state, u):
        # Dynamics come from https://scholarsarchive.byu.edu/cgi/viewcontent.cgi?article=2324&context=facpub
        xdot = np.zeros(12)
        phi = state[6]
        theta = state[7]
        psi = state[8]
        sp = np.sin(phi)
        cp = np.cos(phi)
        st = np.sin(theta)
        ct = np.cos(theta)
        cpsi = np.cos(psi)
        spsi = np.sin(psi)

        R_i_from_b = Rotation.from_euler("ZYX", [psi, theta, phi]).as_dcm()
        R = Euler2Rotation(phi, theta, psi)
        forces = np.array([0, 0, 0, 0, 0, -u[0]/self.m, 0, 0, 0, u[1]/self.J[0,0], u[2]/self.J[1,1], u[3]/self.J[2,2]]) 

        v = state[3:6]
        w = state[9:]
        p_dot = R_i_from_b @ v
        v_dot = np.cross(v, w) + R_i_from_b.T @ (self.g * self.e3) - self.Cd * v  #Add a drag term?

        S = np.array([[1.0, sp * st / ct, cp * st / ct],
                      [0.0, cp, -sp],
                      [0.0, sp / ct, cp / ct]])
        ang_dot = S @ w 
        w_dot = np.linalg.inv(self.J) @ np.cross(-w, (self.J @ w))
        Jx, Jy, Jz = self.J[0,0], self.J[1,1], self.J[2,2]

        xdot[:3] = p_dot
        xdot[3:6] = v_dot
        xdot[6:9] = ang_dot 
        xdot[9:] = w_dot 
        xdot += forces

        return xdot
    
    def get_SS(self, state): #This appears to match what Mat has. Need to run his matlab code to be sure
        v = state[3:6]
        phi = state[6]
        theta = state[7]
        psi = state[8]
        w = state[9:]

        ct = np.cos(theta)
        st = np.sin(theta)
        cp = np.cos(phi)
        sp = np.sin(phi)

        A = np.zeros((12, 12))

        dpd_dv = Rotation.from_euler('ZYX', [psi, theta, phi]).as_dcm()
        A[:3, 3:6] = dpd_dv 

        dvd_dv = -skew(w)
        dvd_dw = skew(v)
        dvd_dang = np.array([[0, -self.g * ct, 0], [self.g * ct * cp, -self.g * st * sp, 0], [-self.g * ct * sp, -self.g * st * cp, 0]])
        # A[3:6, 3:6] = dvd_dv  #Mat isn't using this. Will try adding it later
        A[3:6, 6:9] = dvd_dang 
        # A[3:6, 9:] = dvd_dw #Mat isn't using this. Will try adding it later

        dangd_dw = np.array([[1.0, sp * st / ct, cp * st / ct],
                      [0.0, cp, -sp],
                      [0.0, sp / ct, cp / ct]])
        # A[6:9, 9:] = dangd_dw
        A[6:9, 9:] = np.eye(3) #Mat is doing this not the line above. Will try switching it later

        dwd_dw = np.linalg.inv(self.J) @ (-skew(self.J@w) @ (-np.eye(3)) + skew(-w) @ self.J)
        # A[9:, 9:] = dwd_dw #Mat isn't using this. Will try adding it later

        B = np.zeros((12, 4))

        dvd_du = np.zeros((3,4))
        dvd_du[2,0] = -1/self.m
        B[3:6] = dvd_du 

        dwd_du = np.zeros((3,4))
        dwd_du[:,1:] = np.diag([1/self.J[0,0], 1/self.J[1,1], 1/self.J[2,2]])        
        B[9:] = dwd_du 

        C = np.eye(12) #Full state Feedback
        D = np.zeros((12,4))

        sysd = ss.cont2discrete((A, B, C, D), self.dt) #Get the discrete time system
        Ad = sysd[0]
        Bd = sysd[1]

        # return A, B
        return Ad, Bd 

if __name__=="__main__":
    x_eq = np.zeros((12))
    quad = Dynamics(params.dt)
    A, B = quad.get_SS(x_eq) #About the equilibrium all of then A is correct

    J = quad.J 
    Jx = J[0,0]
    Jy = J[1,1]
    Jz = J[2,2]
    w = x_eq[-3:]
    temp = np.array([[0, (Jy - Jz)/Jx * w[2], (Jy - Jz)/Jx * w[1]], [(Jz - Jx)/Jy * w[2], 0, (Jz - Jx)/Jy * w[0]], [(Jx - Jy)/Jz * w[2], (Jx - Jy)/Jz * w[0], 0]])

    x = np.array([0, 0, 0, 1.0, -1.0, 0.1, np.deg2rad(5.0), np.deg2rad(-5.0), np.deg2rad(45.0), 0.01, -0.01, 0.0])
    A, B = quad.get_SS(x)
    w = x[-3:]
    temp = np.array([[0, (Jy - Jz)/Jx * w[2], (Jy - Jz)/Jx * w[1]], [(Jz - Jx)/Jy * w[2], 0, (Jz - Jx)/Jy * w[0]], [(Jx - Jy)/Jz * w[2], (Jx - Jy)/Jz * w[0], 0]])