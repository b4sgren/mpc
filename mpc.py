import numpy as np 
import cvxpy as cp 
import params
import pyoptsparse #Will use SNOPT for the non-linear MPC
from scipy.spatial.transform import Rotation

'''
TODO:
1. Add ability to track trajectories instead of just waypoints
2. Try augmenting tuning by adding penalty for inputs
3. Implement non-linear model using pyoptsparse (different controller)
'''

class MPC:
    def __init__(self, A, B, u_max, u_min, T=10): 
        self.A = A 
        self.B = B 
        self.u_max = u_max 
        self.u_min = u_min 
        self.Q = np.diag([10.0, 10.0, 2.0, 1.0, 1.0, 0.1, 10.0, 10.0, 50.0, 1.0, 1.0, 3.0]) #Weighting matrix for the states (x_ref - x_k)
        self.R = np.diag([1e-3, 1e-2, 1e-2, 1e-2])   #Weighting matrix for the inputs (penalize high control effort)
        self.T = T #Time horizon

        self.n = self.A.shape[0] #Number of states 
        self.m = self.B.shape[1] #Number of inputs
    
    def calculateControl(self, xr, x0): #xr (n x T+1) is the reference to track and x0 is the current state
        x = cp.Variable((self.n, self.T+1), name="States")
        u = cp.Variable((self.m, self.T), name="Inputs")

        cost = 0
        constr = []
        phi_theta_max = np.array([np.pi/4, np.pi/4])
        for t in range(self.T):
            cost += cp.quad_form(xr[:,t+1] - x[:,t+1], self.Q) + cp.quad_form(u[:,t], self.R) 
            constr += [x[:,t+1] == self.A @ x[:,t] + self.B @ u[:,t]] 
            constr += [x[6:8, t+1] <= phi_theta_max, x[6:8, t+1] >= -phi_theta_max] #Max constraint on roll and pitch angles
            constr += [u[:,t] <= self.u_max,  u[:,t] >= self.u_min] #Constraint on the inputs
        constr += [x[:,0] == x0] # First state must be what is passed in

        problem = cp.Problem(cp.Minimize(cost), constr)
        problem.solve(solver=cp.ECOS)

        if problem.status == cp.OPTIMAL:
            return u.value[:,0]
        else:
            print("Not optimal")
            return np.array([params.mass * 9.81, 0.0, 0.0, 0.0]) #Equilbrium

class LNMPC: #This class will relinearize about the current state
    def __init__(self, A, B, u_max, u_min, T=10): 
        self.A = A 
        self.B = B 
        self.u_max = u_max 
        self.u_min = u_min 
        self.Q = np.diag([10.0, 10.0, 2.0, 1.0, 1.0, 0.1, 10.0, 10.0, 50.0, 1.0, 1.0, 3.0]) #Weighting matrix for the states (x_ref - x_k)
        self.R = np.diag([1e-3, 1e-2, 1e-2, 1e-2])   #Weighting matrix for the inputs (penalize high control effort)
        self.T = T #Time horizon

        self.n = self.A.shape[0] #Number of states 
        self.m = self.B.shape[1] #Number of inputs
    
    def calculateControl(self, xr, x0): #xr (n x T+1) is the reference to track and x0 is the current state
        x = cp.Variable((self.n, self.T+1), name="States")
        u = cp.Variable((self.m, self.T), name="Inputs")

        cost = 0
        constr = []
        phi_theta_max = np.array([np.pi/4, np.pi/4])
        for t in range(self.T):
            cost += cp.quad_form(xr[:,t+1] - x[:,t+1], self.Q) + cp.quad_form(u[:,t], self.R) 
            constr += [x[:,t+1] == self.A @ x[:,t] + self.B @ u[:,t]] 
            constr += [x[6:8, t+1] <= phi_theta_max, x[6:8, t+1] >= -phi_theta_max] #Max constraint on roll and pitch angles
            constr += [u[:,t] <= self.u_max,  u[:,t] >= self.u_min] #Constraint on the inputs
        constr += [x[:,0] == x0] # First state must be what is passed in

        problem = cp.Problem(cp.Minimize(cost), constr)
        problem.solve(solver=cp.ECOS)

        if problem.status == cp.OPTIMAL:
            return u.value[:,0]
        else:
            print("Not optimal")
            return np.array([params.mass * 9.81, 0.0, 0.0, 0.0]) #Equilbrium

class NMPC:
    def __init__(self, u_max, u_min, T=10):
        self.T = T 
        self.u_max = u_max.tolist()
        self.u_min = u_min.tolist() 
        self.Q = np.diag([10.0, 10.0, 2.0, 0, 0, 0, 10.0, 10.0, 50.0, 1.0, 1.0, 1.0]) #Weighting matrix for the states (x_ref - x_k)
        self.R = np.diag([0.1, 0.1, 0.1, 0.1])
        self.u_eq = [params.mass * 9.81, 0.0, 0.0, 0.0] #This will always be my starting guess
        self.rp_max = [np.pi/4, np.pi/4]
        self.rp_min = [-np.pi/4, np.pi/4]
        self.dt = params.dt

        self.m = params.mass 
        self.J = np.diag([params.Jx, params.Jy, params.Jz])
        self.g = 9.81
        self.e3 = np.array([0, 0, 1])
        self.Cd = 0.2 #not sure that I will be using this. At least will change the value
    
    def __call__(self, xdict):
        u = xdict['xvars'].reshape((4, self.T), order='F')
        x = []
        x.append(self.x0.copy())
        funcs = {}

        #Calculate the states for the given inputs
        for i in range(self.T):
            x_dot = self.f(x[-1], u[:,i])
            x_k = x[-1] + x_dot * self.dt
            x.append(x_k)
        
        #Calculate the cost function
        cost = 0
        for xk in x:
            diff = xk - self.x_ref
            cost += diff @ self.Q @ diff
        
        for i in range(self.T):
            uk = u[:,i]
            # cost += uk @ self.R @ uk #Not using this while I get it working
        funcs['obj'] = cost

        # Make sure roll and pitch constraints are met
        x = np.array(x)
        rp = x[6:8, 1:].flatten(order='F')
        funcs['rp'] = rp

        return funcs, False
    
    def f(self, state, u):
        xdot = np.zeros(12, dtype=type(u[0]))
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
        forces = np.array([0, 0, 0, 0, 0, -u[0]/self.m, 0, 0, 0, u[1]/self.J[0,0], u[2]/self.J[1,1], u[3]/self.J[2,2]], dtype=type(u[0])) 

        v = state[3:6]
        w = state[9:]
        p_dot = R_i_from_b @ v
        v_dot = np.cross(v, w) + R_i_from_b.T @ (self.g * self.e3) - self.Cd * v  

        S = np.array([[1.0, sp * st / ct, cp * st / ct],
                      [0.0, cp, -sp],
                      [0.0, sp / ct, cp / ct]])
        ang_dot = S @ w 
        w_dot = np.linalg.inv(self.J) @ np.cross(-w, (self.J @ w))

        xdot[:3] = p_dot
        xdot[3:6] = v_dot
        xdot[6:9] = ang_dot 
        xdot[9:] = w_dot 
        xdot += forces

        return xdot

    def calculateControl(self, xr, x0):
        self.x_ref = xr
        self.x0 = x0 

        optProb = pyoptsparse.Optimization('MPC', self)
        optProb.addVarGroup('xvars', 4 * self.T, type='c', value=self.u_eq*self.T, lower=self.u_min * self.T, upper=self.u_max * self.T ) #List multiplication seems the best way to go
        # optProb.addConGroup('x0', 12, lower=x0, upper=x0) #I don't know if this is needed. x0 is not a state that is changing #Initial state must be equal to x0
        optProb.addConGroup('rp', 2 * (self.T+1), lower=self.rp_min * (self.T+1), upper=self.rp_max * (self.T+1)) #Check list multiplication
        optProb.addObj('obj')

        opt = pyoptsparse.SNOPT()
        sol = opt(optProb, sens='CS')
        u = sol.xStar['xvars'][:4]
        return u