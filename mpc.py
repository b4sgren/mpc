import numpy as np 
import cvxpy as cp 
import params
import pyoptsparse #Will use SNOPT for the non-linear MPC

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
        self.Q = np.diag([10.0, 10.0, 2.0, 0, 0, 0, 10.0, 10.0, 50.0, 1.0, 1.0, 1.0]) #Weighting matrix for the states (x_ref - x_k)
        self.R = np.diag([1e-1, 0.1, 0.1, 0.1])   #Weighting matrix for the inputs (penalize high control effort)
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
            cost += cp.quad_form(xr - x[:,t+1], self.Q) #+ cp.quad_form(u[:,t], self.R) 
            constr += [x[:,t+1] == self.A @ x[:,t] + self.B @ u[:,t]] 
            constr += [x[6:8, t+1] <= phi_theta_max, x[6:8, t+1] >= -phi_theta_max] #Max constraint on roll and pitch angles
            constr += [u[:,t] <= self.u_max,  u[:,t] >= self.u_min] #Constraint on the inputs
        constr += [x[:,0] == x0] # First state must be what is passed in

        problem = cp.Problem(cp.Minimize(cost), constr)
        # problem.solve() #Will need to check if the soln is optimal
        problem.solve(solver=cp.ECOS)

        if problem.status == cp.OPTIMAL:
            print(u.value[:,0])
            return u.value[:,0]
        else:
            print("Not optimal")
            return np.array([params.mass * 9.81, 0.0, 0.0, 0.0]) #Equilbrium

class NMPC:
    def __init__(self, u_max, u_min, T=10):
        self.T = T 
        self.u_max = u_max 
        self.u_min = u_min 
        self.Q = np.diag([1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.])
        self.R = np.diag([0.1, 0.1, 0.1, 0.1])
        self.u_eq = np.array([params.mass * 9.81, 0.0, 0.0, 0.0]) #This will always be my starting guess
    
    def __call__(xdict):
        debug = 1
    
    def equalityConstraints(self):
        debug = 1
    
    def inequalityConstraints(self):
        debug = 1

    def calculateControl(self, xr, x0):
        self.x_ref = xr
        debug = 1