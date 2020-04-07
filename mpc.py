import numpy as np 
import cvxpy as cp 
import params

class MPC:
    def __init__(self, A, B, u_max, u_min, T=10): #Note that Q is diagonal and should only have values for states that I care to track (position and maybe heading)
        self.A = A 
        self.B = B 
        self.u_max = u_max 
        self.u_min = u_min 
        self.Q = np.diag([10.0, 10.0, 2.0, 0, 0, 0, 10.0, 10.0, 0, 1.0, 1.0, 1.0]) #Weighting matrix for the states (x_ref - x_k)
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
            cost += cp.quad_form(xr - x[:,t+1], self.Q) #+ cp.quad_form(u[:,t], self.R) #TODO: Update for different xr's in trajectory
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
