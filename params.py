import numpy as np 

pn0 = 0.0
pe0 = 0.0
pd0 = -10.0
vx0 = 0.0
vy0 = 0.0
vz0 = 0.0
e0 = 1.0 # Do I want to use euler angles or Rotation matrix?
e1 = 0.0
e2 = 0.0
e3 = 0.0
p0 = 0.0
q0 = 0.0
r0 = 0.0
phi0 = 0.0
theta0 = 0.0
psi0 = 0.0

Jx = 0.0224
Jy = 0.0224
Jz = 0.0436
mass = 0.8

u_max = np.array([2 * 9.81 * mass, 0.1, 0.1, 0.1])
u_min = np.array([0.0, -0.1, -0.1, -0.1])

dt = 0.02
t0 = 0.0
tf = 20.0
t_plot = 0.2