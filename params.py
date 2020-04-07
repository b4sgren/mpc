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

mc = 2.0
mp = 0.25
r = .1
d = .3
mass = mc + 4 * mp 
Jx = 2.0/5 * mc * r**2 + 2 * d**2 * mp 
Jy = 2.0/5 * mc * r**2 + 2 * d**2 * mp 
Jz = 2.0/5 * mc * r**2 + 4 * d**2 * mp 

u_max = np.array([9.81 * mass, 0.1, 0.1, 0.1])
u_min = np.array([-9.81 * mass, -0.1, -0.1, -0.1])

dt = 0.02
t0 = 0.0
tf = 10.0
t_plot = 0.02