import pydsim as pyd
import numpy as np
import matplotlib.pyplot as plt
plt.ion()

# --- Input ---
# Circuit components
R = 12
L = 15e-6
C = 10e-6

# Input and reference voltage
v_in = 12
V_ref = 6

# Sim time
t_sim = 20e-3

# PWM period
t_pwm = 1/100e3

# Number of points per cycle
n_pwm = 100

buck = pyd.pe.Buck(R, L, C)
buck.set_pwm(t_pwm, n_pwm)
buck.set_sim_time(t_sim)
buck.set_v_in(v_in)
buck.set_initial_conditions(0.5, 6)

n = buck.n
v_ref = 6 * np.ones(n)
v_ref[:int(n/2)] = 6
v_ref[int(n/2):] = 9

v_in = 12 * np.ones((n, 1))
#v_in[:int(n/5)] = 12
#v_in[int(n/5):] = 20

j, j0, j1 = buck.mpc_sim(v_ref=v_ref, v_in=v_in)
t = buck.t
x = buck.x

jj, jj0, jj1 = buck.mpc_sim(v_ref=v_ref, v_in=v_in, beta=0.00025)
t1 = buck.t
x1 = buck.x

#buck.sim(v_ref=v_ref, v_in=v_in, control='pi')
#t1 = buck.t
#x1 = buck.x

plt.plot(t, x[:, 1])
plt.plot(t1, x1[:, 1])
#plt.plot(buck.t, buck.u)
#plt.plot(t1, x1[:, 1])
