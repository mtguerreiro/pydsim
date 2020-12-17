import pydsim as pyd
import numpy as np
import matplotlib.pyplot as plt
import pysp
plt.ion()

# --- Input ---
# Circuit components
R = 1.2
L = 10e-6
C = 560e-6

# Input and reference voltage
v_in = 24
V_ref = 12

# Sim time
t_sim = 30e-3

# PWM period
t_pwm = 1/200e3

# Number of points per cycle
n_pwm = 500

buck = pyd.pe.Buck(R, L, C)
buck.set_pwm(t_pwm, n_pwm)
buck.set_sim_time(t_sim)
buck.set_v_in(v_in)
#buck.set_filter(10e3)
#buck.set_initial_conditions(0.5, 6)

n = buck.n_cycles
v_ref = 12 * np.ones(n)
v_ref[int(n/2):] = 19

v_in = 24 * np.ones(n)
#v_in[:int(n/5)] = 12
#v_in[int(n/2):] = 20

buck.sim(v_ref=v_ref, v_in=v_in, control='pi')
t = buck.t
x = buck.x
u = buck.u
pwm = buck.pwm

buck.sim(v_ref=v_ref, v_in=v_in, control='mpc', n_step=3)
t1 = buck.t
x1 = buck.x
u1 = buck.u
pwm1 = buck.pwm

plt.plot(t, x[:, 1])
plt.plot(t1, x1[:, 1])
