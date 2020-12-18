import pydsim as pyd
import numpy as np
import matplotlib.pyplot as plt
plt.ion()

# --- Input ---
# Circuit components
R = 5
L = 10e-6
C = 560e-6

# Input and reference voltage
v_in = 10
v_ref = 6

# Sim time
t_sim = 50e-3

# PWM period
t_pwm = 1/200e3

# Number of points per cycle
n_pwm = 200

buck = pyd.pe.Buck(R, L, C)
buck.set_pwm(t_pwm, n_pwm)
buck.set_sim_time(t_sim)
buck.set_v_in(v_in)
#buck.set_initial_conditions(0.5, 6)

buck.sim(v_ref=v_ref, v_in=v_in)
t = buck.t
x = buck.x

pi_params = {'ki': 250, 'kp': 0}
buck.set_ctlparams(pi_params)
buck.sim(v_ref=v_ref, v_in=v_in, control='pi')
t1 = buck.t
x1 = buck.x

plt.plot(t, x[:, 1])
plt.plot(t1, x1[:, 1])
