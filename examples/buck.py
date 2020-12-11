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
t_sim = 50e-3

# PWM period
t_pwm = 1/100e3

# Number of points per cycle
n_pwm = 2000

buck = pyd.pe.Buck(R, L, C)
buck.set_pwm(t_pwm, n_pwm)
buck.set_sim_time(t_sim)
buck.set_v_in(v_in)
buck.set_initial_conditions(0.5, 6)

n = buck.n_cycles
v_ref = 6 * np.ones(n)
#v_ref[:int(n/2)] = 6
#v_ref[int(n/2):] = 9

v_in = 12*np.ones(n)
v_in[:int(n/5)] = 12
v_in[int(n/5):] = 11

buck.sim(v_ref=v_ref, v_in=v_in)
t = buck.t
x = buck.x
