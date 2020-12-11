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
v_in = 10
V_ref = 5

# Sim time
t_sim = 10e-3

# PWM period
t_pwm = 1/200e3

# Number of points per cycle
n_pwm = 1000

buck = pyd.pe.Buck(R, L, C)
buck.set_pwm(t_pwm, n_pwm)
buck.set_sim_time(t_sim)
buck.set_v_in(v_in)
#buck.set_initial_conditions(0.5, 6)

n = buck.n_cycles
v_ref = 5 * np.ones(n)
v_ref[:int(n/2)] = 5
v_ref[int(n/2):] = 2

v_in = 10 * np.ones(n)
#v_in[:int(n/5)] = 12
#v_in[int(n/2):] = 20

buck.sim(v_ref=v_ref, v_in=v_in, control='pi')
t = buck.t
x = buck.x
u = buck.u
pwm = buck.pwm

buck.sim(v_ref=v_ref, v_in=v_in, control='mpc')
t1 = buck.t
x1 = buck.x
u1 = buck.u
pwm1 = buck.pwm

plt.figure()
sp1 = plt.subplot(2,1,1)
plt.plot(t, x[:, 1])
plt.plot(t1, x1[:, 1])

plt.subplot(2,1,2, sharex=sp1)
plt.plot(t, pwm)
plt.plot(t1, pwm1)
