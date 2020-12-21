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
v_in_step = -1
v_ref = 5

# Sim time
t_sim = 50e-3

# PWM period
t_pwm = 1/200e3

# Number of points per cycle
n_pwm = 200

# --- Simulation ---
buck = pyd.peode.Buck(R, L, C)
buck.set_pwm(t_pwm, n_pwm)
buck.set_sim_time(t_sim)

v_in_p = v_in * np.ones(buck.n_cycles)
v_in_p[int(buck.n_cycles / 2):] = v_in + v_in_step

ctlparams = {'ki': 200, 'kp': 0}
buck.set_ctlparams(ctlparams)
buck.sim(v_ref=v_ref, v_in=v_in_p, control='pi')
t_pi = buck.t
x_pi = buck.x

mpc_params = {'alpha': 5, 'beta': 0, 'n_step':3}
buck.set_ctlparams(mpc_params)
buck.sim(v_ref=v_ref, v_in=v_in_p, control='mpc')
t_mp = buck.t
x_mp = buck.x

# --- Results ---
plt.figure(figsize=(10,6))

ax = plt.subplot(2,1,1)
plt.plot(t_pi / 1e-3, x_pi[:, 1], label='pi')
plt.plot(t_mp / 1e-3, x_mp[:, 1], label='mpc')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Voltage (V)')

plt.subplot(2,1,2, sharex=ax)
plt.plot(t_pi / 1e-3, x_pi[:, 0], label='pi')
plt.plot(t_mp / 1e-3, x_mp[:, 0], label='mpc')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Current (A)')

plt.tight_layout()
