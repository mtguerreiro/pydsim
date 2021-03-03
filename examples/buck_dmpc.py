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
v_in_step = -3
v_ref = 5

# Sim time
t_sim = 8e-3

# PWM period
t_pwm = 1/200e3

# Number of points per cycle
n_pwm = 500

# --- Simulation ---
buck = pyd.pe.Buck(R, L, C)
buck.set_pwm(t_pwm, n_pwm)
buck.set_sim_time(t_sim)
#buck.set_initial_conditions(1, 5)
#buck.set_filter(1 / t_pwm / 2)

v_in_p = v_in * np.ones(buck.n_cycles)
v_in_p[int(buck.n_cycles / 2):] = v_in + v_in_step

pid_params = {'ki': 25000, 'kd': 0.001, 'kp': 2.5, 'N': 20000, 'sat': True}
buck.set_ctlparams(pid_params)
buck.sim(v_ref=v_ref, v_in=v_in_p, control='pid')
t_pi = buck.t
x_pi = buck.x
u_pi = buck.u

dmpc_params = {'n_c': 30, 'n_p': 30, 'r_w': 500 / 10 ** 2}
buck.set_ctlparams(dmpc_params)
buck.sim(v_ref=v_ref, v_in=v_in_p, control='dmpc')
t_dmpc = buck.t
x_dmpc = buck.x
u_dmpc = buck.u

# --- Results ---
plt.figure(figsize=(10,6))

ax = plt.subplot(3,1,1)
plt.plot(t_pi / 1e-3, x_pi[:, 1], label='pi')
plt.plot(t_dmpc / 1e-3, x_dmpc[:, 1], label='dmpc')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Voltage (V)')

plt.subplot(3,1,2, sharex=ax)
plt.plot(t_pi / 1e-3, x_pi[:, 0], label='pi')
plt.plot(t_dmpc / 1e-3, x_dmpc[:, 0], label='dmpc')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Current (A)')

plt.subplot(3,1,3, sharex=ax)
plt.plot(t_pi / 1e-3, u_pi, label='pi')
plt.plot(t_dmpc / 1e-3, u_dmpc, label='dmpc')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('$u$')

plt.tight_layout()
