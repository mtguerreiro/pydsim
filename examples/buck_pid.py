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
v_in_step = -2
v_ref = 5

# Sim time
t_sim = 6e-3

# PWM period
t_pwm = 1/200e3

# Number of points per cycle
n_pwm = 200

# --- Simulation ---
buck = pyd.pe.Buck(R, L, C)
buck.set_pwm(t_pwm, n_pwm)
buck.set_sim_time(t_sim)

v_in_p = v_in * np.ones(buck.n_cycles)
#v_in_p[int(buck.n_cycles / 2):] = v_in + v_in_step

v_ref_p = v_ref * np.ones(buck.n_cycles)
v_ref_p[int(buck.n_cycles / 2):] = v_ref + v_in_step

##buck.sim(v_ref=v_ref, v_in=v_in_p, control='ol')
##t_ol = buck.t
##x_ol = buck.x

ctlparams = {'ki': 10000, 'kd': 0.0001, 'kp': 0.75, 'N': 50000, 'sat': True}
buck.set_ctlparams(ctlparams)
buck.sim(v_ref=v_ref_p, v_in=v_in_p, control='pid')
t_pi = buck.t
x_pi = buck.x
u_pi = buck.u

# --- Results ---
plt.figure(figsize=(10,6))

ax = plt.subplot(3,1,1)
##plt.plot(t_ol / 1e-3, x_ol[:, 1], label='ol')
plt.plot(t_pi / 1e-3, x_pi[:, 1], label='pi')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Voltage (V)')

plt.subplot(3,1,2, sharex=ax)
##plt.plot(t_ol / 1e-3, x_ol[:, 0], label='ol')
plt.plot(t_pi / 1e-3, x_pi[:, 0], label='pi')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Current (A)')

plt.subplot(3,1,3, sharex=ax)
##plt.plot(t_ol / 1e-3, x_ol[:, 0], label='ol')
plt.plot(t_pi / 1e-3, u_pi[:, 0], label='pi')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Control signal')

plt.tight_layout()
