import pydsim as pyd
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
plt.ion()

# --- Input ---
# Circuit components
R = 2.2
L = 47e-6
C = 560e-6

# Input and reference voltage
v_in = 10
v_in_step = -2
v_ref = 5

# Sim time
t_sim = 1.5e-3

# PWM frequency
f_pwm = 200e3

# Step size for simulation
max_step = 1e-6
dt = 1 / f_pwm / 500

# --- Simulation ---
buck = pyd.peode.Buck(R, L, C)
buck.set_f_pwm(f_pwm)
buck.set_sim_params(dt, t_sim, max_step)

n = round(t_sim * f_pwm)
v_in_p = v_in * np.ones(n)
#v_in_p[n>>1:] = v_in + v_in_step

##v_ref_p = v_ref * np.ones(buck.n_cycles)
##v_ref_p[int(buck.n_cycles / 2):] = v_ref + v_in_step

ctlparams = {'ki': 10000, 'kd': 0.0001, 'kp': 0.75, 'N': 50000}
buck.sim(v_ref=v_ref, v_in=v_in_p, ctl=pyd.control.PID, ctl_params=ctlparams)
t_pi = buck.signals.t
x_pi = buck.signals.x
u_pi = buck.signals.d

# --- Results ---
plt.style.use('seaborn-colorblind')
matplotlib.rcParams['mathtext.fontset'] = 'stix'
matplotlib.rcParams['font.family'] = 'Latin Modern Sans'
#matplotlib.rcParams.update({'font.size': 13})
matplotlib.rcParams.update({'font.size': 12})
plt.rc('axes', unicode_minus=False)
plt.ion()
l_fs = 11
title_fs = 12.5
#figsize = (5.5,3)
#figsize = (5.5,2.3)

plt.figure(figsize=(8,6))

ax = plt.subplot(3,1,1)
##plt.plot(t_ol / 1e-3, x_ol[:, 1], label='ol')
plt.plot(t_pi / 1e-3, x_pi[:, 1], label='PID')
plt.grid()
plt.legend(fontsize=l_fs, borderaxespad=0.15)
plt.xlabel('Time (ms)')
plt.ylabel('Voltage (V)')

plt.subplot(3,1,2, sharex=ax)
##plt.plot(t_ol / 1e-3, x_ol[:, 0], label='ol')
plt.plot(t_pi / 1e-3, x_pi[:, 0], label='PID')
plt.grid()
plt.legend(fontsize=l_fs, borderaxespad=0.15)
plt.xlabel('Time (ms)')
plt.ylabel('Current (A)')

plt.subplot(3,1,3, sharex=ax)
##plt.plot(t_ol / 1e-3, x_ol[:, 0], label='ol')
plt.plot(t_pi / 1e-3, u_pi, label='PID')
plt.grid()
plt.legend(fontsize=l_fs, borderaxespad=0.15)
plt.xlabel('Time (ms)')
plt.ylabel('Control signal')

plt.tight_layout()
