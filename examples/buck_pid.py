import pydsim as pyd
import numpy as np
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
dt = 1 / f_pwm / 500

# --- Simulation ---
buck = pyd.pe.Buck(R, L, C)
buck.set_f_pwm(f_pwm)
buck.set_sim_params(dt, t_sim)

n = round(t_sim * f_pwm)
v_in_p = v_in * np.ones(n)
#v_in_p[n>>1:] = v_in + v_in_step

##v_ref_p = v_ref * np.ones(buck.n_cycles)
##v_ref_p[int(buck.n_cycles / 2):] = v_ref + v_in_step

##buck.sim(v_ref=v_ref, v_in=v_in_p, control='ol')
##t_ol = buck.t
##x_ol = buck.x

ctlparams = {'ki': 10000, 'kd': 0.0001, 'kp': 0.75, 'N': 50000}
buck.sim(v_ref=v_ref, v_in=v_in_p, ctl=pyd.control.PID, ctl_params=ctlparams)
t_pi = buck.signals.t
x_pi = buck.signals.x
u_pi = buck.signals.d

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
plt.plot(t_pi / 1e-3, u_pi, label='pi')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Control signal')

plt.tight_layout()
