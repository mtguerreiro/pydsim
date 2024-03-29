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
t_sim = 25e-3

# PWM frequency
f_pwm = 200e3

# Step size for simulation
dt = 1 / f_pwm / 500

# --- Simulation ---
buck = pyd.pe.Buck(R, L, C)
buck.set_f_pwm(f_pwm)
buck.set_sim_params(dt, t_sim)

olparams = {'u':v_ref / v_in}
buck.sim(v_ref=v_ref, v_in=v_in, ctl=pyd.control.OL, ctl_params=olparams)
t_ol = buck.signals.t
x_ol = buck.signals.x
u_ol = buck.signals.d

ctlparams = {'ki': 30, 'kp': 0}
buck.sim(v_ref=v_ref, v_in=v_in, ctl=pyd.control.PI, ctl_params=ctlparams)
t_pi = buck.signals.t
x_pi = buck.signals.x
u_pi = buck.signals.d

# --- Results ---
plt.figure(figsize=(10,6))

ax = plt.subplot(3,1,1)
plt.plot(t_ol / 1e-3, x_ol[:, 1], label='ol')
plt.plot(t_pi / 1e-3, x_pi[:, 1], label='pi')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Voltage (V)')

plt.subplot(3,1,2, sharex=ax)
plt.plot(t_ol / 1e-3, x_ol[:, 0], label='ol')
plt.plot(t_pi / 1e-3, x_pi[:, 0], label='pi')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Current (A)')

plt.subplot(3,1,3, sharex=ax)
plt.plot(t_ol / 1e-3, u_ol, label='ol')
plt.plot(t_pi / 1e-3, u_pi, label='pi')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('$u$')

plt.tight_layout()
