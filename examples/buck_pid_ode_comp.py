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
t_sim = 3e-3

# PWM frequency
f_pwm = 200e3

# Step size for simulation
max_step = 1e-6
dt = 1 / f_pwm / 200

# --- Simulation ---
n = round(t_sim * f_pwm)
v_in_p = v_in * np.ones(n)
v_in_p[n>>1:] = v_in + v_in_step

# - ODE -
buck = pyd.peode.Buck(R, L, C)
buck.set_f_pwm(f_pwm)
buck.set_sim_params(dt, t_sim, max_step=max_step)

ctlparams = {'ki': 10000, 'kd': 0.0001, 'kp': 0.75, 'N': 50000}
buck.set_ctlparams(ctlparams)
buck.sim(v_ref=v_ref, v_in=v_in_p, controller=pyd.control.PID)
t_ode = buck.signals.t
x_ode = buck.signals.x
u_ode = buck.signals.d

# - Diff -
buck = pyd.pe.Buck(R, L, C)
buck.set_f_pwm(f_pwm)
buck.set_sim_params(dt, t_sim)

ctlparams = {'ki': 10000, 'kd': 0.0001, 'kp': 0.75, 'N': 50000}
buck.set_ctlparams(ctlparams)
buck.sim(v_ref=v_ref, v_in=v_in_p, controller=pyd.control.PID)
t_dif = buck.signals.t
x_dif = buck.signals.x
u_dif = buck.signals.d

# --- Results ---
plt.figure(figsize=(10,6))

ax = plt.subplot(3,1,1)
plt.plot(t_ode / 1e-3, x_ode[:, 1], label='ODE')
plt.plot(t_dif / 1e-3, x_dif[:, 1], label='Dif')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Voltage (V)')

plt.subplot(3,1,2, sharex=ax)
plt.plot(t_ode / 1e-3, x_ode[:, 0], label='ODE')
plt.plot(t_dif / 1e-3, x_dif[:, 0], label='Dif')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Current (A)')

plt.subplot(3,1,3, sharex=ax)
plt.plot(t_ode / 1e-3, u_ode, label='ODE')
plt.plot(t_dif / 1e-3, u_dif, label='Dif')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Control signal')

plt.tight_layout()

plt.figure(figsize=(10,6))

ax = plt.subplot(3,1,1)
plt.plot(t_ode / 1e-3, x_ode[:, 1] - x_dif[:, 1], label='ODE')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Voltage (V)')

plt.subplot(3,1,2, sharex=ax)
plt.plot(t_ode / 1e-3, x_ode[:, 0] - x_dif[:, 0], label='ODE')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Current (A)')

plt.subplot(3,1,3, sharex=ax)
plt.plot(t_ode / 1e-3, u_ode - u_dif, label='ODE')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Control signal')
