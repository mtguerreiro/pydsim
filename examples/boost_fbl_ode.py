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
v_in = 5
v_ref = 10
v_step = 3

# Sim time
t_sim = 0.5e-3

# PWM frequency
f_pwm = 200e3

# Step size for simulation
max_step = 1e-6
dt = 1 / f_pwm / 100

# --- Simulation ---
n = round(t_sim * f_pwm)
v_in_p = v_in * np.ones(n)
#v_in_p[n>>1:] = v_in + v_step

v_ref_p = v_ref * np.ones(n)
#v_ref_p[n>>1:] = v_ref + v_step

buck = pyd.peode.Boost(R, L, C)
buck.set_f_pwm(f_pwm)
buck.set_sim_params(dt, t_sim, max_step=max_step)
buck.set_initial_conditions(0, 0.0001)

ctlparams = {'ki': 0.6e5}
buck.set_ctlparams(ctlparams)
buck.sim(v_ref=v_ref_p, v_in=v_in_p, controller=pyd.control.FBL)
t = buck.signals.t
x = buck.signals.x
u = buck.signals.d

# --- Results ---
plt.figure(figsize=(10,6))

ax = plt.subplot(3,1,1)
plt.plot(t / 1e-3, x[:, 1])
plt.grid()
plt.xlabel('Time (ms)')
plt.ylabel('Voltage (V)')

plt.subplot(3,1,2, sharex=ax)
plt.plot(t / 1e-3, x[:, 0])
plt.grid()
plt.xlabel('Time (ms)')
plt.ylabel('Current (A)')


plt.subplot(3,1,3, sharex=ax)
plt.plot(t / 1e-3, u)
plt.grid()
plt.xlabel('Time (ms)')
plt.ylabel('$u$')

plt.tight_layout()
