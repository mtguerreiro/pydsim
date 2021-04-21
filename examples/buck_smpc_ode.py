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
t_sim = 10e-3

# PWM frequency
f_pwm = 200e3

# Step size for simulation
max_step = 1e-6
dt = 1 / f_pwm / 200

# --- Simulation ---
buck = pyd.peode.Buck(R, L, C)
buck.set_f_pwm(f_pwm)
buck.set_sim_params(dt, t_sim, max_step=max_step)

t_pwm = 1 / f_pwm
n = round(t_sim * f_pwm)
v_in_p = v_in * np.ones(n)
v_in_p[int(n / 2):] = v_in + v_in_step

v_ref_p = v_ref * np.ones(n)
#v_ref_p[int(n / 2):] = v_ref + v_in_step
##t_i = int(5e-3 / t_pwm)
##t_f = int(6e-3 / t_pwm)
##v_ref_p[t_i:t_f] = np.arange(v_ref, v_ref + v_in_step, v_in_step / (t_f - t_i))
##v_ref_p[t_f:] = v_ref + v_in_step
##v_ref_p = 5 + np.sin(2 * np.pi * 2000 * t_pwm * np.arange(n))

pid_params = {'ki': 10000, 'kd': 0.0001, 'kp': 0.75, 'N': 50000}
buck.set_ctlparams(pid_params)
buck.sim(v_ref=v_ref_p, v_in=v_in_p, controller=pyd.control.PID)
t_pi = buck.signals.t
x_pi = buck.signals.x

mpc_params = {'alpha': 5, 'beta': 0, 'n_step': 3, 'il_max': 5, 'ref': v_ref_p}
buck.set_ctlparams(mpc_params)
buck.sim(v_ref=v_ref_p, v_in=v_in_p, controller=pyd.control.SMPC)
t_mp = buck.signals.t
x_mp = buck.signals.x

# --- Results ---
plt.figure(figsize=(10,6))

t_ref = t_pwm * np.arange(n) / 1e-3
ax = plt.subplot(2,1,1)
plt.plot(t_ref, v_ref_p, label='ref')
plt.plot(t_pi / 1e-3, x_pi[:, 1], label='pid')
plt.plot(t_mp / 1e-3, x_mp[:, 1], label='mpc')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Voltage (V)')

plt.subplot(2,1,2, sharex=ax)
plt.plot(t_pi / 1e-3, x_pi[:, 0], label='pid')
plt.plot(t_mp / 1e-3, x_mp[:, 0], label='mpc')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Current (A)')

plt.tight_layout()
