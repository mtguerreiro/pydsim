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

# PWM period
t_pwm = 1/200e3

# Number of points per cycle
n_pwm = 200

# --- Simulation ---
buck = pyd.pe.Buck(R, L, C)
buck.set_pwm(t_pwm, n_pwm)
buck.set_sim_time(t_sim)

v_in_p = v_in * np.ones(buck.n_cycles)
v_in_p[int(buck.n_cycles / 2):] = v_in + v_in_step

v_ref_p = v_ref * np.ones(buck.n_cycles)
#v_ref_p[int(buck.n_cycles / 2):] = v_ref + v_in_step
##t_i = int(5e-3 / t_pwm)
##t_f = int(6e-3 / t_pwm)
##v_ref_p[t_i:t_f] = np.arange(v_ref, v_ref + v_in_step, v_in_step / (t_f - t_i))
##v_ref_p[t_f:] = v_ref + v_in_step
##v_ref_p = 5 + np.sin(2 * np.pi * 2000 * t_pwm * np.arange(buck.n_cycles))

pid_params = {'ki': 10000, 'kd': 0.0001, 'kp': 0.75, 'N': 50000, 'sat': True}
buck.set_ctlparams(pid_params)
buck.sim(v_ref=v_ref_p, v_in=v_in_p, control='pid')
t_pi = buck.t
x_pi = buck.x

mpc_params = {'alpha': 5, 'beta': 0, 'n_step': 3, 'il_max': 5, 'ref': v_ref_p}
buck.set_ctlparams(mpc_params)
buck.sim(v_ref=v_ref_p, v_in=v_in_p, control='mpc')
t_mp = buck.t
x_mp = buck.x

# --- Results ---
plt.figure(figsize=(10,6))

t_ref = t_pwm * np.arange(buck.n_cycles) / 1e-3
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
