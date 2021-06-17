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
v_in_step = 3
v_ref = 5

# Sim time
t_sim = 1.5e-3

# PWM frequency
f_pwm = 200e3

# Step size for simulation
dt = 1 / f_pwm / 500

# --- Simulation ---
buck = pyd.peode.Buck(R, L, C)
buck.set_f_pwm(f_pwm)
buck.set_sim_params(dt, t_sim)
#buck.set_initial_conditions(2, 0.8)

t_pwm = 1 / f_pwm
n = round(t_sim * f_pwm)
v_in_p = v_in * np.ones(n)
#v_in_p[int(n / 2):] = v_in + v_in_step

v_ref_p = v_ref * np.ones(n)
##v_ref_p[int(n / 2):] = v_ref  + v_in_step
##t_i = int(4e-3 / t_pwm)
##t_f = int(4.5e-3 / t_pwm)
##v_ref_p[t_i:t_f] = np.arange(v_ref, v_ref + v_in_step, v_in_step / (t_f - t_i))
##v_ref_p[t_f:] = v_ref + v_in_step
##v_ref_p = 5 + np.sin(2 * np.pi * 500 * t_pwm * np.arange(n))

##pid_params = {'ki': 10000, 'kd': 0.0001, 'kp': 0.75, 'N': 50000}
##buck.set_ctlparams(pid_params)
##buck.sim(v_ref=v_ref_p, v_in=v_in_p, controller=pyd.control.PID)
##t_pi = buck.signals.t
##x_pi = buck.signals.x
##u_pi = buck.signals.d

#dmpc_params = {'n_c': 30, 'n_p': 30, 'r_w': 100}
#buck.set_ctlparams(dmpc_params)
#buck.sim(v_ref=v_ref_p, v_in=v_in_p, controller=pyd.control.DMPC)
##dmpc_params = {'n_c': 12, 'n_p': 20, 'r_w': 15, 'u_lim': [0, 1], 'il_lim': [-15, 15], 'n_ct':1}
##buck.set_ctlparams(dmpc_params)
##buck.sim(v_ref=v_ref_p, v_in=v_in_p, controller=pyd.control.DMPC_C)
##t_dmpc = buck.signals.t
##x_dmpc = buck.signals.x
##u_dmpc = buck.signals.d

#dmpc_params = {'n_c': 15, 'n_p': 25, 'r_w': 0.05, 'u_lim': [0, 1], 'il_lim': [-15, 15], 'n_ct':15}
dmpc_params = {'n_c': 15, 'n_p': 25, 'r_w': 0.05, 'u_lim': [0, 1], 'il_lim': [-15, 15], 'n_ct':1, 'solver':'quadprog'}
buck.set_ctlparams(dmpc_params)
buck.sim(v_ref=v_ref_p, v_in=v_in_p, controller=pyd.control.DMPC_C)
t_dmpc = buck.signals.t
x_dmpc = buck.signals.x
u_dmpc = buck.signals.d
#n_iters = buck.ctl.n_iters

#dmpc_params = {'n_c': 15, 'n_p': 25, 'r_w': 0.05, 'u_lim': [0, 1], 'il_lim': [-15, 15], 'n_ct':15}
dmpc_params = {'n_c': 15, 'n_p': 25, 'r_w': 0.05, 'u_lim': [0, 1], 'il_lim': [-15, 15], 'n_ct':1, 'n_iter':200, 'solver':'hild'}
buck.set_ctlparams(dmpc_params)
buck.sim(v_ref=v_ref_p, v_in=v_in_p, controller=pyd.control.DMPC_C)
t_dmpc_c = buck.signals.t
x_dmpc_c = buck.signals.x
u_dmpc_c = buck.signals.d
n_iters = buck.ctl.n_iters

# --- Results ---
plt.figure(figsize=(10,6))

t_ref = t_pwm * np.arange(n)

ax = plt.subplot(4,1,1)
plt.plot(t_dmpc / 1e-3, x_dmpc[:, 1], label='dmpc')
plt.plot(t_dmpc_c / 1e-3, x_dmpc_c[:, 1], label='dmpc-c')
plt.plot(t_ref / 1e-3, v_ref_p, label='ref')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Voltage (V)')

plt.subplot(4,1,2, sharex=ax)
plt.plot(t_dmpc / 1e-3, x_dmpc[:, 0], label='dmpc')
plt.plot(t_dmpc_c / 1e-3, x_dmpc_c[:, 0], label='dmpc-c')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Current (A)')

plt.subplot(4,1,3, sharex=ax)
plt.plot(t_dmpc / 1e-3, u_dmpc, label='dmpc')
plt.plot(t_dmpc_c / 1e-3, u_dmpc_c, label='dmpc-c')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('$u$')

plt.subplot(4,1,4, sharex=ax)
##plt.plot(t_dmpc / 1e-3, u_dmpc, label='dmpc')
plt.step(t_ref / 1e-3, n_iters, label='dmpc-c')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('$u$')

plt.tight_layout()
