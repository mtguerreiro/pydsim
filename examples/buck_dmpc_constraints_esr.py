import pydsim as pyd
import numpy as np
import matplotlib.pyplot as plt
plt.ion()

# --- Input ---
# Circuit components
R = 1.1
L = 47e-6
C = 560e-6

Rl = 15e-3
Rds = 15e-3
Rc = 60e-3

# Input and reference voltage
v_in = 20
v_in_step = 3
v_ref = 4.7

# Sim time
t_sim = 1.5e-3

# PWM frequency
f_pwm = 200e3

# Step size for simulation
dt = 1 / f_pwm / 500

# --- Simulation ---
buck = pyd.peode.Buck(R, L, C, Rl=Rl, Rds=Rds, Rc=Rc)
buck.set_f_pwm(f_pwm)
buck.set_sim_params(dt, t_sim)
#buck.set_initial_conditions(2, 0.8)

t_pwm = 1 / f_pwm
n = round(t_sim * f_pwm)
v_in_p = v_in * np.ones(n)
v_in_p[int(n / 2):] = v_in + v_in_step

v_ref_p = v_ref * np.ones(n)

dmpc_params = {'n_c': 2, 'n_p': 20, 'r_w': 20, 'u_lim': [0, 1], 'il_lim': [-8, 8], 'n_ct':1, 'n_iter':3, 'solver':'hild'}
buck.sim(v_ref=v_ref_p, v_in=v_in_p, ctl=pyd.control.DMPC_C, ctl_params=dmpc_params)
t_dmpc_q = buck.signals.t
x_dmpc_q = buck.signals.x
u_dmpc_q = buck.signals.d

dmpc_params = {'n_c': 2, 'n_p': 20, 'r_w': 20, 'u_lim': [0, 1], 'il_lim': [-8, 8], 'n_ct':1, 'n_iter':3, 'solver':'hild'}
buck.sim(v_ref=v_ref_p, v_in=v_in_p, ctl=pyd.control.DMPC_C, ctl_params=dmpc_params)
t_dmpc_h = buck.signals.t
x_dmpc_h = buck.signals.x
u_dmpc_h = buck.signals.d
n_iters = buck.ctl.n_iters

# --- Results ---
plt.figure(figsize=(10,6))

t_ref = t_pwm * np.arange(n)

ax = plt.subplot(4,1,1)
plt.plot(t_dmpc_q / 1e-3, x_dmpc_q[:, 1], label='quadprog')
plt.plot(t_dmpc_h / 1e-3, x_dmpc_h[:, 1], label='hild')
plt.plot(t_ref / 1e-3, v_ref_p, label='ref')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Voltage (V)')

plt.subplot(4,1,2, sharex=ax)
plt.plot(t_dmpc_q / 1e-3, x_dmpc_q[:, 0], label='quadprog')
plt.plot(t_dmpc_h / 1e-3, x_dmpc_h[:, 0], label='hild')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Current (A)')

plt.subplot(4,1,3, sharex=ax)
plt.plot(t_dmpc_q / 1e-3, u_dmpc_q, label='quadprog')
plt.plot(t_dmpc_h / 1e-3, u_dmpc_h, label='hild')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('$u$')

plt.subplot(4,1,4, sharex=ax)
##plt.plot(t_dmpc / 1e-3, u_dmpc, label='dmpc')
plt.step(t_ref / 1e-3, n_iters, label='hild')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Iterations')

plt.tight_layout()
