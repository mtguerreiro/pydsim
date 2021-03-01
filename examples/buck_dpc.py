import pydsim as pyd
import numpy as np
import matplotlib.pyplot as plt
plt.ion()

# --- Input ---
# Circuit components
##R = 10
##L = 100e-6
##C = 10e-6
R = 5
L = 10e-6
C = 560e-6

# Input and reference voltage
v_in = 10
#v_in_step = -1
v_ref = 5

# Sim time
t_sim = 20e-3

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

##ctlparams = {'ki': 200, 'kp': 0}
##buck.set_ctlparams(ctlparams)
##buck.sim(v_ref=v_ref, v_in=v_in_p, control='pi')
##t_pi = buck.t
##x_pi = buck.x

dmpc_params = {'n_c': 30, 'n_p': 30, 'r_w': 3000}
buck.set_ctlparams(dmpc_params)
buck.sim(v_ref=v_ref, v_in=v_in_p, control='dmpc')
t_dmpc = buck.t
x_dmpc = buck.x
u_dmpc = buck.u

buck.sim(v_ref=v_ref, v_in=v_in_p, control='ol')
t_ol = buck.t
x_ol = buck.x
u_ol = buck.u

# --- Results ---
plt.figure(figsize=(10,6))

ax = plt.subplot(2,1,1)
#plt.plot(t_pi / 1e-3, x_pi[:, 1], label='pi')
plt.plot(t_ol / 1e-3, x_ol[:, 1], label='open-loop')
plt.plot(t_dmpc / 1e-3, x_dmpc[:, 1], label='dmpc')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Voltage (V)')

plt.subplot(2,1,2, sharex=ax)
plt.plot(t_ol / 1e-3, u_ol, label='open-loop')
plt.plot(t_dmpc / 1e-3, u_dmpc, label='dmpc')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('$u$')

plt.tight_layout()
