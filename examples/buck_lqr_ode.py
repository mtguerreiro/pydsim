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
t_sim = 1e-3

# PWM frequency
f_pwm = 200e3

# Step size for simulation
max_step = 1e-6
dt = 1 / f_pwm / 50

# Specs for state feedback
Qw = np.array([[10, 0, 0], [0, 1, 0], [0, 0, 50e9]])
Rw = np.array([[0.5]])
Hw = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
N = 200

# --- Simulation ---
buck = pyd.peode.Buck(R, L, C)
buck.set_f_pwm(f_pwm)
buck.set_sim_params(dt, t_sim, max_step=max_step)

n = round(t_sim * f_pwm)
v_in_p = v_in * np.ones(n)
#v_in_p[n>>1:] = v_in + v_in_step

ctlparams = {'Q': Qw, 'R': Rw, 'H': Hw, 'N': N}
buck.set_ctlparams(ctlparams)
buck.sim(v_ref=v_ref, v_in=v_in_p, controller=pyd.control.LQR)

t_sfb = buck.signals.t
x_sfb = buck.signals.x
u_sfb = buck.signals.d

# --- Results ---
plt.figure(figsize=(10,6))

ax = plt.subplot(3,1,1)
plt.plot(t_sfb / 1e-3, x_sfb[:, 1], label='sfb')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Voltage (V)')

plt.subplot(3,1,2, sharex=ax)
plt.plot(t_sfb / 1e-3, x_sfb[:, 0], label='sfb')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Current (A)')

plt.subplot(3,1,3, sharex=ax)
plt.plot(t_sfb / 1e-3, u_sfb, label='sfb')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('$u$')

plt.tight_layout()
