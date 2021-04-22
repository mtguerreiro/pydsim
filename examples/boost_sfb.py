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
v_in_step = 1
v_ref = 15

# Sim time
t_sim = 5e-3

# PWM frequency
f_pwm = 200e3

# Step size for simulation
dt = 1 / f_pwm / 500

# Specs for state feedback
Ts = 1.5e-3
os = 10/100

# --- Simulation ---
buck = pyd.peode.Boost(R, L, C)
buck.set_f_pwm(f_pwm)
buck.set_sim_params(dt, t_sim)
buck.set_initial_conditions(2, 11)

n = round(t_sim * f_pwm)
v_in_p = v_in * np.ones(n)
v_in_p[n>>1:] = v_in + v_in_step

zeta = -np.log(os) / np.sqrt(np.pi**2 + (np.log(os))**2)
wn = 4/Ts/zeta
p1 = -zeta * wn + wn * np.sqrt(zeta**2 - 1, dtype=complex)
p2 = np.conj(p1)
p3 = 10 * p1.real

sfb_params = {'poles': [p1, p2, p3]}
buck.set_ctlparams(sfb_params)
buck.sim(v_ref=v_ref, v_in=v_in_p, controller=pyd.control.LinSFB)

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
