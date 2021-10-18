import pydsim as pyd
import numpy as np
import matplotlib.pyplot as plt
plt.ion()

# --- Input ---
# Circuit components
R = 2.2
L = 47e-6
C = 560e-6

# Parasitics
Rds = 15e-3
Rl = 15e-3
Rc = 60e-3

# Input and reference voltage
v_in = 16
v_in_step = -3
v_ref = 8

# Sim time
t_sim = 5e-3

# PWM frequency
f_pwm = 200e3

# Step size for simulation
max_step = 1e-6
dt = 1 / f_pwm / 500

# Specs for state feedback
Ts = 1.25e-3
os = 2/100

# --- Simulation ---
buck = pyd.peode.Buck(R, L, C, Rds=Rds, Rl=Rl, Rc=Rc)
buck.set_f_pwm(f_pwm)
buck.set_sim_params(dt, t_sim, max_step=max_step)

n = round(t_sim * f_pwm)
v_in_p = v_in * np.ones(n)
#v_in_p[n>>1:] = v_in + v_in_step

zeta = -np.log(os) / np.sqrt(np.pi**2 + (np.log(os))**2)
wn = 4/Ts/zeta
p1 = -zeta * wn + wn * np.sqrt(zeta**2 - 1, dtype=complex)
p2 = np.conj(p1)
p3 = 10 * p1.real

sfb_params = {'poles': [p1, p2]}
buck.sim(v_ref=v_ref, v_in=v_in_p, ctl=pyd.control.SFB, ctl_params=sfb_params)

t_sfb = buck.signals.t
x_sfb = buck.signals.x
u_sfb = buck.signals.d

sfbi_params = {'poles': [p1, p2, p3]}
buck.sim(v_ref=v_ref, v_in=v_in_p, ctl=pyd.control.SFB_I, ctl_params=sfbi_params)
t_sfbi = buck.signals.t
x_sfbi = buck.signals.x
u_sfbi = buck.signals.d


# --- Results ---
plt.figure(figsize=(10,6))

ax = plt.subplot(3,1,1)
plt.plot(t_sfb / 1e-3, x_sfb[:, 1], label='sfb')
plt.plot(t_sfbi / 1e-3, x_sfbi[:, 1], label='sfb_i')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Voltage (V)')

plt.subplot(3,1,2, sharex=ax)
plt.plot(t_sfb / 1e-3, x_sfb[:, 0], label='sfb')
plt.plot(t_sfbi / 1e-3, x_sfbi[:, 0], label='sfb_i')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Current (A)')

plt.subplot(3,1,3, sharex=ax)
plt.plot(t_sfb / 1e-3, u_sfb, label='sfb')
plt.plot(t_sfbi / 1e-3, u_sfbi, label='sfb_i')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('$u$')

plt.tight_layout()
