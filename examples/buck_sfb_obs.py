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
v_in_step = 3
v_ref = 5

# Sim time
t_sim = 10e-3

# PWM frequency
f_pwm = 200e3

# Step size for simulation
dt = 1 / f_pwm / 100

# Specs for state feedback
Ts = 1.5e-3
os = 1/100

# --- Simulation ---
buck = pyd.peode.Buck(R, L, C)
buck.set_f_pwm(f_pwm)
buck.set_sim_params(dt, t_sim)
#buck.set_initial_conditions(1, 1)

n = round(t_sim * f_pwm)
v_in_p = v_in * np.ones(n)
v_in_p[n>>1:] = v_in + v_in_step

v_ref_p = v_ref * np.ones(n)
#v_ref_p[n>>1:] = v_ref + v_in_step

# Plant poles
zeta = -np.log(os) / np.sqrt(np.pi**2 + (np.log(os))**2)
wn = 4/Ts/zeta
p1 = -zeta * wn + wn * np.sqrt(zeta**2 - 1, dtype=complex)
p2 = np.conj(p1)
p3 = 10 * p1.real

# Observer poles
##p1_o = 30 * p1
##p2_o = np.conj(p1_o)
##p3_o = 10 * p1_o.real
p1_o = 10 * p1
p2_o = np.conj(p1_o)
p3_o = 5 * p1_o.real

# Sim
sfb_params = {'poles': [p1, p2, p3], 'poles_o': [p1_o, p2_o, p3_o]}
buck.set_ctlparams(sfb_params)
buck.sim(v_ref=v_ref_p, v_in=v_in_p, controller=pyd.control.SFB_OBS)

t_sfb = buck.signals.t
x_sfb = buck.signals.x
u_sfb = buck.signals.d


# --- Results ---
x_obs = buck.ctl.get_sobs()
t_obs = buck.signals.t_p

plt.plot(t_sfb / 1e-3, x_sfb, label='sfb')
plt.plot(t_obs / 1e-3, x_obs, '--')
