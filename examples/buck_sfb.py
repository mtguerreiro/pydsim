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
t_sim = 5e-3

# PWM frequency
f_pwm = 200e3

# Step size for simulation
dt = 1 / f_pwm / 500

# Specs for state feedback
Ts = 0.5e-3
os = 5/100

# --- Simulation ---
buck = pyd.pe.Buck(R, L, C)
buck.set_f_pwm(f_pwm)
buck.set_sim_params(dt, t_sim)

n = round(t_sim * f_pwm)
v_in_p = v_in * np.ones(n)
v_in_p[n>>1:] = v_in + v_in_step

zeta = -np.log(os) / np.sqrt(np.pi**2 + (np.log(os))**2)
wn = 4/Ts/zeta
p1 = -zeta * wn + wn * np.sqrt(zeta**2 - 1, dtype=complex)
p2 = np.conj(p1)
p3 = 10 * p1.real

sfb_params = {'p1':p1, 'p2':p2, 'p3':p3}
buck.set_ctlparams(sfb_params)
buck.sim(v_ref=v_ref, v_in=v_in_p, control='sfb')

t_sfb = buck.signals.t
x_sfb = buck.signals.x
u_sfb = buck.signals.d

##buck.set_f_pwm(f_pwm / 2)
##buck.sim(v_ref=v_ref, v_in=v_in, control='sfb')
##t_sfb1 = buck.signals.t
##x_sfb1 = buck.signals.x
##u_sfb1 = buck.signals.d


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

#buck.set_pwm(t_pwm, n_pwm)
#buck.set_sim_time(t_sim)
#buck.set_initial_conditions(1, 2.5)
#buck.set_filter(1 / t_pwm / 2)

##import pydsim as pyd
##import numpy as np
##import matplotlib.pyplot as plt
##plt.ion()
##
### --- Input ---
### Circuit components
##R = 5
##L = 10e-6
##C = 560e-6
##
### Input and reference voltage
##v_in = 10
##v_in_step = -3
##v_ref = 5
##
### Sim time
##t_sim = 8e-3
##
### PWM period
##t_pwm = 1/200e3
##
### Number of points per cycle
##n_pwm = 500
##
### Specs for state feedback
##Ts = 0.5e-3
##os = 5/100
##
### --- Simulation ---
##buck = pyd.pe.Buck(R, L, C)
##buck.set_pwm(t_pwm, n_pwm)
##buck.set_sim_time(t_sim)
###buck.set_initial_conditions(1, 2.5)
###buck.set_filter(1 / t_pwm / 2)
##
##v_in_p = v_in * np.ones(buck.n_cycles)
##v_in_p[int(buck.n_cycles / 2):] = v_in + v_in_step
##
##v_ref_p = v_ref * np.ones(buck.n_cycles)
###v_ref_p[int(buck.n_cycles / 2):] = v_ref  + v_in_step
####t_i = int(4e-3 / t_pwm)
####t_f = int(4.5e-3 / t_pwm)
####v_ref_p[t_i:t_f] = np.arange(v_ref, v_ref + v_in_step, v_in_step / (t_f - t_i))
####v_ref_p[t_f:] = v_ref + v_in_step
#####v_ref_p = 5 + np.sin(2 * np.pi * 500 * t_pwm * np.arange(buck.n_cycles))
##
##pid_params = {'ki': 10000, 'kd': 0.0001, 'kp': 0.75, 'N': 50000, 'sat': True}
##buck.set_ctlparams(pid_params)
##buck.sim(v_ref=v_ref_p, v_in=v_in_p, control='pid')
##t_pi = buck.t
##x_pi = buck.x
##u_pi = buck.u
##
### Computes poles of compensated system
##zeta = -np.log(os) / np.sqrt(np.pi**2 + (np.log(os))**2)
##wn = 4/Ts/zeta
##p1 = -zeta * wn + wn * np.sqrt(zeta**2 - 1, dtype=complex)
##p2 = np.conj(p1)
##p3 = 10 * p1.real
##
##sfb_params = {'p1':p1, 'p2':p2, 'p3':p3}
###sfb_params = {'p1': -10000+1j*5500, 'p2': -10000-1j*5500, 'p3': -15000}
##buck.set_ctlparams(sfb_params)
##buck.sim(v_ref=v_ref_p, v_in=v_in_p, control='sfb')
##t_sfb = buck.t
##x_sfb = buck.x
##u_sfb = buck.u
##
##### --- Results ---
##plt.figure(figsize=(10,6))
##
##t_ref = t_pwm * np.arange(buck.n_cycles)
##
##ax = plt.subplot(3,1,1)
##plt.plot(t_pi / 1e-3, x_pi[:, 1], label='pid')
##plt.plot(t_sfb / 1e-3, x_sfb[:, 1], label='sfb')
##plt.plot(t_ref / 1e-3, v_ref_p, label='ref')
##plt.grid()
##plt.legend()
##plt.xlabel('Time (ms)')
##plt.ylabel('Voltage (V)')
##
##plt.subplot(3,1,2, sharex=ax)
##plt.plot(t_pi / 1e-3, x_pi[:, 0], label='pid')
##plt.plot(t_sfb / 1e-3, x_sfb[:, 0], label='sfb')
##plt.grid()
##plt.legend()
##plt.xlabel('Time (ms)')
##plt.ylabel('Current (A)')
##
##plt.subplot(3,1,3, sharex=ax)
##plt.plot(t_pi / 1e-3, u_pi, label='pid')
##plt.plot(t_sfb / 1e-3, u_sfb, label='sfb')
##plt.grid()
##plt.legend()
##plt.xlabel('Time (ms)')
##plt.ylabel('$u$')
##
##plt.tight_layout()
