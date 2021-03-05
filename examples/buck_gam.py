import pydsim as pyd
import numpy as np
import scipy
import matplotlib.pyplot as plt
plt.ion()

# --- Input ---
# Circuit components
R = 5
L = 100e-6
C = 10e-6

# Input and reference voltage
v_in = 10
v_ref = 8

# Sim time
t_sim = 1e-3

# PWM period
t_pwm = 1/200e3

# Number of points per cycle
n_pwm = 200

# --- Simulation ---
# GAM
buck = pyd.pegam.Buck(R, L, C)
buck.set_pwm(t_pwm, n_pwm)
buck.set_sim_time(t_sim)
buck.set_v_in(10)
buck.set_harmonics([1, 3, 5])
buck.set_max_step(t_pwm / 20)
#buck.set_initial_conditions(0.5, 6)

buck.sim(v_ref=v_ref, v_in=v_in)
t_gam = buck.t
x_gam = buck.x

# ODE
buck = pyd.peode.Buck(R, L, C)
buck.set_pwm(t_pwm, n_pwm)
buck.set_sim_time(t_sim)
buck.set_max_step(t_pwm / 20)

buck.sim(v_ref=v_ref, v_in=v_in)
t_ode = buck.t
x_ode = buck.x

# --- Results ---
plt.figure(figsize=(10,6))

ax = plt.subplot(2,1,1)
plt.plot(t_gam / 1e-3, x_gam[:, 1], label='GAM')
plt.plot(t_ode / 1e-3, x_ode[:, 1], label='ODE')
plt.grid()
plt.xlabel('Time (ms)')
plt.ylabel('Voltage (V)')

plt.subplot(2,1,2, sharex=ax)
plt.plot(t_gam / 1e-3, x_gam[:, 0], label='GAM')
plt.plot(t_ode / 1e-3, x_ode[:, 0], label='ODE')
plt.grid()
plt.xlabel('Time (ms)')
plt.ylabel('Current (A)')

plt.tight_layout()

