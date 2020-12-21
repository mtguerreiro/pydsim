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
v_ref = 6.6

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
buck.set_initial_conditions(0.5, 6)

buck.sim(v_ref=v_ref, v_in=v_in)
t = buck.t
x = buck.x

# --- Results ---
plt.figure(figsize=(10,6))

ax = plt.subplot(2,1,1)
plt.plot(t / 1e-3, x[:, 1])
plt.grid()
plt.xlabel('Time (ms)')
plt.ylabel('Voltage (V)')

plt.subplot(2,1,2, sharex=ax)
plt.plot(t / 1e-3, x[:, 0])
plt.grid()
plt.xlabel('Time (ms)')
plt.ylabel('Current (A)')

plt.tight_layout()
