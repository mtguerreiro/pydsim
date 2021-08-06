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

# PWM frequency
f_pwm = 200e3

# Step size for simulation
dt = 1 / f_pwm / 500

# --- Simulation ---
buck = pyd.pe.Buck(R, L, C)
buck.set_f_pwm(f_pwm)
buck.set_sim_params(dt, t_sim)
buck.set_initial_conditions(0.5, 6)

buck.set_controller(pyd.control.OL, {'u':0.2})
buck.sim(v_ref=v_ref, v_in=v_in)
t = buck.signals.t
x = buck.signals.x

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
