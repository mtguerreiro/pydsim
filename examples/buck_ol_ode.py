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
v_in = 16
v_ref = 8

# Sim time
t_sim = 10e-3

# PWM frequency
f_pwm = 50e3

# Step size for simulation
max_step = 1e-6
dt = 1 / f_pwm / 100

# --- Simulation ---
buck = pyd.peode.Buck(R, L, C)
buck.set_f_pwm(f_pwm)
buck.set_sim_params(dt, t_sim, max_step=max_step)
buck.set_initial_conditions(0, 0)

buck.set_controller(pyd.control.OL, {'u':0.5})
buck.sim(v_ref=v_ref, v_in=v_in)
t = buck.signals.t
x = buck.signals.x

buck_esr = pyd.peode.Buck(R, L, C, Rl=Rl, Rc=Rc, Rds=Rds)
buck_esr.set_f_pwm(f_pwm)
buck_esr.set_sim_params(dt, t_sim, max_step=max_step)
buck_esr.set_initial_conditions(0, 0)

buck_esr.set_controller(pyd.control.OL, {'u':0.5})
buck_esr.sim(v_ref=v_ref, v_in=v_in)
t_esr = buck_esr.signals.t
x_esr = buck_esr.signals.x

# --- Results ---
plt.figure(figsize=(10,6))

ax = plt.subplot(2,1,1)
plt.plot(t / 1e-3, x[:, 1], label='Ideal')
plt.plot(t_esr / 1e-3, x_esr[:, 1], label='Parasitics')
plt.grid()
plt.legend()
plt.xlabel('Time (ms)')
plt.ylabel('Voltage (V)')

plt.subplot(2,1,2, sharex=ax)
plt.plot(t / 1e-3, x[:, 0], label='Ideal')
plt.plot(t_esr / 1e-3, x_esr[:, 0], label='Parasitics')
plt.grid()
plt.xlabel('Time (ms)')
plt.ylabel('Current (A)')

plt.tight_layout()
