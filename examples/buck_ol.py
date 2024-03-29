import pydsim as pyd
import numpy as np
import matplotlib.pyplot as plt
plt.ion()

# --- Input ---
# Circuit components
R = 2.2
L = 4.7e-6
C = 560e-6

Rl = 15e-3
Rds = 11e-3
Rc = 30e-3

# Input and reference voltage
v_in = 10
v_ref = 6.6

# Sim time
t_sim = 1e-3

# PWM frequency
f_pwm = 200e3

# Step size for simulation
dt = 1 / f_pwm / 100

# --- Simulation ---
buck = pyd.pe.Buck(R, L, C)
buck.set_f_pwm(f_pwm)
buck.set_sim_params(dt, t_sim)
buck.set_initial_conditions(0, 0)

buck.set_controller(pyd.control.OL, {'u':0.2})
buck.sim(v_ref=v_ref, v_in=v_in)
t = buck.signals.t
x = buck.signals.x

buck_esr = pyd.peode.Buck(R, L, C, Rl=Rl, Rc=Rc, Rds=Rds)
buck_esr.set_f_pwm(f_pwm)
buck_esr.set_sim_params(dt, t_sim)
buck_esr.set_initial_conditions(0, 0)

buck_esr.set_controller(pyd.control.OL, {'u':0.2})
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
