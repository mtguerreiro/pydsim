import pydsim as pyd
import numpy as np
import scipy
import matplotlib.pyplot as plt
plt.ion()

# --- Input ---
# Circuit components
R = 5
L = 10e-6
C = 560e-6

# Input and reference voltage
v_in = 1
v_ref = 0.5

# Sim time
t_sim = 30e-3

# PWM period
t_pwm = 1/200e3

# Number of points per cycle
n_pwm = 200

# --- Simulation ---
buck = pyd.peode.Buck(R, L, C)

##buck.set_pwm(t_pwm, n_pwm)
##buck.set_sim_time(t_sim)
###buck.set_initial_conditions(0.5, 6)
##
##buck.sim(v_ref=v_ref, v_in=v_in)
##t = buck.t
##x = buck.x

Am = buck.Am
Bm = buck.Bm
Cm = buck.Cm

Aa = np.zeros([3, 3])
Aa[:2,:2] = Am
Aa[:2,2] = 50*Bm[:, 0]
Aa[2,:2] = -Cm

Ba = np.zeros([3, 1])
Ba[-1, 0] = 1

def f_x(t, x):
    #x_dot = Am @ x + Bm @ u
    x_dot = Aa @ x + Ba * 0.5
    return x_dot

sol = scipy.integrate.solve_ivp(f_x, [0, 30e-3], [0, 0, 0], max_step=1e-6, vectorized=True)


# --- Results ---
##plt.figure(figsize=(10,6))
##
##ax = plt.subplot(2,1,1)
##plt.plot(t / 1e-3, x[:, 1])
##plt.grid()
##plt.xlabel('Time (ms)')
##plt.ylabel('Voltage (V)')
##
##plt.subplot(2,1,2, sharex=ax)
##plt.plot(t / 1e-3, x[:, 0])
##plt.grid()
##plt.xlabel('Time (ms)')
##plt.ylabel('Current (A)')
##
##plt.tight_layout()
##
