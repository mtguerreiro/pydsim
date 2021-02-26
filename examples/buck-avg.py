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
v_ref = 0.6

# Sim time
t_sim = 20e-3

# --- Simulation ---
buck = pyd.peode.Buck(R, L, C)

##buck.set_pwm(t_pwm, n_pwm)
##buck.set_sim_time(t_sim)
###buck.set_initial_conditions(0.5, 6)
##
##buck.sim(v_ref=v_ref, v_in=v_in)
##t = buck.t
##x = buck.x

# Plant
Am = buck.Am
Bm = buck.Bm
Cm = np.array([buck.Cm])
xm_i = [0, 0]

### Controller
##Ki = 4000
##Kp = 0.05
##N = 1000
##Kd = 0.5
##
##Ae = np.array([[0, 1], [0, -N]])
##Be = np.array([[0], [1]])
##Ce = np.array([[N*Ki, Ki - N**2*Kd]])
##De = Kp + N*Kd
##xc_i = [0, 0]

Ki = 0
Kp = 1
Ae = np.array([[0]])
Be = np.array([[1]])
Ce = np.array([[Ki]])
De = np.array([[Kp]])
xc_i = [0]

r = v_ref

# Aug model
n_p = Am.shape[0]
n_c = Ae.shape[0]

Aa = np.zeros([n_p + n_c, n_p + n_c])
Aa[:n_p,:n_p] = Am - Bm @ Cm * De
Aa[:n_p, n_p:] = Bm @ Ce
Aa[n_p:, :n_p] = -Be @ Cm
Aa[n_p:, n_p:] = Ae

Ba = np.zeros([n_p + n_c, 1])
Ba[:n_p, :] = Bm * De
Ba[n_p:, :] = Be
xm_i.extend(xc_i)

def f_x(t, x):
    #x_dot = Am @ x + Bm @ u
    x_dot = Aa @ x + Ba * r
    return x_dot

sol = scipy.integrate.solve_ivp(f_x, [0, t_sim], xm_i, max_step=1e-6, vectorized=True)


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
