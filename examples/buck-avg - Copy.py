import pydsim as pyd
import numpy as np
import scipy
import matplotlib.pyplot as plt
plt.ion()

# --- Input ---
# Circuit components
R = 10
L = 100e-6
C = 10e-6

# Input and reference voltage
v_in = 10
v_ref = 5

# Sim time
t_sim = 4e-3

# PWM period
t_pwm = 1/200e3


# --- Simulation with diff eqs ---
#buck = pyd.pe.Buck(R, L, C)
#buck.set_pwm(t_pwm, n_pwm)
#buck.set_sim_time(t_sim)
#buck.set_initial_conditions(0.5, 6)

#buck.sim(v_ref=5, v_in=v_in)
#t = buck.t
#x = buck.x


# --- Simulation ---

# DC solution
t_step = 0.2e-6
Adc = np.array([[0,     -1/L],
                [1/C,   -1/(R*C)]])
Bdc = np.array([[1/L],
                [0]])
Cdc = np.array([[0, 1]])
x_dc_i = [0, 0]

delta = lambda t : 0.5
u = lambda t : v_in * 0.5 if t < 2e-3 else v_in * 0.3

def f_x_dc(t, x):
    x_dot = Adc @ x + Bdc @ (v_ref - Cdc @ x)
    return x_dot

sol_dc = scipy.integrate.solve_ivp(f_x_dc, [0, t_sim], x_dc_i, max_step=t_step, vectorized=True)
##
##plt.plot(sol_dc.t, sol_dc.y[1,:])

### 1st harmonic
##f = 200e3; w = 2*np.pi*f
##k = 1
##A_1st = np.array([[0, k*w, -1/L, 0],
##                  [-k*w, 0, 0, -1/L],
##                  [1/C, 0, -1/(R*C), w*k],
##                  [0, 1/C, -w*k, -1/(R*C)]])
##B_1st = np.array([[1/(2*np.pi*k*L)], [1/(2*np.pi*k*L)], [0], [0]])
##x_1st_i = [0, 0, 0, 0]
##
##delta_1st = lambda t : np.array([[v_in * np.sin(2*np.pi*k*0.5)],[v_in * np.cos(2*np.pi*k*0.5)-1], [0], [0]]) if t < 2e-3 else np.array([[v_in * np.sin(2*np.pi*k*0.3)],[v_in * np.cos(2*np.pi*k*0.3)-1], [0], [0]])
##def f_x_1st(t, x):
##    x_dot = A_1st @ x + B_1st * delta_1st(t)
##    return x_dot
##
##sol_1st = scipy.integrate.solve_ivp(f_x_1st, [0, t_sim], x_1st_i, t_eval=sol_dc.t, max_step=t_step, vectorized=True)
##
###plt.plot(sol_1st.t, sol_1st.y[1, :])
##
##i_l_k_re = sol_1st.y[0, :]
##i_l_k_im = sol_1st.y[1, :]
##i_l = 2 * (i_l_k_re * np.cos(k*w*sol_1st.t) - i_l_k_im * np.sin(k*w*sol_1st.t))
##
##v_c_k_re = sol_1st.y[2, :]
##v_c_k_im = sol_1st.y[3, :]
##v_c = 2 * (v_c_k_re * np.cos(k*w*sol_1st.t) - v_c_k_im * np.sin(k*w*sol_1st.t))
##
### 3rd harmonic
##f = 200e3; w = 2*np.pi*f
##k = 3
##A_3rd = np.array([[0, k*w, -1/L, 0],
##                  [-k*w, 0, 0, -1/L],
##                  [1/C, 0, -1/(R*C), w*k],
##                  [0, 1/C, -w*k, -1/(R*C)]])
##B_3rd = np.array([[v_in/(2*np.pi*k*L)], [v_in/(2*np.pi*k*L)], [0], [0]])
##x_3rd_i = [0, 0, 0, 0]
##
##delta_3rd = lambda t : np.array([[np.sin(2*np.pi*k*0.5)],[np.cos(2*np.pi*k*0.5)-1], [0], [0]])
##def f_x_3rd(t, x):
##    x_dot = A_3rd @ x + B_3rd * delta_3rd(t)
##    return x_dot
##
##sol_3rd = scipy.integrate.solve_ivp(f_x_3rd, [0, t_sim], x_3rd_i, t_eval=sol_dc.t, max_step=t_step, vectorized=True)
##
###plt.plot(sol_2nd.t, sol_2nd.y[1, :])
##
##i_l_3_re = sol_3rd.y[0, :]
##i_l_3_im = sol_3rd.y[1, :]
##i_l_3 = 3 * (i_l_3_re * np.cos(k*w*sol_1st.t) - i_l_3_im * np.sin(k*w*sol_1st.t))
##
##v_c_3_re = sol_3rd.y[2, :]
##v_c_3_im = sol_3rd.y[3, :]
##v_c_3 = 2 * (v_c_3_re * np.cos(k*w*sol_1st.t) - v_c_3_im * np.sin(k*w*sol_1st.t))
##
### 5th harmonic
##f = 200e3; w = 2*np.pi*f
##k = 5
##A_5th = np.array([[0, k*w, -1/L, 0],
##                  [-k*w, 0, 0, -1/L],
##                  [1/C, 0, -1/(R*C), w*k],
##                  [0, 1/C, -w*k, -1/(R*C)]])
##B_5th = np.array([[v_in/(2*np.pi*k*L)], [v_in/(2*np.pi*k*L)], [0], [0]])
##x_5th_i = [0, 0, 0, 0]
##
##delta_5th = lambda t : np.array([[np.sin(2*np.pi*k*0.5)],[np.cos(2*np.pi*k*0.5)-1], [0], [0]])
##def f_x_5th(t, x):
##    x_dot = A_5th @ x + B_5th * delta_5th(t)
##    return x_dot
##
##sol_5th = scipy.integrate.solve_ivp(f_x_5th, [0, t_sim], x_5th_i, t_eval=sol_dc.t, max_step=t_step, vectorized=True)
##
###plt.plot(sol_2nd.t, sol_2nd.y[1, :])
##
##i_l_5_re = sol_5th.y[0, :]
##i_l_5_im = sol_5th.y[1, :]
##i_l_5 = 2 * (i_l_5_re * np.cos(k*w*sol_1st.t) - i_l_5_im * np.sin(k*w*sol_1st.t))
##
##v_c_5_re = sol_5th.y[2, :]
##v_c_5_im = sol_5th.y[3, :]
##v_c_5 = 2 * (v_c_5_re * np.cos(k*w*sol_1st.t) - v_c_5_im * np.sin(k*w*sol_1st.t))

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
