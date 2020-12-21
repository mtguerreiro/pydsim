import scipy
import pydsim as pyd
import numpy as np
import matplotlib.pyplot as plt
import numba
import time
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
t_sim = 1e-3

# PWM period
t_pwm = 1/200e3

buck = pyd.peode.Buck(R, L, C)
buck.set_pwm(t_pwm, n_pwm=100)
buck.set_sim_time(t_sim)

pi_params = {'ki': 250, 'kp': 0}
buck.set_ctlparams(pi_params)

buck.sim(v_ref, v_in, control='pi')


##n_pwm = 100
##
##n_cycles = int(t_sim / t_pwm)
##dt = t_pwm / n_pwm
##n = round(t_sim / dt)
##
##t = dt * np.arange(n + 1)
##x = np.zeros((n, 2))
##
##buck = pyd.pe.Buck(R, L, C)
##
##A = buck.Am
##B = buck.Bm
##
###@numba.njit()
##def fun_u_0(t, x, u):
##    dx = A @ x
##    return dx
##
###@numba.njit()
##def fun_u_1(t, x, u):
##    dx = A @ x + B * u
##    return dx
##
##
##
##
##_ti = time.time()
##x0 = (0, 0)
### Loops for each switching cycle
##for i in range(n_cycles):
##    #print(i)
##    # Indexes for the initial and final points of this cycle
##    i_i = n_pwm * i
##    i_f = n_pwm * (i + 1)
##
##    # Control law - always between 0 and 1
##    u = 1
##
##    # Initial and final time of this cycle
##    t_i = t[i_i]
##    t_f = t[i_f]
##
##    # Switching instant
##    i_s = round((t_i + t_pwm * u) / dt)
##    t_s = t[i_s]
##
##    if i_s != i_i:
##        t_eval = t[i_i:i_s+1]
##        t_span = (t_eval[0], t_eval[-1])
##        sol = scipy.integrate.solve_ivp(fun_u_1, t_span, x0, t_eval=t_eval, args=(v_in,), vectorized=True, max_step=1e-6)
##        x0 = (sol.y[0, -1], sol.y[1, -1])
##        x[i_i:i_s, 0] = sol.y[0, :-1]
##        x[i_i:i_s, 1] = sol.y[1, :-1]
##
##    if i_s != i_f:
##        t_eval = t[i_s:i_f+1]
##        t_span = (t_eval[0], t_eval[-1])
##        sol = scipy.integrate.solve_ivp(fun_u_0, t_span, x0, t_eval=t_eval, args=(0,), vectorized=True, max_step=1e-6)
##        x0 = (sol.y[0, -1], sol.y[1, -1])
##        x[i_s:i_f, 0] = sol.y[0, :-1]
##        x[i_s:i_f, 1] = sol.y[1, :-1]
##    
##_tf = time.time()
##print('Sol time: {:2f} s'.format(_tf - _ti))

            
##_ti = time.time()
##x0 = (0, 0)
##for i in range(n_cycles):
##    ii = n_pwm * i
##    ie = n_pwm * (i + 1)
##    
##    ti = t_pwm * i
##    tf = t_pwm * (i + 1)
##
##    t_eval = t[ii:int((ii+ie)/2) + 1]
##    t_span = (t_eval[0], t_eval[-1])
##    sol = scipy.integrate.solve_ivp(fun_u_1, t_span, x0, t_eval=t_eval, args=(v_in,), vectorized=True, max_step=10e-8)
##    x0 = (sol.y[0, -1], sol.y[1, -1])
##    x[ii:int((ii+ie)/2), 0] = sol.y[0, :-1]
##    x[ii:int((ii+ie)/2), 1] = sol.y[1, :-1]
##
##    t_eval = t[int((ii+ie)/2):ie + 1]
##    t_span = (t_eval[0], t_eval[-1])
##    sol = scipy.integrate.solve_ivp(fun_u_0, t_span, x0, t_eval=t_eval, args=(v_in,), vectorized=True, max_step=10e-8)
##    x0 = (sol.y[0, -1], sol.y[1, -1])
##    x[int((ii+ie)/2):ie, 0] = sol.y[0, :]
##    x[int((ii+ie)/2):ie, 1] = sol.y[1, :]
##    
##_tf = time.time()
##print('Sol time: {:2f} s'.format(_tf - _ti))







##buck.set_pwm(t_pwm, n_pwm)
##buck.set_sim_time(t_sim)
##buck.set_v_in(v_in)
##buck.set_initial_conditions(0.5, 6)
##
##n = buck.n_cycles
##v_ref = 6 * np.ones(n)
###v_ref[:int(n/2)] = 6
###v_ref[int(n/2):] = 9
##
##v_in = 12*np.ones(n)
##v_in[:int(n/5)] = 12
##v_in[int(n/5):] = 20
##
##buck.sim(v_ref=v_ref, v_in=v_in)
##t = buck.t
##x = buck.x
##
##buck.sim(v_ref=v_ref, v_in=v_in, control='pi')
##t1 = buck.t
##x1 = buck.x
##
##plt.plot(t, x[:, 1])
##plt.plot(t1, x1[:, 1])
