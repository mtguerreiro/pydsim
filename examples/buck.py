import pydsim as pyd
import numpy as np
import matplotlib.pyplot as plt
plt.ion()

# --- Input ---
# Circuit components
R = 12
L = 15e-6
C = 10e-6

# Input and reference voltage
v_in = 12
V_ref = 6

# Sim time
t_sim = 10e-3

# PWM period
t_pwm = 1/200e3

# Number of points per cycle
n_pwm = 2000

buck = pyd.pe.Buck(R, L, C)
buck.set_pwm(t_pwm, n_pwm)
buck.set_sim_time(t_sim)
buck.set_v_in(v_in)
buck.set_initial_conditions(0.5, 6)

n = buck.n_cycles
v_ref = 6 * np.ones(n)
#v_ref[:int(n/2)] = 6
#v_ref[int(n/2):] = 9

v_in = 12*np.ones(n)
#v_in[:int(n/2)] = 12
#v_in[int(n/2):] = 9

buck.sim(v_ref=v_ref, v_in=v_in)
t = buck.t
x1 = buck.x


##buck.sim(v_ref=9)
##x2 = buck.x


plt.plot(t, x1[:, 1])

##plt.plot(t, x2[:, 1])

###vc, il, u, u_pwm = pyd.pe.buck(R, L, C, V, V_ref, t_pwm, N_pc, t_sim)
##o, u = pyd.pe.buck(R, L, C, V, V_ref, t_pwm, N_pc, t_sim)
##t = o[:, 0]
##u_pwm = o[:, 1]
##il = o[:, 2]
##vc = o[:, 3]
##
##plt.plot(t, u_pwm)
##plt.plot(t, vc)


##
### --- Sim parameters ---
### Discretization time
##dt = t_pwm / N_pc
##
### Number of sim points and number of cycles
##N_cycles = round(t_sim / t_pwm)
##N = round(t_sim / dt)
##
##print('\nSim parameters:')
##print('N: {:}'.format(N))
##
### --- Model ---
### Circuit model. The state vector is considered as [il, vc]
##Am = np.array([[0, -1/L],
##              [1/C, -1/R/C]])
##
##Bm = np.array([[1/L],
##              [0]])
##
##Cm = np.array([0, 1])
##
### Discretized model
###Ad, Bd, Cd, _, _ = scipy.signal.cont2discrete((Am, Bm, Cm, 0), dt, method='bilinear')
##Ad = np.eye(2) + dt * Am
##Bd = dt * Bm
##
##poles, _ = np.linalg.eig(Am)
##polesd, _ = np.linalg.eig(Ad)
##
##print('Poles of open-loop system (continuous): {:.1f} and {:.1f}'.format(*poles))
##print('Poles of open-loop system (discrete): {:.5f} and {:.5f}'.format(*polesd))
##
### --- Simulation ---
### A function for quantization of a signal
##def quantize(x, res, ref):
##
##    n = int(2 * (x / ref) * (2 ** res - 1))
##    if (n % 2) == 1:
##        n = n + 1
##
##    return int(n / 2)
##
### State vector (il, vc). We create a vector with one more position so it is
### easier to simulate. Anyway, we get rid of this last sample later on.
##x = np.zeros((N + 1, 2))
##
### Error signal
##e = np.zeros((N_cycles, 1))
##
### Control signal
##u = np.zeros((N_cycles, 1))
##
### Control signal converted to voltage through ADC
##u_v = np.zeros((N_cycles, 1))
##
### Control signal applied to the plant (PWM)
##u_pwm = np.zeros((N, 1))
##
### Triangle reference for PWM
##u_t = np.arange(0, V_s, V_s / N_pc)
##
### Control signal applied within switching period
##u_s = np.zeros((N_pc, 1))
##
### Reference signal
##v_ref = V_ref * np.ones((N_cycles, 1))
##
### Converts reference voltage for MCU
##V_ref_mcu = V_ref * (V_s / V)
##
### Quantized signals for MCU
##V_ofs_q = quantize(V_ofs, N_adc, V_adc)
##V_ref_q = quantize(V_ref_mcu, N_adc, V_adc)
##
### Control functions
##def open_loop(ref):
##
##    return ref
##
##
##def proportional(error):
##
##    return Kp * error
##
##
##def pi(u_1, error, error_1):
##    
##    return u_1 + Kp * error + (dt * Ki - Kp) * error_1
##
##
###K1 = 1.001/dt; K2 = 750000;
###a0 = (2-dt*K2); a1 = (dt*K2-1); b0 = 0.0015; b1 = 0; b2 = dt*K1-1
##a0 = 1.7053386; a1 = -0.7053386; b0 = 1.8741544; b1 = -3.0928031; b2 = 1.2759663;
##def two_p_two_z(u_1, u_2, error, error_1, error_2):
##    
##    return a0*u_1 + a1*u_2 + b0*error + b1*error_1 + b2*error_2
##
##
##ii = 0
##_ti = time.time()
### Loops for each switching cycle
##for i in range(N_cycles):
##
##    # --- Conditioning circuit ---
##    # Output voltage
##    v_o = x[ii, 1]
##
##    # Simulates de conditioning circuit. The measured signal is first scaled
##    # to 0-V_s, and then an offset is applied.
##    v_o_c = v_o * (V_s / V) + V_ofs
##    # ------
##    
##    # --- MCU ---
##    # The conditioned signal is now quantized
##    v_o_d = quantize(v_o_c, N_adc, V_adc)
##
##    # The DC component must be removed
##    v_o_d = v_o_d - V_ofs_q
##    
##    # Computes the error
##    e[i] = V_ref_q - v_o_d
##
##    # Computes the control law
##    u[i] = pi(u[i - 1], e[i], e[i - 1])
##    #u[i] = two_p_two_z(u[i - 1], u[i - 2], e[i], e[i - 1], e[i - 2])
##    #u[i] = V_ref_q
##    #u[i] = u[i - 1] + Kp * e[i] + (dt * Ki - Kp) * e[i - 1]
##
##    # Converts the control law to PWM (through a timer in practice)
##    u_v[i] = u[i] / (2**N_adc - 1) * V_adc
##
##    u_s[:] = 0
##    u_s[u_t < u_v[i], 0] = V
##    # ------
##    
##    # --- System's response for one switching cycle ---
##    for j in range(N_pc):
##        x[ii + 1] = Ad @ x[ii] + Bd @ u_s[j]
##        u_pwm[ii] = u_s[j]
##        ii = ii + 1
##    # ------
##    
##il = x[:-1, 0]
##vc = x[:-1, 1]
##
##_tf = time.time()
##print('Sim time: {:.4f} s\n'.format(_tf - _ti))

# --- Plots ---
##t = dt * np.arange(N)
##t_cycle = t_pwm * np.arange(N_cycles)
##
##plt.figure()
##ax = plt.subplot(3, 1, 1)
###plt.plot(t/1e-3, u_pwm, label='PWM')
##plt.plot(t_cycle/1e-3, u, label='Control signal')
##plt.legend()
##plt.grid()
##
##plt.subplot(3, 1, 2, sharex=ax)
##plt.plot(t/1e-3, vc, label='Capacitor voltage')
##plt.plot(t_cycle/1e-3, v_ref, label='Reference')
##plt.legend()
##plt.grid()
##
##plt.subplot(3, 1, 3, sharex=ax)
##plt.plot(t/1e-3, il, label='Inductor current')
##plt.legend()
##plt.grid()
##plt.xlabel('Time (ms)')
##plt.tight_layout()
