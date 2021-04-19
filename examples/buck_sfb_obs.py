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
v_in_step = -1
v_ref = 5

# Sim time
t_sim = 10e-3

# PWM frequency
f_pwm = 200e3

# Step size for simulation
dt = 1 / f_pwm / 500

# Specs for state feedback
Ts = 0.5e-3
os = 15/100

# --- Simulation ---
buck = pyd.peode.Buck(R, L, C)
buck.set_f_pwm(f_pwm)
buck.set_sim_params(dt, t_sim)
buck.set_initial_conditions(2, 6)

n = round(t_sim * f_pwm)
v_in_p = v_in * np.ones(n)
#v_in_p[n>>1:] = v_in + v_in_step

v_ref_p = v_ref * np.ones(n)
#v_ref_p[n>>1:] = v_ref + v_in_step

zeta = -np.log(os) / np.sqrt(np.pi**2 + (np.log(os))**2)
wn = 4/Ts/zeta
p1 = -zeta * wn + wn * np.sqrt(zeta**2 - 1, dtype=complex)
p2 = np.conj(p1)
p1_o = 5 * p1.real + 1j * p1.imag
#p1_o = 2 * p1
p2_o = np.conj(p1_o)
#p3 = 10 * p1.real

#sfb_params = {'poles': [p1, p2, p3]}
sfb_params = {'poles': [p1, p2], 'poles_o': [p1_o, p2_o]}
buck.set_ctlparams(sfb_params)
buck.sim(v_ref=v_ref_p, v_in=v_in_p, controller=pyd.control.SFB_OBS)
#print(buck.ctl.K_x)

t_sfb = buck.signals.t
x_sfb = buck.signals.x
u_sfb = buck.signals.d

##buck.set_f_pwm(f_pwm / 2)
##buck.sim(v_ref=v_ref, v_in=v_in, controller=pyd.control.SFB)
##t_sfb1 = buck.signals.t
##x_sfb1 = buck.signals.x
##u_sfb1 = buck.signals.d


# --- Results ---
x_obs = buck.ctl.get_sobs()
t_obs = buck.signals.t_p

plt.plot(t_sfb / 1e-3, x_sfb, label='sfb')
plt.plot(t_obs / 1e-3, x_obs, '--')
##plt.figure(figsize=(10,6))
##
##ax = plt.subplot(3,1,1)
##plt.plot(t_sfb / 1e-3, x_sfb[:, 1], label='sfb')
##plt.grid()
##plt.legend()
##plt.xlabel('Time (ms)')
##plt.ylabel('Voltage (V)')
##
##plt.subplot(3,1,2, sharex=ax)
##plt.plot(t_sfb / 1e-3, x_sfb[:, 0], label='sfb')
##plt.grid()
##plt.legend()
##plt.xlabel('Time (ms)')
##plt.ylabel('Current (A)')
##
##plt.subplot(3,1,3, sharex=ax)
##plt.plot(t_sfb / 1e-3, u_sfb, label='sfb')
##plt.grid()
##plt.legend()
##plt.xlabel('Time (ms)')
##plt.ylabel('$u$')
##
##plt.tight_layout()


# --- Test ---
A = buck.model.A
B = buck.model.B
C = buck.model.C

Mo = np.zeros((2,2), dtype=np.float)
Mo[0, :] = C
Mo[1, :] = C @ A

p = np.linalg.eigvals(A)
pobs = np.array([5 * p[0].real + 1j * p[0].imag / 5, 5 * p[0].real - 1j * p[0].imag / 5])

ceq = np.polymul([1, -pobs[0]], [1, -pobs[1]]).real
Phi_d = ceq[0] * A @ A + ceq[1] * A + ceq[2] * np.eye(2)
K_e = Phi_d @ np.linalg.inv(Mo) @ np.array([[0], [1]])
