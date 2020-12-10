import time
import scipy.signal
import numpy as np

import pydsim.utils as pydutils

class Buck:

    def __init__(self, R, L, C, v_in, v_ref, t_pwm, n_pwm, t_sim):

        self.dt = t_pwm / n_pwm
        self.n = round(t_sim / self.dt)
        self.n_cycles = round(t_sim / t_pwm)
        
        self.R = R
        self.L = L
        self.C = C

        self.v_in = v_in
        self.v_ref = v_ref

        self.t_pwm = t_pwm
        self.n_pwm = n_pwm
        self.t_sim = t_sim

        # Continuous model
        self.Am = np.array([[0, -1/L],
                  [1/C, -1/R/C]])
        self.Bm = np.array([[1/L],
                  [0]])
        self.Cm = np.array([0, 1])

        # Discrete model
        self.Ad = np.eye(2) + self.dt * self.Am
        self.Bd = self.dt * self.Bm

        # Continuous and discrete poles
        self.poles, _ = np.linalg.eig(self.Am)
        self.polesd, _ = np.linalg.eig(self.Ad)        
        print('Poles of open-loop system (continuous): {:.1f} and {:.1f}'.format(*self.poles))
        print('Poles of open-loop system (discrete): {:.5f} and {:.5f}'.format(*self.polesd))

        # Initializes output vectors
        self.t = self.dt * np.arange(self.n)
        self.x = np.zeros((self.n, 2))
        self.u = np.zeros((self.n, 1))
        self.pwm = np.zeros((self.n, 1))

    def sim(self):

        # Loads useful variables
        n = self.n
        n_cycles = self.n_cycles
        n_pwm = self.n_pwm

        # States
        x = np.zeros((n + 1, 2))
        
        # Triangle reference for PWM
        u_t = np.arange(0, self.v_in, self.v_in / n_pwm)

        # Control signal applied within switching period
        u_s = np.zeros((n_pwm, 1))

        # Reference signal
        v_ref_a = self.v_ref * np.ones((n_cycles, 1))
    
        ii = 0
        _ti = time.time()
        
        # Loops for each switching cycle
        for i in range(n_cycles):

            # Computes control law
            self.u[ii:(n_pwm*(ii+1))] = v_ref_a[i]
            
            u_s[:] = 0
            u_s[u_t < self.u[ii], 0] = self.v_in
            
            # System's response for one switching cycle
            for j in range(n_pwm):
                x[ii + 1] = self.Ad @ x[ii] + self.Bd @ u_s[j]
                self.pwm[ii] = u_s[j]
                ii = ii + 1

        self.x = x[:-1, :]
                
        _tf = time.time()
        print('Sim time: {:.4f} s\n'.format(_tf - _ti))
    
def buck(R, L, C, v_in, v_ref, t_pwm, n_pwm, t_sim):

    # --- Sim parameters ---
    # Discretization time
    dt = t_pwm / n_pwm

    # Number of sim points and number of cycles
    n = round(t_sim / dt)
    n_cycles = round(t_sim / t_pwm)

    print('\nSim parameters:')
    print('Number of sim points: {:}'.format(n))

    # --- Model ---
    # Circuit model. The state vector is considered as [il, vc]
    Am = np.array([[0, -1/L],
                  [1/C, -1/R/C]])

    Bm = np.array([[1/L],
                  [0]])

    Cm = np.array([0, 1])

    # Discretized model
    #Ad, Bd, Cd, _, _ = scipy.signal.cont2discrete((Am, Bm, Cm, 0), dt, method='bilinear')
    Ad = np.eye(2) + dt * Am
    Bd = dt * Bm

    poles, _ = np.linalg.eig(Am)
    polesd, _ = np.linalg.eig(Ad)

    print('Poles of open-loop system (continuous): {:.1f} and {:.1f}'.format(*poles))
    print('Poles of open-loop system (discrete): {:.5f} and {:.5f}'.format(*polesd))

    # --- Simulation ---
    # State vector (il, vc). We create a vector with one more position so it is
    # easier to simulate. Anyway, we get rid of this last sample later on.
    x = np.zeros((n + 1, 2))

    # Error signal
    e = np.zeros((n_cycles, 1))

    # Control signal
    u = np.zeros((n_cycles, 1))

    # Control signal applied to the plant (PWM)
    u_pwm = np.zeros((n, 1))

    # Triangle reference for PWM
    u_t = np.arange(0, v_in, v_in / n_pwm)

    # Control signal applied within switching period
    u_s = np.zeros((n_pwm, 1))

    # Reference signal
    v_ref_a = v_ref * np.ones((n_cycles, 1))

    ii = 0
    _ti = time.time()
    # Loops for each switching cycle
    for i in range(n_cycles):

        # Computes control law
        u[i] = v_ref_a[i]
        u_s[:] = 0
        u_s[u_t < u[i], 0] = v_in
        
        # System's response for one switching cycle
        for j in range(n_pwm):
            x[ii + 1] = Ad @ x[ii] + Bd @ u_s[j]
            u_pwm[ii] = u_s[j]
            ii = ii + 1

    o = np.zeros((n, x.shape[1] + 2))
    o[:, 0] = dt * np.arange(n)
    o[:, 1] = u_pwm[:, 0]
    o[:, 2:] = x[:-1, :]
    
    _tf = time.time()
    print('Sim time: {:.4f} s\n'.format(_tf - _ti))

    return o, u
