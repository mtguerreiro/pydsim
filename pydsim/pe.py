import time
import scipy.signal
import numpy as np

import pydsim.utils as pydutils
import pydsim.control as pydctl

class Buck:

    def __init__(self, R, L, C):

        self.R = R
        self.L = L
        self.C = C

        self.init_params()

        # Continuous model
        self.Am = np.array([[0, -1/L],
                  [1/C, -1/R/C]])
        self.Bm = np.array([[1/L],
                  [0]])
        self.Cm = np.array([0, 1])

    
    def init_params(self):
        self.dt = None
        self.t_pwm = None
        self.n_pwm = None
        self.t_sim = None
        self.n = None
        self.n_cycles = None

    
    def set_pwm(self, t_pwm, n_pwm):
        self.dt = t_pwm / n_pwm
        self.t_pwm = t_pwm
        self.n_pwm = n_pwm
        
        if self.t_sim is not None:
            self.set_sim_time(self.t_sim)
            self.set_vectors()
        
        self.set_model(self.R, self.L, self.C)

    
    def set_sim_time(self, t_sim):
        self.t_sim = t_sim
        self.n = round(self.t_sim / self.dt)
        self.n_cycles = round(self.t_sim / self.t_pwm)
        self.t = self.dt * np.arange(self.n)
        self.set_vectors()

    
    def set_model(self, R, L, C):
        # Continuous and discrete poles
        self.Am = np.array([[0, -1/L],
                  [1/C, -1/R/C]])
        self.Bm = np.array([[1/L],
                  [0]])
        self.Cm = np.array([0, 1])
        
        self.Ad = np.eye(2) + self.dt * self.Am
        self.Bd = self.dt * self.Bm

        self.poles, _ = np.linalg.eig(self.Am)
        self.polesd, _ = np.linalg.eig(self.Ad)        
        print('Poles of open-loop system (continuous): {:.1f} and {:.1f}'.format(*self.poles))
        print('Poles of open-loop system (discrete): {:.5f} and {:.5f}'.format(*self.polesd))


    def set_v_in(self, v_in):
        self.v_in = v_in

    
    def set_vectors(self):
        self.t = self.dt * np.arange(self.n)
        self.x = np.zeros((self.n, 2))
        self.u = np.zeros((self.n, 1))
        self.pwm = np.zeros((self.n, 1))
        self.e = np.zeros((self.n, 1))

    
    def sim(self, v_ref):

        # Loads useful variables
        n = self.n
        n_pwm = self.n_pwm
        n_cycles = self.n_cycles

        # Control
        pi = pydctl.PI(0.3, 50000, self.dt)

        # Vectors
        x = np.zeros((n + 1, 2))
        
        # Triangle reference for PWM
        u_t = np.arange(0, self.v_in, self.v_in / n_pwm)

        # Control signal applied within switching period
        u_s = np.zeros((n_pwm, 1))

        # Reference signal
        v_ref_a = v_ref * np.ones((n_cycles, 1))
    
        ii = 0
        _ti = time.time()
        
        # Loops for each switching cycle
        for i in range(n_cycles):

            # Computes control law
            self.e[ii] = v_ref_a[i] - x[ii, 1]
            
            #self.u[ii:(n_pwm*(ii+1))] = v_ref_a[i]
            self.u[ii:(n_pwm*(ii+1))] = pi.control(self.e[ii])

            u_t = np.arange(0, self.v_in, self.v_in / n_pwm)
            u_s[:] = 0
            u_s[u_t < self.u[ii], 0] = self.v_in
            
            # System's response for one switching cycle
            for j in range(n_pwm):
                x[ii + 1] = self.Ad @ x[ii] + self.Bd @ u_s[j]
                self.pwm[ii] = u_s[j]
                self.e[ii] = v_ref_a[i] - x[ii, 1]
                ii = ii + 1

        self.x = x[:-1, :]
                
        _tf = time.time()
        print('Sim time: {:.4f} s\n'.format(_tf - _ti))
