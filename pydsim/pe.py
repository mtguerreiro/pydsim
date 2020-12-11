import time
import numpy as np
import numba

import pydsim.utils as pydutils
import pydsim.control as pydctl
import pydsim.nbutils as pydnb

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
        print('Sim points: {:}'.format(self.n))
        print('Sim cycles: {:}'.format(self.n_cycles))

    
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


    def set_initial_conditions(self, il, vc):
        self.x[0, 0] = il
        self.x[0, 1] = vc

    
    def sim(self, v_ref, v_in=None, control='ol'):

        # Loads useful variables
        n = self.n
        n_pwm = self.n_pwm
        n_cycles = self.n_cycles

        if type(v_ref) is int or type(v_ref) is float:
            v_ref = v_ref * np.ones(n_cycles)

        if v_in is None:
            v_in = self.v_in * np.ones(n_cycles)

        # Vectors
        x = np.zeros((n + 1, 2))
        x[0] = self.x[0]

        # Control
        if control == 'pi':
            ctlparams = {'ki': 1000, 'kp': 0.1, 'dt': n_pwm * self.dt}
            ctlini = {'e_1': v_ref[0] - x[0, 1], 'u_1': v_ref[0] / v_in[0]}
            ctl = pydctl.PI(ctlparams)
        else:
            ctlparams = {'dc': v_ref[0] / v_in[0]}
            ctlini = {'dc': v_ref[0] / v_in[0]}
            ctl = pydctl.OL(ctlparams)
            
        ctl.set_initial_conditions(ctlini)
        
        #pi = pydctl.PI(0.1, 1000, n_pwm * self.dt)
        #u_1 = v_ref[0] / v_in[0]
        #e_1 = v_ref[0] - x[0, 1]
        #pi.set_initial_conditions(u_1, e_1)
        
        # Triangle reference for PWM
        u_t = np.arange(0, 1, 1 / n_pwm)

        # Control signal applied within switching period
        u_s = np.zeros((n_pwm, 1))

        # Reference signal
        #v_ref_a = v_ref * np.ones((n_cycles, 1))
    
        ii = 0
        _ti = time.time()
        
        # Loops for each switching cycle
        for i in range(n_cycles):

            i_s = ii
            i_e = ii + n_pwm

            #self.u[ii:(n_pwm*(ii+1))] = v_ref[i] / self.v_in
            # Computes control law
            e = (v_ref[i] - x[ii, 1]) / v_in[i]
            
            #self.u[ii:(n_pwm*(ii+1))] = v_ref_a[i]
            self.u[i_s:i_e] = ctl.control(e)

            #u_t = np.arange(0, self.v_in, self.v_in / n_pwm)
            u_s[:] = 0
            u_s[u_t < self.u[ii], 0] = v_in[i]

            # System's response for one switching cycle - with numba
            pydnb.sim(x[i_s:i_e, :], self.Ad, self.Bd, u_s, n_pwm)
            ii = ii + n_pwm
            
            # System's response for one switching cycle
            #for j in range(n_pwm):
            #    x[ii + 1] = self.Ad @ x[ii] + self.Bd @ u_s[j]
            #    #self.e[ii] = v_ref_a[i] - x[ii, 1]
            #    ii = ii + 1
            self.pwm[i_s:i_e] = u_s
        
        self.x = x[:-1, :]
                
        _tf = time.time()
        print('Sim time: {:.4f} s\n'.format(_tf - _ti))


    def mpc_sim(self, v_ref, v_in=None, control='ol', beta=0):

        alpha = 1
        #beta = 1
        # Loads useful variables
        n = self.n
        n_pwm = self.n_pwm
        n_cycles = self.n_cycles

        if type(v_ref) is int or type(v_ref) is float:
            v_ref = v_ref * np.ones(n)

        if v_in is None:
            v_in = self.v_in * np.ones((n, 1))

        # Vectors
        x = np.zeros((n + 1, 2))
        x[0] = self.x[0]
    
        _ti = time.time()

        J = np.zeros(n)
        J_u_0 = np.zeros(n)
        J_u_0_0 = np.zeros(n)
        J_u_0_1 = np.zeros(n)
        J_u_1 = np.zeros(n)
        J_u_1_0 = np.zeros(n)
        J_u_1_1 = np.zeros(n)
        
        # Loops time instant
        for i in range(n):

            # Calculates the system output and cost function for u = 0 and u = 1
            x_u_0 = self.Ad @ x[i]
            x_u_0_0 = self.Ad @ x_u_0
            x_u_0_1 = self.Ad @ x_u_0 + self.Bd @ v_in[i]
            
            x_u_1 = self.Ad @ x[i] + self.Bd @ v_in[i]
            x_u_1_0 = self.Ad @ x_u_1
            x_u_1_1 = self.Ad @ x_u_1 + self.Bd @ v_in[i]
            
            #print(x_u_0[1])
            #print(v_ref[i])

            J_u_0[i] = alpha * (v_ref[i] - x_u_0[1]) ** 2
            J_u_0_0[i] = J_u_0[i] + alpha * (v_ref[i] - x_u_0_0[1]) ** 2
            J_u_0_1[i] = J_u_0[i] + alpha * (v_ref[i] - x_u_0_1[1]) ** 2 + beta
            if J_u_0_0[i] < J_u_0_0[i]:
                J_u_0[i] = J_u_0_0[i]
            else:
                J_u_0[i] = J_u_0_1[i]

            J_u_1[i] = alpha * (v_ref[i] - x_u_1[1]) ** 2 + beta
            J_u_1_0[i] = J_u_1[i] + alpha * (v_ref[i] - x_u_1_0[1]) ** 2
            J_u_1_1[i] = J_u_1[i] + alpha * (v_ref[i] - x_u_1_1[1]) ** 2 + beta
            if J_u_1_0[i] < J_u_1_0[i]:
                J_u_1[i] = J_u_1_0[i]
            else:
                J_u_1[i] = J_u_1_1[i]
                
            self.u[i] = 0
            J[i] = J_u_0[i]
            if J_u_1[i] <= J_u_0[i]:
                self.u[i] = v_in[i]
                J[i] = J_u_1[i]

            x[i + 1] = self.Ad @ x[i] + self.Bd @ self.u[i]
            # System's response for one switching cycle - with numba
            #pydnb.sim(x[i_s:i_e, :], self.Ad, self.Bd, u_s, n_pwm)
            #ii = ii + n_pwm
            
            # System's response for one switching cycle
            #for j in range(n_pwm):
            #    x[ii + 1] = self.Ad @ x[ii] + self.Bd @ u_s[j]
            #    #self.e[ii] = v_ref_a[i] - x[ii, 1]
            #    ii = ii + 1
            #self.pwm[i_s:i_e] = u_s
        
        self.x = x[:-1, :]
                
        _tf = time.time()
        print('Sim time: {:.4f} s\n'.format(_tf - _ti))

        return J, J_u_0, J_u_1
