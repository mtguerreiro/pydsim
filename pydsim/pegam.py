import time
import numpy as np
import scipy
import numba

import pydsim.peode as peode
import pydsim.utils as pydutils
import pydsim.control as pydctl
import pydsim.nbutils as pydnb

import pysp
import pynoise


def buck_f(t, x, A, B, u):
    return A @ x + B * u

class Buck:

    def __init__(self, R, L, C):

        self.R = R
        self.L = L
        self.C = C

        self.max_step = 1e-6
        self.init_params()

        # Continuous model
        self.Am = np.array([[0,     -1/L],
                            [1/C,   -1/R/C]])
        self.Bm = np.array([[1/L],
                            [0]])
        self.Cm = np.array([0, 1])

        self.x_ini = np.zeros((1, 2))

    
    def init_params(self):
        self.t_pwm = None
        self.t_sim = None
        self.v_in = None


    def set_pwm(self, t_pwm, n_pwm=None):
        self.t_pwm = t_pwm
        

    def set_max_step(self, max_step):
        self.max_step = max_step
        

    def set_sim_time(self, t_sim):
        self.t_sim = t_sim
        

    def set_harmonics(self, k):
        h = [0]
        h.extend(k)
        self.harmonics = k


    def set_v_in(self, v_in):
        self.v_in = v_in


    def set_initial_conditions(self, il, vc):
        self.x_ini[0, 0] = il
        self.x_ini[0, 1] = vc


    def set_ctlparams(self, params):
        self.ctlparams = params


    def print_run_params(self):
        print('\n---------------------------------------------')
        print('|{:^43}|'.format('Run params'))
        print('---------------------------------------------')
        print('|{:^21}|{:^21.4e}|'.format('Max. step size', self.max_step))
        print('|{:^21}|{:^21}|'.format('Harmonics', str(self.harmonics)))
        print('---------------------------------------------\n')

    def gam_matrices(self, k, f, delta):
        w = 2 * np.pi * f
        R = self.R; L = self.L; C = self.C
        
        A = np.array([[0,       k*w,    -1/L,       0],
                      [-k*w,    0,      0,          -1/L],
                      [1/C,     0,      -1/(R*C),   k*w],
                      [0,       1/C,    -w*k,       -1/(R*C)]])
        
        B = np.array([[1 / (2*np.pi*k*L)],
                      [1 / (2*np.pi*k*L)],
                      [0],
                      [0]])

        C = np.array([[np.sin(2*np.pi*k*delta)],
                      [np.cos(2*np.pi*k*delta) - 1],
                      [0],
                      [0]])

        return A, B, C
    
    
    
    def sim(self, v_ref, v_in=None):

        self.print_run_params()

        # Loads/creates useful variables
        t_pwm = self.t_pwm
        max_step = self.max_step
        t_sim = self.t_sim
        k = self.harmonics

        if v_in is None:
            if self.v_in is None:
                print('Need to define v_in before simulating!')
                return -1
            v_in = self.v_in

        delta = v_ref / v_in
    
        _ti = time.time()
        #print(self.x_ini)
        x0 = (self.x_ini[0, 0], self.x_ini[0, 1])

        # Starts by solving the dc component
        Adc = self.Am
        Bdc = self.Bm

        def f_x_dc(t, x):
            x_dot = Adc @ x + Bdc * delta * v_in
            return x_dot

        sol_dc = scipy.integrate.solve_ivp(f_x_dc, [0, t_sim], x0, max_step=max_step, vectorized=True)
        t = sol_dc.t

        x_h = np.zeros((len(k) + 1, t.shape[0], 2))
        x_h[0, :, 0] = sol_dc.y[0, :]
        x_h[0, :, 1] = sol_dc.y[1, :]

        self.t = t

        # Now solve for each harmonic
        for i, ki in enumerate(k):
            print(ki)
            Ah, Bh, Ch = self.gam_matrices(ki, 1 / t_pwm, delta)
            Bh = Bh * v_in
            x0 = [0, 0, 0, 0]
            def f_h(t, x):
                x_dot = Ah @ x + Bh * Ch
                return x_dot
            sol_h = scipy.integrate.solve_ivp(f_h, [0, t_sim], x0, t_eval=t, max_step=max_step, vectorized=True)

            i_l_ki_re = sol_h.y[0, :]
            i_l_ki_im = sol_h.y[1, :]
            i_l_ki = 2 * (i_l_ki_re * np.cos(ki*2*np.pi/t_pwm*t) - i_l_ki_im * np.sin(ki*2*np.pi/t_pwm*t))

            v_c_ki_re = sol_h.y[2, :]
            v_c_ki_im = sol_h.y[3, :]
            v_c_ki = 2 * (v_c_ki_re * np.cos(ki*2*np.pi/t_pwm*t) - v_c_ki_im * np.sin(ki*2*np.pi/t_pwm*t))

            x_h[i + 1, :, 0] = i_l_ki
            x_h[i + 1, :, 1] = v_c_ki
            
        _tf = time.time()
        print('Sim time: {:.4f} s\n'.format(_tf - _ti))

        self.x_h = x_h
        self.x = np.sum(x_h, axis=0)
