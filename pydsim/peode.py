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
        self.Am = np.array([[0, -1/L],
                  [1/C, -1/R/C]])
        self.Bm = np.array([[1/L],
                  [0]])
        self.Cm = np.array([0, 1])

    
    def init_params(self):
        self.t_pwm = None
        self.t_sim = None
        self.n = None
        self.n_cycles = None
        self.dt = None


    def set_max_step(self, max_step):
        self.max_step = max_step

    
    def set_pwm(self, t_pwm, n_pwm=100):
        self.t_pwm = t_pwm
        self.n_pwm = n_pwm
        self.dt = t_pwm / n_pwm
        if self.t_sim is not None:
            self.set_run_params()

    
    def set_sim_time(self, t_sim):
        self.t_sim = t_sim
        if self.t_pwm is not None:
            self.set_run_params()
    
    
    def set_run_params(self):
        self.n = round(self.t_sim / self.dt)
        self.n_cycles = round(self.t_sim / self.t_pwm)
        self.t = self.dt * np.arange(self.n)
        self.set_vectors()
        print('\n---------------------------------------------')
        print('|{:^43}|'.format('Run params'))
        print('---------------------------------------------')
        print('|{:^21}|{:^21.4e}|'.format('Max. step size', self.max_step))
        print('|{:^21}|{:^21}|'.format('Sim cycles', self.n_cycles))
        print('|{:^21}|{:^21}|'.format('State points', self.n))
        print('---------------------------------------------\n')


    def set_v_in(self, v_in):
        self.v_in = v_in


    def set_initial_conditions(self, il, vc):
        self.x[0, 0] = il
        self.x[0, 1] = vc


    def set_ctlparams(self, params):
        self.ctlparams = params

        
    def set_vectors(self):
        self.t = self.dt * np.arange(self.n)
        self.x = np.zeros((self.n, 2))
        self.u = np.zeros((self.n, 1))
        self.pwm = np.zeros((self.n, 1))
        self.e = np.zeros((self.n, 1))
    
    
    def sim(self, v_ref, v_in=None, control='ol'):

        # Loads/creates useful variables
        dt = self.dt
        n = self.n
        t = dt * np.arange(n + 1)
        t_pwm = self.t_pwm
        n_pwm = round(t_pwm / dt)
        n_cycles = self.n_cycles
        max_step = self.max_step

        # Sets v_ref and v_in as vectors if they are numbers
        if type(v_ref) is int or type(v_ref) is float:
            v_ref = v_ref * np.ones(n_cycles)

        if type(v_in) is int or type(v_in) is float:
            v_in = v_in * np.ones(n_cycles)
        elif v_in is None:
            v_in = self.v_in * np.ones(n_cycles)

        # State and control vector
        x = np.zeros((n + 1, 2))
        x[0] = self.x[0]
        u = np.zeros((n, 1))

        # Control
        if control == 'pi':
            ctlparams = self.ctlparams
            ctlparams['dt'] =  t_pwm
            ctlini = {'e_1': v_ref[0] - x[0, 1], 'u_1': v_ref[0] / v_in[0]}
            ctl = pydctl.PI(ctlparams)
            #ctl.set_initial_conditions(ctlini)

        elif control == 'mpc':
            ctlparams = self.ctlparams
            ctlparams['A'] = self.Am
            ctlparams['B'] = self.Bm
            ctlparams['C'] = self.Cm
            ctlparams['dt'] = t_pwm
            ctl = pydctl.MPC(ctlparams)
            self.ctl = ctl

        else:
            ctlparams = {'dc': v_ref[0] / v_in[0]}
            ctlini = {'dc': v_ref[0] / v_in[0]}
            ctl = pydctl.OL(ctlparams)
            ctl.set_initial_conditions(ctlini)
        
        # Triangle reference for PWM
        u_t = np.arange(0, 1, 1 / n_pwm)

        # Control signal applied within switching period
        u_s = np.zeros((n_pwm, 1))

        pwm = np.zeros((n, 1))

        # Reference signal
        #v_ref_a = v_ref * np.ones((n_cycles, 1))
    
        _ti = time.time()
        x0 = (0, 0)
        # Loops for each switching cycle
        for i in range(n_cycles):
            #print(i)
            # Indexes for the initial and final points of this cycle
            i_i = n_pwm * i
            i_f = n_pwm * (i + 1)

            # Control law - always between 0 and 1
            #print('x[i_i]: {:.2f}'.format(x[i_i, 1]))
            _u = ctl.control(x[i_i], v_in[i], v_ref[i])
            u[i_i:i_f, 0] = _u

            # Initial and final time of this cycle
            t_i = t[i_i]
            t_f = t[i_f]

            # Switching instant
            t_s = t_i + t_pwm * _u
            i_s = round(t_s / dt)

            if i_s != i_i:
                t_span = (t_i, t_s)
                sol = scipy.integrate.solve_ivp(peode.buck_f, t_span, x0, args=(self.Am, self.Bm, v_in[i]), vectorized=True, max_step=max_step, dense_output=True)
                x0 = (sol.y[0, -1], sol.y[1, -1])
                if i_s != i_f:
                    t_eval = t[i_i:i_s]
                    x_eval = sol.sol(t_eval)
                    x[i_i:i_s, 0] = x_eval[0, :]
                    x[i_i:i_s, 1] = x_eval[1, :]
                else:
                    t_eval = t[i_i:i_s + 1]
                    x_eval = sol.sol(t_eval)
                    x[i_i:i_s + 1, 0] = x_eval[0, :]
                    x[i_i:i_s + 1, 1] = x_eval[1, :]

            if i_s != i_f:
                t_span = (t_s, t_f)
                sol = scipy.integrate.solve_ivp(peode.buck_f, t_span, x0, args=(self.Am, self.Bm, 0), vectorized=True, max_step=max_step, dense_output=True)
                x0 = (sol.y[0, -1], sol.y[1, -1])
                t_eval = t[i_s:i_f + 1]
                x_eval = sol.sol(t_eval)
                x[i_s:i_f + 1, 0] = x_eval[0, :]
                x[i_s:i_f + 1, 1] = x_eval[1, :]
        
        self.x = x[:-1, :]
        self.u = u
                
        _tf = time.time()
        print('Sim time: {:.4f} s\n'.format(_tf - _ti))
