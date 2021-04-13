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

    def __init__(self, R, L, C, f_pwm=None):

        self.circuit = self.__Circuit(R, L, C, f_pwm)
        self.model = self.__Model()
        self.sim_params = self.__SimParams()
        self.signals = self.__Signals()

        self.ctlparams = None
        self.ctl = None


    def set_sim_params(self, dt, t_sim, dt_max=1e-6):
        self.sim_params._set_step(dt)
        self.sim_params._set_t_sim(t_sim)
        self.sim_params._set_max_step(dt_max)


    def set_f_pwm(self, f_pwm):
        
        self.circuit._set_f_pwm(f_pwm)
            

    def set_initial_conditions(self, il, vc):

        self.signals.x_ini[0] = il
        self.signals.x_ini[1] = vc


    def set_ctlparams(self, params):

        self.ctlparams = params

    
    def set_controller(self, controller, params):
        
        # Control
        ctl = controller()
        if type(ctl) is pydctl.PI:
            t_pwm = self.circuit.t_pwm
            kp, ki = params['kp'], params['ki']
            ctl._set_params(kp, ki, t_pwm)
            
        elif type(ctl) is pydctl.PID:
            t_pwm = self.circuit.t_pwm
            ki = params['ki']
            kp = params['kp']
            kd = params['kd']
            N = params['N']
            ctl._set_params(kp, ki, kd, N, t_pwm)   
            
        elif type(ctl) is pydctl.SMPC:
            t_pwm = self.circuit.t_pwm
            A, B, C = self.model.A, self.model.B, self.model.C
            v_in = self.signals.v_in[0]
            n_step = params['n_step']
            alpha, beta, il_max = params['alpha'], params['beta'], params['il_max']
            if 'ref' in params:
                ref = params['ref']
            else:
                ref = None
            ctl._set_params(A, B, C, t_pwm, v_in, n_step, alpha=alpha, beta=beta, il_max=il_max, ref=ref)
            
        elif type(ctl) is pydctl.DMPC:
            t_pwm = self.circuit.t_pwm
            A, B, C = self.model.A, self.model.B, self.model.C
            v_in = self.signals.v_in[0]
            n_c, n_p, r_w = params['n_c'], params['n_p'], params['r_w']
            if 'ref' in params:
                ref = params['ref']
            else:
                ref = None
            ctl._set_params(A, B, C, t_pwm, v_in, n_p, n_c, r_w, ref)

        elif type(ctl) is pydctl.SFB:
            t_pwm = self.circuit.t_pwm
            A, B, C = self.model.A, self.model.B, self.model.C
            v_in = self.signals.v_in[0]
            poles = params['poles']
            ctl._set_params(A, B, C, poles, v_in, t_pwm)

        elif type(ctl) is pydctl.SFB2:
            t_pwm = self.circuit.t_pwm
            A, B, C = self.model.A, self.model.B, self.model.C
            v_in = self.signals.v_in[0]
            poles = params['poles']
            ctl._set_params(A, B, C, poles, v_in, t_pwm)
            
        else:
            v_ref = self.signals.v_ref[0]
            v_in = self.signals.v_in[0]
            d = v_ref / v_in
            ctl._set_params(d)

        return ctl
    
    
    def sim(self, v_ref, v_in=None, controller=pydctl.OL):

        #  --- Set model and params for simulation ---
        # Circuit params
        R = self.circuit.R; L = self.circuit.L; C = self.circuit.C
        f_pwm = self.circuit.f_pwm
        t_pwm = 1 / f_pwm

        # Sim params
        dt = self.sim_params.dt
        t_sim = self.sim_params.t_sim
        dt_max = self.sim_params.dt_max

        # Model
        self.model._set_model(R, L, C, dt)
        Am = self.model.A; Bm = self.model.B; Cm = self.model.C

        # Run params
        n = round(t_sim / dt)
        n_pwm = round(t_pwm / dt)
        n_cycles = round(t_sim / t_pwm)

        # --- Sets reference and input voltage ---
        if type(v_ref) is int or type(v_ref) is float:
            v_ref = v_ref * np.ones(n_cycles)

        if type(v_in) is int or type(v_in) is float:
            v_in = v_in * np.ones(n_cycles)
        elif v_in is None:
            v_in = self.v_in * np.ones(n_cycles)

        # --- Sets signals ---
        sig = self.signals
        sig._set_vectors(dt, t_pwm, t_sim)
        sig.v_in[:] = v_in[:]
        sig.v_ref[:] = v_ref[:]

        sig.x[0, :] = sig.x_ini[:]
        sig._x[0, :] = sig.x_ini[:]

        # We need to create and auxiliary time vector. This is the same
        # as the one in signals, but with 1 higher dimension.
        t = dt * np.arange(n + 1)
        
        # --- Set control ---
        ctl = self.set_controller(controller, self.ctlparams)
        self.ctl = ctl

        # --- Sim ---
        # Triangle reference for PWM
        u_t = np.arange(0, 1, 1 / n_pwm)

        # Control signal applied within switching period. We create this as a
        # 2-D vector so numba can perform dot products.
        u_s = np.zeros((n_pwm, 1))
    
        _ti = time.time()
        x0 = (self.signals.x_ini[0], self.signals.x_ini[1])

        # Loops for each switching cycle
        for i in range(n_cycles):

            # Indexes for the initial and final points of this cycle
            i_s = n_pwm * i
            i_e = n_pwm * (i + 1)

            # Control law - always between 0 and 1
            csig = ctl.meas(sig, i_s, i)
            _u = ctl.control(csig)
            if _u < 0:
                _u = 0
            elif _u > 1:
                _u = 1

            u_s[:] = 0
            u_s[u_t < _u, 0] = v_in[i]
            
            sig.d[i_s:i_e] = _u
            sig.pwm[i_s:i_e] = u_s[:, 0]
            
            # Initial and final time of this cycle
            t_s = t[i_s]
            t_e = t[i_e]

            # Switching instant
            t_sw = t_s + t_pwm * _u
            i_sw = round(t_sw / dt)

            if i_sw != i_s:
                t_span = (t_s, t_sw)
                sol = scipy.integrate.solve_ivp(peode.buck_f, t_span, x0, args=(Am, Bm, v_in[i]), vectorized=True, max_step=dt_max, dense_output=True)
                x0 = (sol.y[0, -1], sol.y[1, -1])
                if i_sw != i_e:
                    t_eval = t[i_s:i_sw]
                    x_eval = sol.sol(t_eval)
                    sig._x[i_s:i_sw, 0] = x_eval[0, :]
                    sig._x[i_s:i_sw, 1] = x_eval[1, :]
                else:
                    t_eval = sig.t[i_s:i_sw + 1]
                    x_eval = sol.sol(t_eval)
                    sig._x[i_s:i_sw + 1, 0] = x_eval[0, :]
                    sig._x[i_s:i_sw + 1, 1] = x_eval[1, :]

            if i_sw != i_e:
                t_span = (t_sw, t_e)
                sol = scipy.integrate.solve_ivp(peode.buck_f, t_span, x0, args=(Am, Bm, 0), vectorized=True, max_step=dt_max, dense_output=True)
                x0 = (sol.y[0, -1], sol.y[1, -1])
                t_eval = t[i_sw:i_e + 1]
                x_eval = sol.sol(t_eval)
                sig._x[i_sw:i_e + 1, 0] = x_eval[0, :]
                sig._x[i_sw:i_e + 1, 1] = x_eval[1, :]
        
        sig.x[:] = sig._x[:-1, :]
                
        _tf = time.time()
        print('Sim time: {:.4f} s\n'.format(_tf - _ti))


    class __Circuit:

        def __init__(self, R, L, C, f_pwm):

            self.R = R
            self.L = L
            self.C = C
            
            if f_pwm is not None:
                self.f_pwm = f_pwm
                self.t_pwm = 1 / f_pwm
            else:
                self.f_pwm = None
                self.t_pwm = None


        def _get_params(self):

            return self.R, self.L, self.C, self.f_pwm
        

        def _set_f_pwm(self, f_pwm):
            
            self.f_pwm = f_pwm
            self.t_pwm = 1 / f_pwm


    class __Signals:

        def __init__(self):

            self.x_ini = np.array([0.0, 0.0])
            self.t = None
            self.t_p = None
            self.x = None
            self._x = None
            self.v_in = None
            self.v_ref = None

            self.d = None
            self.pwm = None

        def _set_vectors(self, dt, t_pwm, t_sim):

            n = round(t_sim / dt)
            self.t = dt * np.arange(n)
            self.x = np.zeros((n, 2))
            self._x = np.zeros((n + 1, 2))
            self.pwm = np.zeros(n)
            self.d = np.zeros(n)

            n_cycles = round(t_sim / t_pwm)
            self.t_p = t_pwm * np.arange(n_cycles)
            self.v_in = np.zeros(n_cycles)
            self.v_ref = np.zeros(n_cycles)
            

    class __Model:

        def __init__(self):

            self.A = None
            self.B = None
            self.C = None

            self.Ad = None
            self.Bd = None
            self.Cd = None

            self.dt = None
            

        def _set_model(self, R, L, C, dt):

            self.dt = dt
            
            A, B, C = self._continuous(R, L, C)
            self.A = A; self.B = B; self.C = C
            
            self.Ad, self.Bd, self.Cd = self._discrete(A, B, C, dt)
            
            
        def _continuous(self, R, L, C):
            
            A = np.array([[0,      -1/L],
                          [1/C,    -1/R/C]])
            
            B = np.array([[1/L],
                          [0]])
            
            C = np.array([0, 1])

            return (A, B, C)
        

        def _discrete(self, A, B, C, dt):
            
            Ad, Bd, Cd, _, _ = scipy.signal.cont2discrete((self.A, self.B, self.C, 0), self.dt, method='bilinear')

            return (Ad, Bd, Cd)


    class __SimParams:

        def __init__(self, dt=None, t_sim=None, dt_max=1e-6):
            
            self.dt = dt
            self.t_sim = t_sim
            self.dt_max = dt_max

            
        def _set_step(self, dt):
            self.dt = dt
            

        def _set_t_sim(self, t_sim):
            self.t_sim = t_sim


        def _set_max_step(self, dt_max):
            self.dt_max = dt_max
