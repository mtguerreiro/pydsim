import time
import numpy as np
import scipy
import numba

import pydsim.peode as peode
import pydsim.utils as pydutils
import pydsim.control as pydctl
import pydsim.nbutils as pydnb
import pydsim.data_types as pyddtypes

import pysp
import pynoise


class Buck:

    def __init__(self, R, L, C, f_pwm=None, Rl=0., Rc=0., Rds=0.):

        self.circuit = pyddtypes.TwoPoleCircuit(R, L, C, f_pwm, Rl=Rl, Rc=Rc, Rds=Rds)
        self.model = pyddtypes.BuckModel()
        self.sim_params = pyddtypes.SimParams()
        self.signals = pyddtypes.Signals()

        self.ctlparams = None
        self.ctl = None


    def set_sim_params(self, dt, t_sim, max_step=1e-6):
        self.sim_params._set_dt(dt)
        self.sim_params._set_t_sim(t_sim)
        self.sim_params._set_max_step(max_step)


    def set_f_pwm(self, f_pwm):
        
        self.circuit._set_f_pwm(f_pwm)
            

    def set_initial_conditions(self, il, vc):

        self.signals.x_ini[0] = il
        self.signals.x_ini[1] = vc


    def set_model(self):
        R = self.circuit.R; L = self.circuit.L; C = self.circuit.C
        Rl, Rc, Rds = self.circuit.Rl, self.circuit.Rc, self.circuit.Rds
        t_pwm = 1 / self.circuit.f_pwm
        
        self.model._set_model(R, L, C, dt=t_pwm, Rl=Rl, Rc=Rc, Rds=Rds)

        
    def set_controller(self, ctl, params):

        self.ctl = pydctl.set_controller_buck(self, ctl, params)
        self.ctlparams = params

    
    def _ode_f(self, t, x, A, B, u):
        
        return A @ x + B * u

        
    def sim(self, v_ref, v_in=None, ctl=None, ctl_params=None):

        #  --- Set model and params for simulation ---
        # Circuit params
        R = self.circuit.R; L = self.circuit.L; C = self.circuit.C
        Rl, Rc, Rds = self.circuit.Rl, self.circuit.Rc, self.circuit.Rds
        f_pwm = self.circuit.f_pwm
        t_pwm = 1 / f_pwm

        # Sim params
        dt = self.sim_params.dt
        t_sim = self.sim_params.t_sim
        max_step = self.sim_params.max_step

        # Model
        self.model._set_model(R, L, C, dt=t_pwm, Rl=Rl, Rc=Rc, Rds=Rds)
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
        if ctl is None:
            if self.ctl is None:
                raise ValueError('Controller not defined.')
            ctl = self.ctl
        else:
            ctl = pydctl.set_controller_buck(self, ctl, ctl_params)
            self.ctl = ctl
            self.ctlparams = ctl_params
            
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
                sol = scipy.integrate.solve_ivp(self._ode_f, t_span, x0, args=(Am, Bm, v_in[i]), vectorized=True, max_step=max_step, dense_output=True)
                x0 = (sol.y[0, -1], sol.y[1, -1])
                if i_sw != i_e:
                    t_eval = t[i_s:i_sw]
                    x_eval = sol.sol(t_eval)
                    sig._x[i_s:i_sw, 0] = x_eval[0, :]
                    sig._x[i_s:i_sw, 1] = x_eval[1, :]
                else:
                    t_eval = t[i_s:i_sw + 1]
                    x_eval = sol.sol(t_eval)
                    sig._x[i_s:i_sw + 1, 0] = x_eval[0, :]
                    sig._x[i_s:i_sw + 1, 1] = x_eval[1, :]

            if i_sw != i_e:
                t_span = (t_sw, t_e)
                sol = scipy.integrate.solve_ivp(self._ode_f, t_span, x0, args=(Am, Bm, 0), vectorized=True, max_step=max_step, dense_output=True)
                x0 = (sol.y[0, -1], sol.y[1, -1])
                t_eval = t[i_sw:i_e + 1]
                x_eval = sol.sol(t_eval)
                sig._x[i_sw:i_e + 1, 0] = x_eval[0, :]
                sig._x[i_sw:i_e + 1, 1] = x_eval[1, :]

            #sig._x[i_s:i_e+1, 0] = pynoise.awgn(sig._x[i_s:i_e+1, 0], 40)
            #sig._x[i_s:i_e+1, 1] = pynoise.awgn(sig._x[i_s:i_e+1, 1], 30)
                
        sig.x[:] = sig._x[:-1, :]
                
        _tf = time.time()
        print('Sim time: {:.4f} s\n'.format(_tf - _ti))


class Boost:

    def __init__(self, R, L, C, f_pwm=None):

        self.circuit = pyddtypes.TwoPoleCircuit(R, L, C, f_pwm)
        self.model = pyddtypes.BoostLinModel()
        self.sim_params = pyddtypes.SimParams()
        self.signals = pyddtypes.Signals()

        self.ctlparams = None
        self.ctl = None


    def set_sim_params(self, dt, t_sim, max_step=1e-6):
        self.sim_params._set_dt(dt)
        self.sim_params._set_t_sim(t_sim)
        self.sim_params._set_max_step(max_step)


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
        if type(ctl) is pydctl.FBL:
            L = self.circuit.L
            R = self.circuit.R
            ki = self.ctlparams['ki']
            ctl._set_params(R, L, ki)

        elif type(ctl) is pydctl.LinSFB:
            t_pwm = self.circuit.t_pwm
            A, B, C = self.model.A, self.model.B, self.model.C
            v_in = self.signals.v_in[0]
            poles = params['poles']
            ctl._set_params(A, B, C, poles, v_in, t_pwm)
            
        else:
            v_ref = self.signals.v_ref[0]
            v_in = self.signals.v_in[0]
            d = 1 - v_in / v_ref
            ctl._set_params(d)

        return ctl
    

    def _ode_f(self, t, x, R, L, C, Vi, u):
        il = x[0]
        vc = x[1]

        il_dot = -vc / L * (1 - u) + Vi / L
        vc_dot = il / C * (1 - u) - vc / R / C
        
        return np.array([il_dot, vc_dot])

        
    def sim(self, v_ref, v_in=None, controller=pydctl.OL):

        #  --- Set params for simulation ---
        # Circuit params
        R = self.circuit.R; L = self.circuit.L; C = self.circuit.C
        f_pwm = self.circuit.f_pwm
        t_pwm = 1 / f_pwm

        # Sim params
        dt = self.sim_params.dt
        t_sim = self.sim_params.t_sim
        max_step = self.sim_params.max_step

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

        # --- Sets model ---
        self.model._set_model(R, L, C, v_ref[0], v_in[0], dt)
        Am = self.model.A; Bm = self.model.B; Cm = self.model.C

        # --- Sets signals ---
        sig = self.signals
        sig._set_vectors(dt, t_pwm, t_sim)
        sig.v_in[:] = v_in[:]
        sig.v_ref[:] = v_ref[:]
        sig.x_lp = self.model.x_lp
        sig.u_lp = self.model.u_lp

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
                args = (R, L, C, v_in[i], 1)
                sol = scipy.integrate.solve_ivp(self._ode_f, t_span, x0, args=args, vectorized=True, max_step=max_step, dense_output=True)
                x0 = (sol.y[0, -1], sol.y[1, -1])
                if i_sw != i_e:
                    t_eval = t[i_s:i_sw]
                    x_eval = sol.sol(t_eval)
                    sig._x[i_s:i_sw, 0] = x_eval[0, :]
                    sig._x[i_s:i_sw, 1] = x_eval[1, :]
                else:
                    t_eval = t[i_s:i_sw + 1]
                    x_eval = sol.sol(t_eval)
                    sig._x[i_s:i_sw + 1, 0] = x_eval[0, :]
                    sig._x[i_s:i_sw + 1, 1] = x_eval[1, :]

            if i_sw != i_e:
                t_span = (t_sw, t_e)
                args = (R, L, C, v_in[i], 0)
                sol = scipy.integrate.solve_ivp(self._ode_f, t_span, x0, args=args, vectorized=True, max_step=max_step, dense_output=True)
                x0 = (sol.y[0, -1], sol.y[1, -1])
                t_eval = t[i_sw:i_e + 1]
                x_eval = sol.sol(t_eval)
                sig._x[i_sw:i_e + 1, 0] = x_eval[0, :]
                sig._x[i_sw:i_e + 1, 1] = x_eval[1, :]

            #sig._x[i_s:i_e+1, 0] = pynoise.awgn(sig._x[i_s:i_e+1, 0], 40)
            #sig._x[i_s:i_e+1, 1] = pynoise.awgn(sig._x[i_s:i_e+1, 1], 30)
                
        sig.x[:] = sig._x[:-1, :]
                
        _tf = time.time()
        print('Sim time: {:.4f} s\n'.format(_tf - _ti))
