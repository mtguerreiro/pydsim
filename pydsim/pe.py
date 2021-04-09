import time
import numpy as np
import scipy
import numba

import pydsim.utils as pydutils
import pydsim.control as pydctl
import pydsim.nbutils as pydnb

import pysp
import pynoise


class Buck:

    def __init__(self, R, L, C, f_pwm=None):

        self.circuit = self.__Circuit(R, L, C, f_pwm)
        self.model = self.__Model()
        self.sim_params = self.__SimParams()
        self.signals = self.__Signals()

        #self.init_params()

        # Set up filter
        self.filter = None
        #self.filter = self.init_filter()

        #self.x_ini = np.zeros((1, 2))


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

            self.t = None
            self.t_p = None
            self.x = None
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

        def __init__(self, dt=None, t_sim=None):
            
            self.dt = None
            self.t_sim = None

            
        def _set_step(self, dt):
            self.dt = dt
            

        def _set_t_sim(self, t_sim):
            self.t_sim = t_sim


    def set_sim_params(self, dt, t_sim):
        self.sim_params._set_step(dt)
        self.sim_params._set_t_sim(t_sim)


    def set_f_pwm(self, f_pwm):
        
        self.circuit._set_f_pwm(f_pwm)
            
    
##    def init_params(self):
##        self.dt = None
##        self.t_pwm = None
##        self.n_pwm = None
##        self.t_sim = None
##        self.n = None
##        self.n_cycles = None
##
##    
##    def set_pwm(self, t_pwm, n_pwm):
##        self.dt = t_pwm / n_pwm
##        self.t_pwm = t_pwm
##        self.n_pwm = n_pwm
##        
##        if self.t_sim is not None:
##            self.set_sim_time(self.t_sim)
##            self.set_vectors()
##        
##        self.set_model(self.R, self.L, self.C)
##        if self.filter is not None:
##            self.init_filter()
##
##    
##    def set_sim_time(self, t_sim):
##        self.t_sim = t_sim
##        self.n = round(self.t_sim / self.dt)
##        self.n_cycles = round(self.t_sim / self.t_pwm)
##        self.t = self.dt * np.arange(self.n)
##        self.set_vectors()
##        print('\n---------------------------------------------')
##        print('|{:^43}|'.format('Run params'))
##        print('---------------------------------------------')
##        print('|{:^21}|{:^21}|'.format('Sim points', self.n))
##        print('|{:^21}|{:^21}|'.format('Sim cycles', self.n_cycles))
##        print('|{:^21}|{:^21.4e}|'.format('dt', self.dt))
##        print('---------------------------------------------\n')
##
##    
##    def set_model(self, R, L, C):
##        # Continuous and discrete poles
##        self.Am = np.array([[0, -1/L],
##                  [1/C, -1/R/C]])
##        self.Bm = np.array([[1/L],
##                  [0]])
##        self.Cm = np.array([0, 1])
##        
##        self.Ad, self.Bd, self.Cd, _, _ = scipy.signal.cont2discrete((self.Am, self.Bm, self.Cm, 0), self.dt, method='bilinear')
##        #self.Ad = np.eye(2) + self.dt * self.Am
##        #self.Bd = self.dt * self.Bm
##
##        self.poles, _ = np.linalg.eig(self.Am)
##        self.polesd, _ = np.linalg.eig(self.Ad)        
##        print('Poles of open-loop system (continuous): {:.1f} and {:.1f}'.format(*self.poles))
##        print('Poles of open-loop system (discrete): {:.5f} and {:.5f}'.format(*self.polesd))
##
##
##    def set_v_in(self, v_in):
##        self.v_in = v_in
##
##    
##    def set_vectors(self):
##        self.t = self.dt * np.arange(self.n)
##        self.x = np.zeros((self.n, 2))
##        self.u = np.zeros((self.n, 1))
##        self.pwm = np.zeros((self.n, 1))
##        self.e = np.zeros((self.n, 1))
##
##
##    def set_initial_conditions(self, il, vc):
##        #self.x_ini[:] = 0
##        self.x_ini[0, 0] = il
##        self.x_ini[0, 1] = vc
##
##
##    def set_ctl_initial_conditions(self, ctl_params):
##        self.ctl_ini = ctl_params
##        
##    
##    def set_filter(self, fc):
##
##        # Sets the pass band
##        self.__filter_wp = 2 * np.pi * fc
##        self.__filter_Hwp = 0.707
##
##        # Sets the stop band with frequency one decade after fc and
##        # attenuation of 40 dB (for a 2nd order filter)
##        self.__filter_ws = 2 * np.pi * 10 * fc
##        self.__filter_Hws = 0.01
##
##        self.filter = None
##        if self.dt is not None:
##            self.init_filter()
##
##
    def set_ctlparams(self, params):

        self.ctlparams = params
##
##        
##    def init_filter(self):
##        wp = self.__filter_wp
##        Hwp = self.__filter_Hwp
##        ws = self.__filter_ws
##        Hws = self.__filter_Hws
##        
##        self.filter = pysp.filters.butter(wp, Hwp, ws, Hws, T=self.dt, method='bilinear')
##        self.filter_num = self.filter.tfz_sos[0][0]
##        self.filter_den = self.filter.tfz_sos[1][0]

    
    def sim(self, v_ref, v_in=None, control='ol'):

        #  --- Set model and params for simulation ---
        # Circuit params
        R = self.circuit.R; L = self.circuit.L; C = self.circuit.C
        f_pwm = self.circuit.f_pwm
        t_pwm = 1 / f_pwm
        print('\n', R, L, C, f_pwm)

        # Sim params
        dt = self.sim_params.dt
        t_sim = self.sim_params.t_sim
        print('\n', dt, t_sim)

        # Model
        self.model._set_model(R, L, C, dt)
        Ad = self.model.Ad; Bd = self.model.Bd; Cd = self.model.Cd
        Am = self.model.A; Bm = self.model.B; Cm = self.model.C
        print('\n', Am, Bm, Cm)
        print(Ad, Bd, Cd)

        # Run params
        n = round(t_sim / dt)
        n_pwm = round(t_pwm / dt)
        n_cycles = round(t_sim / t_pwm)
        print('\n', n, n_pwm, n_cycles)

        # Signals of the converter
        signals = self.signals
        signals._set_vectors(dt, t_pwm, t_sim)

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

##        # --- Sets vectors ---
##        # Vectors
##        x = np.zeros((n + 1, 2))
##        #x[0] = self.x_ini[0]
##        xfilt = np.zeros((n + 1, 2))
##        u = np.zeros((n, 1))

        # --- Set control ---
        # Control
        if control == 'pi':
            ctlparams = self.ctlparams
            ctlparams['dt'] =  self.t_pwm
            ctlini = {'e_1': v_ref[0] - x[0, 1], 'u_1': v_ref[0] / v_in[0]}
            ctl = pydctl.PI(ctlparams)
            #ctl.set_initial_conditions(ctlini)
            
        elif control == 'pid':
            ctlparams = self.ctlparams
            ctlparams['dt'] =  self.t_pwm
            #ctlini = {'e_1': v_ref[0] - x[0, 1], 'u_1': v_ref[0] / v_in[0]}
            ctl = pydctl.PID(ctlparams)
            if self.x_ini[0, 0] != 0 and self.x_ini[0, 1] != 0:
                ctl.set_initial_conditions(u_1=v_ref[0] / v_in[0], u_2=v_ref[0] / v_in[0])
            self.ctl = ctl
            
        elif control == 'mpc':
            ctlparams = self.ctlparams
            ctlparams['A'] = self.Am
            ctlparams['B'] = self.Bm
            ctlparams['C'] = self.Cm
            ctlparams['dt'] = n_pwm * self.dt
            ctlparams['v_in'] = v_in[0]
            ctl = pydctl.MPC(ctlparams)
            self.ctl = ctl

        elif control == 'dmpc':
            ctlparams = self.ctlparams
            ctlparams['v_in'] = v_in[0]
            ctlparams['A'] = self.Am
            ctlparams['B'] = self.Bm
            ctlparams['C'] = self.Cm
            ctlparams['dt'] = n_pwm * self.dt
            ctl = pydctl.DMPC(ctlparams)
            #ctl.u_1 = 0.5
            #ctl.x_1 = self.x_ini[0]
            self.ctl = ctl

        elif control == 'sfb':
            ctlparams = self.ctlparams
            ctlparams['v_in'] = v_in[0]
            ctlparams['A'] = Am
            ctlparams['B'] = Bm
            ctlparams['C'] = Cm
            ctlparams['dt'] = n_pwm * dt
            ctl = pydctl.SFB(ctlparams)
            self.ctl = ctl
            
        else:
            ctlparams = {'dc': v_ref[0] / v_in[0]}
            ctlini = {'dc': v_ref[0] / v_in[0]}
            ctl = pydctl.OL(ctlparams)
            ctl.set_initial_conditions(ctlini)
        
        #pi = pydctl.PI(0.1, 1000, n_pwm * self.dt)
        #u_1 = v_ref[0] / v_in[0]
        #e_1 = v_ref[0] - x[0, 1]
        #pi.set_initial_conditions(u_1, e_1)

        # --- Sim ---
        # Triangle reference for PWM
        u_t = np.arange(0, 1, 1 / n_pwm)

        # Control signal applied within switching period. We create this as a
        # 2-D vector so numba can perform dot products.
        u_s = np.zeros((n_pwm, 1))
        
        ii = 0
        _ti = time.time()
        
        # Loops for each switching cycle
        for i in range(n_cycles):

            # Indexes for the start and end of this cycle
            i_s = ii
            i_e = ii + n_pwm

            # Computes control law
            if self.filter is None:
                _u = ctl.control(sig._x[ii], sig.v_in[i], sig.v_ref[i])
            else:
                _u = ctl.control(sig._xfilt[ii], sig.v_in[i], sig.v_ref[i])
                
            if _u < 0:
                _u = 0
            elif _u > 1:
                _u = 1
            
            u_s[:] = 0
            u_s[u_t < _u, 0] = v_in[i]

            sig.d[i_s:i_e] = _u
            sig.pwm[i_s:i_e] = u_s[:, 0]

            # System's response for one switching cycle - with numba
            pydnb.sim(sig._x[i_s:i_e, :], Ad, Bd, u_s, n_pwm)
            #sig._x[i_s:i_e+1, 0] = pynoise.awgn(sig._x[i_s:i_e+1, 0], 40)
            #sig._x[i_s:i_e+1, 1] = pynoise.awgn(sig._x[i_s:i_e+1, 1], 30)
            
            # Filters the voltage. Remember that the system's response for one
            # switching cycle gives us the system's output at i_e + 1 (note
            # the dif. equation x[n+1] = Ax[n]...). Thus, we filter up to the
            # i_e + 1 sample.
##            if self.filter is not None:
##                xi = np.array([x[i_s - 1, :], x[i_s - 2, :]])
##                yi = np.array([xfilt[i_s - 1, :], xfilt[i_s - 2, :]])
##                xfilt[i_s:(i_e + 1), :] = pysp.filter_utils.sos_filter(f_tf, x[i_s:(i_e + 1), :], x_init=xi, y_init=yi)
            
            ii = ii + n_pwm

        sig.x[:] = sig._x[:-1, :]
                
        _tf = time.time()
        print('Sim time: {:.4f} s\n'.format(_tf - _ti))
